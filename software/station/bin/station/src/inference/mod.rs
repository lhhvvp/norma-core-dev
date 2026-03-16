use std::sync::Arc;
use bytes::Bytes;
use dashmap::DashMap;
use parking_lot::{Mutex, Condvar};
use prost::Message;
use normfs::NormFS;
use tokio::sync::mpsc;
use normfs::UintN;
use crate::station_proto::inference::{InferenceRx, inference_rx};

const QUEUE_ID: &str = "inference-states";

pub type InferenceSignal = Arc<(Mutex<bool>, Condvar)>;

pub struct Inference {
    shutdown_tx: mpsc::UnboundedSender<()>,
    normfs: Arc<NormFS>,
    // Track latest pointer per queue (queue_id -> (pointer, data_type))
    latest_pointers: Arc<DashMap<String, (UintN, i32)>>,
    signal: InferenceSignal,
}

impl Inference {
    pub fn shutdown(&self) {
        let _ = self.shutdown_tx.send(());

        // Signal worker thread to wake up and exit
        let (lock, cvar) = &*self.signal;
        let mut signaled = lock.lock();
        *signaled = true;
        cvar.notify_one();
    }

    pub async fn start_queue(normfs: &Arc<NormFS>) -> Result<(), normfs::Error> {
        let queue_id = normfs.resolve(QUEUE_ID);
        normfs.ensure_queue_exists_for_write(&queue_id).await?;

        Ok(())
    }

    pub fn start(
        normfs: Arc<NormFS>,
    ) -> Self {
        let (shutdown_tx, mut shutdown_rx) = mpsc::unbounded_channel();
        let signal = Arc::new((Mutex::new(false), Condvar::new()));

        let latest_pointers: Arc<DashMap<String, (UintN, i32)>> = Arc::new(DashMap::new());

        let queue_id = normfs.resolve(QUEUE_ID);

        let worker_signal = signal.clone();
        let worker_normfs = normfs.clone();
        let worker_pointers = latest_pointers.clone();
        let worker_queue_id = queue_id.clone();

        tokio::task::spawn_blocking(move || {
            let (lock, cvar) = &*worker_signal;
            let mut signaled = lock.lock();

            loop {
                // Wait for signal
                while !*signaled {
                    cvar.wait(&mut signaled);
                }

                // Check if shutdown requested
                if shutdown_rx.try_recv().is_ok() {
                    break;
                }

                // Clear signal flag (new updates during rebuild will set it again)
                *signaled = false;

                // Release lock while we work
                drop(signaled);

                // Snapshot all latest pointers
                let entries: Vec<inference_rx::Entry> = worker_pointers.iter()
                    .map(|entry| inference_rx::Entry {
                        queue: entry.key().clone(),
                        r#type: entry.value().1,
                        ptr: entry.value().0.value_to_bytes(),
                    })
                    .collect();

                if !entries.is_empty() {
                    // Publish complete snapshot to inference-states
                    let rx = InferenceRx {
                        entries,
                        local_stamp_ns: systime::get_local_stamp_ns(),
                        monotonic_stamp_ns: systime::get_monotonic_stamp_ns(),
                        app_start_id: systime::get_app_start_id(),
                    };

                    match worker_normfs.enqueue(&worker_queue_id, Bytes::from(rx.encode_to_vec())) {
                        Ok(_) => {
                        }
                        Err(e) => {
                            log::error!("Failed to enqueue inference state to NormFS: {:?}", e);
                        }
                    }
                }

                // Re-acquire lock for next iteration
                signaled = lock.lock();
            }

            log::info!("Inference worker thread exiting");
        });

        Self {
            shutdown_tx,
            normfs,
            latest_pointers,
            signal,
        }
    }

    pub fn register_queue(&self, queue_id: &normfs::QueueId, queue_data_type: i32) {
        let queue_path = queue_id.as_str().to_string();
        let latest_pointers = self.latest_pointers.clone();
        let signal = self.signal.clone();

        log::info!("Inference: registering queue '{}' with type {}", queue_id, queue_data_type);

        let callback = Box::new(move |entries: &[(UintN, Bytes)]| {
            if let Some((id, _bytes)) = entries.last() {
                latest_pointers.insert(queue_path.clone(), (id.clone(), queue_data_type));

                // Signal worker thread to rebuild and publish
                let (lock, cvar) = &*signal;
                let mut signaled = lock.lock();
                *signaled = true;
                cvar.notify_one();
            }
            true // Continue subscription
        });

        match self.normfs.subscribe(queue_id, callback) {
            Ok(subscriber_id) => {
                log::info!("Inference: subscribed to queue '{}' with subscriber_id {}", queue_id, subscriber_id);
            }
            Err(e) => {
                log::error!("Failed to subscribe to queue '{}': {:?}", queue_id, e);
            }
        }
    }
}
