//! `ActuationSender` — QoS-aware forwarding of `ActuationBatch`es to the
//! backend's outbound envelope channel.
//!
//! Two lanes (plan §6.4):
//!
//! - **Lossy setpoint lane** (`QOS_LOSSY_SETPOINT`): continuous
//!   position/velocity targets. Channel capacity 256; on full, new
//!   batches are **dropped** (drop-newest via `try_send`) and a warning
//!   is logged. Matches the real-time control convention that a fresh
//!   setpoint obsoletes a stale one.
//!
//! - **Reliable control lane** (`QOS_RELIABLE_CONTROL`): discrete
//!   actions (torque enable/disable, reset, one-shot commands). Channel
//!   capacity 32; senders **await backpressure** so discrete actions are
//!   never silently lost.
//!
//! Unrecognised lanes (`QOS_UNSPECIFIED`) are rejected at the send
//! boundary — callers must set a lane explicitly.

use crate::errors::SimRuntimeError;
use crate::proto::{envelope::Payload, ActuationBatch, Envelope, QosLane};
use tokio::sync::mpsc;

pub(crate) struct ActuationSender {
    lossy_tx: mpsc::Sender<ActuationBatch>,
    reliable_tx: mpsc::Sender<ActuationBatch>,
}

impl ActuationSender {
    pub fn new(outbound: mpsc::Sender<Envelope>) -> Self {
        let (lossy_tx, mut lossy_rx) = mpsc::channel::<ActuationBatch>(256);
        let (reliable_tx, mut reliable_rx) = mpsc::channel::<ActuationBatch>(32);

        let out_lossy = outbound.clone();
        tokio::spawn(async move {
            while let Some(batch) = lossy_rx.recv().await {
                let env = Envelope {
                    payload: Some(Payload::Actuation(batch)),
                };
                if out_lossy.send(env).await.is_err() {
                    break;
                }
            }
        });

        let out_reliable = outbound;
        tokio::spawn(async move {
            while let Some(batch) = reliable_rx.recv().await {
                let env = Envelope {
                    payload: Some(Payload::Actuation(batch)),
                };
                if out_reliable.send(env).await.is_err() {
                    break;
                }
            }
        });

        Self { lossy_tx, reliable_tx }
    }

    pub async fn send(&self, batch: ActuationBatch) -> Result<(), SimRuntimeError> {
        // Prost 0.12 generates `TryFrom<i32>` for enumerations. Mirrors the
        // pattern used by st3215/src/state.rs:306 which does
        // `St3215SignalType::try_from(envelope.signal_type)`.
        match QosLane::try_from(batch.lane).unwrap_or(QosLane::QosUnspecified) {
            QosLane::QosLossySetpoint => match self.lossy_tx.try_send(batch) {
                Ok(()) => Ok(()),
                Err(mpsc::error::TrySendError::Full(_)) => {
                    log::warn!(
                        target: "sim_runtime::actuation",
                        "lossy lane full, dropping batch (drop-newest)"
                    );
                    Ok(())
                }
                Err(mpsc::error::TrySendError::Closed(_)) => Err(SimRuntimeError::IpcClosed),
            },
            QosLane::QosReliableControl => self
                .reliable_tx
                .send(batch)
                .await
                .map_err(|_| SimRuntimeError::IpcClosed),
            QosLane::QosUnspecified => Err(SimRuntimeError::ConfigValidation(
                "batch.lane must be set".into(),
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{envelope::Payload, ActuationBatch};

    fn make_batch(lane: QosLane) -> ActuationBatch {
        ActuationBatch {
            as_of: None,
            commands: vec![],
            lane: lane as i32,
        }
    }

    #[tokio::test]
    async fn test_reliable_lane_roundtrip() {
        let (out_tx, mut out_rx) = mpsc::channel::<Envelope>(4);
        let sender = ActuationSender::new(out_tx);
        sender
            .send(make_batch(QosLane::QosReliableControl))
            .await
            .unwrap();
        let env = out_rx.recv().await.expect("envelope arrived");
        assert!(matches!(env.payload, Some(Payload::Actuation(_))));
    }

    #[tokio::test]
    async fn test_lossy_lane_roundtrip() {
        let (out_tx, mut out_rx) = mpsc::channel::<Envelope>(4);
        let sender = ActuationSender::new(out_tx);
        sender
            .send(make_batch(QosLane::QosLossySetpoint))
            .await
            .unwrap();
        let env = out_rx.recv().await.expect("envelope arrived");
        assert!(matches!(env.payload, Some(Payload::Actuation(_))));
    }

    #[tokio::test]
    async fn test_unspecified_lane_rejected() {
        let (out_tx, _out_rx) = mpsc::channel::<Envelope>(4);
        let sender = ActuationSender::new(out_tx);
        let res = sender.send(make_batch(QosLane::QosUnspecified)).await;
        assert!(matches!(res, Err(SimRuntimeError::ConfigValidation(_))));
    }

    #[tokio::test]
    async fn test_lossy_drop_on_full() {
        // Outbound capacity 1, no consumer: the inbound lossy task will
        // enqueue one batch into outbound (cap 1), then block on its send,
        // so subsequent batches accumulate in the lossy channel (cap 256)
        // and eventually saturate. We flood many more to force drops.
        let (out_tx, _out_rx) = mpsc::channel::<Envelope>(1);
        let sender = ActuationSender::new(out_tx);

        // Every call returns Ok even after the channel is full — drops
        // are silent (drop-newest with warn log).
        for _ in 0..1000 {
            sender
                .send(make_batch(QosLane::QosLossySetpoint))
                .await
                .unwrap();
        }
        // Reaching here without error means the drop-newest path fired
        // and send() kept returning Ok as designed.
    }
}
