//! `HealthPublisher` — writes `WorldDescriptor` once and `SimHealth`
//! every 1 Hz to NormFS queues, and mirrors `SimHealth` to in-process
//! broadcast subscribers.
//!
//! Queues (plan §6.5):
//!   - `sim/descriptor` — exactly one WorldDescriptor per session, used
//!     for offline replay of capability-keyed command logs.
//!   - `sim/health`     — periodic SimHealth events for the web UI,
//!     the sim-compat bridge's `health_task`, and external tooling.
//!
//! NormFS API reference (verified against the existing servo-bus driver
//! crate under `software/drivers/`):
//!   - `normfs.resolve(name)` is synchronous, returns a cached QueueId.
//!   - `normfs.ensure_queue_exists_for_write(&qid)` is async, called
//!     during HealthPublisher::new.
//!   - `normfs.enqueue(&qid, Bytes)` is SYNCHRONOUS and returns
//!     `Result<normfs::UintN, normfs::Error>`. Both `publish_descriptor`
//!     and `publish_health` are therefore synchronous.
//!
//! (CI-enforced architecture invariant: this crate must not mention
//! the legacy servo driver literal name. See the check in Chunk 8's
//! `make check-arch-invariants`.)

use crate::errors::SimRuntimeError;
use crate::proto::{SimHealth, WorldDescriptor};
use bytes::Bytes;
use normfs::NormFS;
use prost::Message;
use std::sync::Arc;
use tokio::sync::broadcast;

pub(crate) const SIM_HEALTH_QUEUE_ID: &str = "sim/health";
pub(crate) const SIM_DESCRIPTOR_QUEUE_ID: &str = "sim/descriptor";

pub(crate) struct HealthPublisher {
    normfs: Arc<NormFS>,
    broadcast_tx: broadcast::Sender<SimHealth>,
    session_id: String,
    health_qid: normfs::QueueId,
    descriptor_qid: normfs::QueueId,
}

impl HealthPublisher {
    pub async fn new(
        normfs: Arc<NormFS>,
        session_id: String,
    ) -> Result<Self, SimRuntimeError> {
        let health_qid = normfs.resolve(SIM_HEALTH_QUEUE_ID);
        normfs
            .ensure_queue_exists_for_write(&health_qid)
            .await
            .map_err(|e| SimRuntimeError::NormfsError(format!("health queue: {:?}", e)))?;

        let descriptor_qid = normfs.resolve(SIM_DESCRIPTOR_QUEUE_ID);
        normfs
            .ensure_queue_exists_for_write(&descriptor_qid)
            .await
            .map_err(|e| SimRuntimeError::NormfsError(format!("descriptor queue: {:?}", e)))?;

        let (broadcast_tx, _) = broadcast::channel(64);
        Ok(Self {
            normfs,
            broadcast_tx,
            session_id,
            health_qid,
            descriptor_qid,
        })
    }

    pub fn subscribe(&self) -> broadcast::Receiver<SimHealth> {
        self.broadcast_tx.subscribe()
    }

    /// One-shot write of `WorldDescriptor` to `sim/descriptor` after
    /// handshake. Enables offline replay of archived capability-keyed
    /// commands by downstream tooling that needs to know the world
    /// topology the commands were issued against.
    pub fn publish_descriptor(&self, desc: &WorldDescriptor) -> Result<(), SimRuntimeError> {
        let mut buf = Vec::with_capacity(desc.encoded_len());
        desc.encode(&mut buf)
            .map_err(|e| SimRuntimeError::NormfsError(format!("proto encode: {:?}", e)))?;
        self.normfs
            .enqueue(&self.descriptor_qid, Bytes::from(buf))
            .map_err(|e| SimRuntimeError::NormfsError(format!("{:?}", e)))?;
        Ok(())
    }

    /// Publish a `SimHealth` event. Overwrites `runtime_session_id` with
    /// the HealthPublisher's session id so callers don't need to fill it
    /// in. Fans out to the broadcast channel AND the sim/health queue.
    pub fn publish_health(&self, mut health: SimHealth) -> Result<(), SimRuntimeError> {
        health.runtime_session_id = self.session_id.clone();
        // Broadcast to in-process subscribers; no-op if zero listeners.
        let _ = self.broadcast_tx.send(health.clone());

        let mut buf = Vec::with_capacity(health.encoded_len());
        health
            .encode(&mut buf)
            .map_err(|e| SimRuntimeError::NormfsError(format!("proto encode: {:?}", e)))?;
        self.normfs
            .enqueue(&self.health_qid, Bytes::from(buf))
            .map_err(|e| SimRuntimeError::NormfsError(format!("{:?}", e)))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::WorldClock;
    use normfs::NormFsSettings;
    use tempfile::TempDir;

    async fn make_normfs() -> (Arc<NormFS>, TempDir) {
        let tmp = tempfile::tempdir().expect("tempdir");
        let settings = NormFsSettings::default();
        let fs = NormFS::new(tmp.path().to_path_buf(), settings)
            .await
            .expect("NormFS::new");
        (Arc::new(fs), tmp)
    }

    #[tokio::test]
    async fn test_publish_descriptor_to_queue() {
        let (normfs, _tmp) = make_normfs().await;
        let hp = HealthPublisher::new(normfs.clone(), "test-session".into())
            .await
            .expect("health publisher");

        let desc = WorldDescriptor {
            world_name: "test_world".into(),
            robots: vec![],
            initial_clock: Some(WorldClock {
                world_tick: 0,
                sim_time_ns: 0,
                wall_time_ns: 0,
            }),
            publish_hz: 100,
            physics_hz: 500,
        };
        hp.publish_descriptor(&desc).expect("publish_descriptor");
        // We cannot easily read the raw queue bytes back without more
        // NormFS plumbing (that's a Chunk 5 concern). The invariant under
        // test here is "publish_descriptor returns Ok with no NormFS
        // error"; the round-trip correctness is covered by Task 4.9's
        // integration test via the dispatch loop.
    }

    #[tokio::test]
    async fn test_publish_health_fans_out_to_broadcast() {
        let (normfs, _tmp) = make_normfs().await;
        let hp = HealthPublisher::new(normfs.clone(), "sess1".into())
            .await
            .expect("health publisher");

        let mut rx = hp.subscribe();
        hp.publish_health(SimHealth {
            backend_alive: true,
            world_tick: 42,
            ..Default::default()
        })
        .expect("publish_health");

        let event = rx.recv().await.expect("got health event");
        assert_eq!(event.world_tick, 42);
        assert!(event.backend_alive);
        assert_eq!(event.runtime_session_id, "sess1");
    }

    #[tokio::test]
    async fn test_publish_health_overrides_session_id() {
        let (normfs, _tmp) = make_normfs().await;
        let hp = HealthPublisher::new(normfs.clone(), "master".into())
            .await
            .expect("health publisher");

        let mut rx = hp.subscribe();
        // Caller supplies a different session_id; publish_health must
        // overwrite with its own.
        hp.publish_health(SimHealth {
            runtime_session_id: "caller-supplied".into(),
            ..Default::default()
        })
        .expect("publish_health");

        let event = rx.recv().await.unwrap();
        assert_eq!(event.runtime_session_id, "master");
    }
}
