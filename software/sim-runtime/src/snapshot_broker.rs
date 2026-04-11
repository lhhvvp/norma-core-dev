//! `SnapshotBroker` — fan-out of `WorldSnapshot`s to multiple in-process
//! subscribers via a tokio broadcast channel.
//!
//! Snapshots are wrapped in `Arc` so subscribers share the underlying
//! payload without deep copies. Slow receivers lag (or Lag-error) per the
//! standard tokio broadcast semantics; this is acceptable for snapshots,
//! which are the "lossy, latest-wins" lane of the subsystem.

use crate::proto::WorldSnapshot;
use std::sync::Arc;
use tokio::sync::broadcast;

pub(crate) struct SnapshotBroker {
    tx: broadcast::Sender<Arc<WorldSnapshot>>,
}

impl SnapshotBroker {
    pub fn new(capacity: usize) -> Self {
        let (tx, _) = broadcast::channel(capacity);
        Self { tx }
    }

    pub fn publish(&self, snapshot: WorldSnapshot) {
        // `send` returns Err when there are no active receivers — that is
        // acceptable here (a snapshot without a listener is a no-op).
        let _ = self.tx.send(Arc::new(snapshot));
    }

    pub fn subscribe(&self) -> broadcast::Receiver<Arc<WorldSnapshot>> {
        self.tx.subscribe()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_broker_multi_subscriber() {
        let broker = SnapshotBroker::new(16);
        let mut a = broker.subscribe();
        let mut b = broker.subscribe();

        let snap = WorldSnapshot {
            clock: None,
            actuators: vec![],
            sensors: vec![],
        };
        broker.publish(snap);

        let a_got = a.recv().await.unwrap();
        let b_got = b.recv().await.unwrap();
        assert!(a_got.actuators.is_empty());
        assert!(b_got.actuators.is_empty());
    }

    #[tokio::test]
    async fn test_publish_without_subscribers_is_noop() {
        let broker = SnapshotBroker::new(16);
        // No subscribers — publish must not panic.
        broker.publish(WorldSnapshot {
            clock: None,
            actuators: vec![],
            sensors: vec![],
        });
    }
}
