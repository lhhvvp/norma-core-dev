//! `MockBackend` — an in-process fake `WorldBackend` used by
//! `sim-runtime`'s unit tests in Chunks 3 and 4.
//!
//! The mock does not spawn any subprocess and performs no I/O. It
//! owns a WorldDescriptor and a queue of pre-baked inbound Envelopes;
//! on `start` it returns two fresh mpsc channels, drains its scripted
//! queue into the inbound side, and optionally forwards outbound
//! Envelopes to an observer channel so tests can assert what the
//! runtime sent.

#![cfg(test)]

use super::{BackendStarted, BackendTermination, WorldBackend};
use crate::errors::SimRuntimeError;
use crate::proto::{Envelope, WorldDescriptor};
use async_trait::async_trait;
use std::time::Duration;
use tokio::sync::mpsc;

pub(crate) struct MockBackend {
    pub scripted_inbound: Vec<Envelope>,
    pub descriptor: WorldDescriptor,
    pub outbound_observer: Option<mpsc::Sender<Envelope>>,
}

impl MockBackend {
    pub fn new(descriptor: WorldDescriptor) -> Self {
        Self {
            scripted_inbound: vec![],
            descriptor,
            outbound_observer: None,
        }
    }
}

#[async_trait]
impl WorldBackend for MockBackend {
    async fn start(
        &mut self,
        _startup_timeout: Duration,
    ) -> Result<BackendStarted, SimRuntimeError> {
        let (out_tx, mut out_rx) = mpsc::channel::<Envelope>(256);
        let (in_tx, in_rx) = mpsc::channel::<Envelope>(256);

        // Pre-load the scripted inbound Envelopes.
        for env in self.scripted_inbound.drain(..) {
            let _ = in_tx.send(env).await;
        }

        // Forward outbound to observer if the test wired one up.
        if let Some(obs) = self.outbound_observer.take() {
            tokio::spawn(async move {
                while let Some(env) = out_rx.recv().await {
                    if obs.send(env).await.is_err() {
                        break;
                    }
                }
            });
        }

        Ok(BackendStarted {
            descriptor: self.descriptor.clone(),
            outbound_tx: out_tx,
            inbound_rx: in_rx,
        })
    }

    async fn wait_terminated(self: Box<Self>) -> BackendTermination {
        BackendTermination::Clean
    }

    async fn shutdown(&mut self, _grace: Duration) -> Result<(), SimRuntimeError> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{envelope::Payload, Goodbye, WorldClock};

    fn fake_descriptor() -> WorldDescriptor {
        WorldDescriptor {
            world_name: "mock_world".into(),
            robots: vec![],
            initial_clock: Some(WorldClock {
                world_tick: 0,
                sim_time_ns: 0,
                wall_time_ns: 0,
            }),
            publish_hz: 100,
            physics_hz: 500,
        }
    }

    #[tokio::test]
    async fn test_mock_start_returns_descriptor() {
        let mut mock = MockBackend::new(fake_descriptor());
        let started = mock.start(Duration::from_secs(1)).await.unwrap();
        assert_eq!(started.descriptor.world_name, "mock_world");
    }

    #[tokio::test]
    async fn test_mock_drains_scripted_inbound() {
        let mut mock = MockBackend::new(fake_descriptor());
        mock.scripted_inbound.push(Envelope {
            payload: Some(Payload::Goodbye(Goodbye {
                reason: "scripted".into(),
            })),
        });
        let mut started = mock.start(Duration::from_secs(1)).await.unwrap();
        let env = started.inbound_rx.recv().await.expect("got scripted env");
        match env.payload {
            Some(Payload::Goodbye(g)) => assert_eq!(g.reason, "scripted"),
            other => panic!("expected Goodbye, got {:?}", other),
        }
    }

    #[tokio::test]
    async fn test_mock_outbound_observer() {
        let (obs_tx, mut obs_rx) = mpsc::channel::<Envelope>(4);
        let mut mock = MockBackend::new(fake_descriptor());
        mock.outbound_observer = Some(obs_tx);
        let started = mock.start(Duration::from_secs(1)).await.unwrap();
        let env = Envelope {
            payload: Some(Payload::Goodbye(Goodbye {
                reason: "outbound".into(),
            })),
        };
        started.outbound_tx.send(env).await.unwrap();
        let observed = obs_rx.recv().await.expect("observer saw outbound");
        match observed.payload {
            Some(Payload::Goodbye(g)) => assert_eq!(g.reason, "outbound"),
            other => panic!("expected Goodbye, got {:?}", other),
        }
    }
}
