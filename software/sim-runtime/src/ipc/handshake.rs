//! Client-side handshake: send Hello, wait for Welcome, validate the
//! protocol version, return the backend's `WorldDescriptor`.
//!
//! Failure modes:
//!   - `HandshakeTimeout` — no frame arrived in the configured window
//!   - `IpcClosed` — outbound channel dead or inbound closed cleanly
//!   - `ProtocolMismatch { ours, theirs }` — version disagreement
//!   - `ConfigValidation("expected Welcome")` — peer sent a valid
//!     Envelope but the payload was not the expected `Welcome` variant

use crate::errors::SimRuntimeError;
use crate::proto::{envelope::Payload, Envelope, Hello, Welcome, WorldDescriptor};
use tokio::sync::mpsc;
use tokio::time::{timeout, Duration};

pub(crate) const PROTOCOL_VERSION: u32 = 1;

pub(crate) async fn perform_client_handshake(
    outbound_tx: &mpsc::Sender<Envelope>,
    inbound_rx: &mut mpsc::Receiver<Envelope>,
    client_id: String,
    handshake_timeout: Duration,
) -> Result<WorldDescriptor, SimRuntimeError> {
    let hello = Envelope {
        payload: Some(Payload::Hello(Hello {
            protocol_version: PROTOCOL_VERSION,
            client_role: "sim-runtime".into(),
            client_id,
        })),
    };
    outbound_tx
        .send(hello)
        .await
        .map_err(|_| SimRuntimeError::IpcClosed)?;

    let welcome_env = timeout(handshake_timeout, inbound_rx.recv())
        .await
        .map_err(|_| SimRuntimeError::HandshakeTimeout)?
        .ok_or(SimRuntimeError::IpcClosed)?;

    match welcome_env.payload {
        Some(Payload::Welcome(Welcome {
            protocol_version,
            world: Some(descriptor),
        })) => {
            if protocol_version != PROTOCOL_VERSION {
                return Err(SimRuntimeError::ProtocolMismatch {
                    ours: PROTOCOL_VERSION,
                    theirs: protocol_version,
                });
            }
            Ok(descriptor)
        }
        _ => Err(SimRuntimeError::ConfigValidation(
            "expected Welcome".into(),
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{WorldClock, WorldDescriptor};

    fn fake_descriptor() -> WorldDescriptor {
        WorldDescriptor {
            world_name: "test_world".into(),
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
    async fn test_handshake_happy_path() {
        let (out_tx, mut out_rx) = mpsc::channel::<Envelope>(4);
        let (in_tx, mut in_rx) = mpsc::channel::<Envelope>(4);

        // Client-side task: simulate what the fake backend would do
        // — wait for Hello, reply with Welcome.
        let handshake_fut = async {
            perform_client_handshake(
                &out_tx,
                &mut in_rx,
                "test-client".into(),
                Duration::from_secs(1),
            )
            .await
        };
        let backend_fut = async {
            let hello = out_rx.recv().await.expect("client sent Hello");
            assert!(matches!(hello.payload, Some(Payload::Hello(_))));
            let welcome = Envelope {
                payload: Some(Payload::Welcome(Welcome {
                    protocol_version: PROTOCOL_VERSION,
                    world: Some(fake_descriptor()),
                })),
            };
            in_tx.send(welcome).await.unwrap();
        };
        let (res, _) = tokio::join!(handshake_fut, backend_fut);
        let desc = res.expect("handshake ok");
        assert_eq!(desc.world_name, "test_world");
    }

    #[tokio::test]
    async fn test_handshake_protocol_mismatch() {
        let (out_tx, mut out_rx) = mpsc::channel::<Envelope>(4);
        let (in_tx, mut in_rx) = mpsc::channel::<Envelope>(4);

        let handshake_fut = async {
            perform_client_handshake(
                &out_tx,
                &mut in_rx,
                "test-client".into(),
                Duration::from_secs(1),
            )
            .await
        };
        let backend_fut = async {
            let _ = out_rx.recv().await.unwrap();
            let welcome = Envelope {
                payload: Some(Payload::Welcome(Welcome {
                    protocol_version: 99,
                    world: Some(fake_descriptor()),
                })),
            };
            in_tx.send(welcome).await.unwrap();
        };
        let (res, _) = tokio::join!(handshake_fut, backend_fut);
        assert!(matches!(
            res,
            Err(SimRuntimeError::ProtocolMismatch { ours: 1, theirs: 99 })
        ));
    }

    #[tokio::test]
    async fn test_handshake_timeout() {
        let (out_tx, _out_rx) = mpsc::channel::<Envelope>(4);
        let (_in_tx, mut in_rx) = mpsc::channel::<Envelope>(4);
        let res = perform_client_handshake(
            &out_tx,
            &mut in_rx,
            "test-client".into(),
            Duration::from_millis(20),
        )
        .await;
        assert!(matches!(res, Err(SimRuntimeError::HandshakeTimeout)));
    }
}
