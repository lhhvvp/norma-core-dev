//! `spawn_supervisor` — one tokio task per sim session that waits for
//! the backend to terminate, translates the in-crate `BackendTermination`
//! enum into its proto counterpart, and publishes a final `SimHealth`
//! with `backend_alive = false` so downstream subscribers (web UI,
//! compat bridge) see the failure immediately.

use crate::backend::{BackendTermination, WorldBackend};
use crate::health::HealthPublisher;
use crate::proto::{backend_termination, BackendTermination as BtProto, SimHealth};
use std::sync::Arc;

pub(crate) fn spawn_supervisor(
    backend: Box<dyn WorldBackend>,
    health: Arc<HealthPublisher>,
) {
    tokio::spawn(async move {
        let termination = backend.wait_terminated().await;
        let (cause, exit_code, signal, stderr_tail) = match termination {
            BackendTermination::Clean => {
                (backend_termination::Cause::BtClean as i32, 0, 0, vec![])
            }
            BackendTermination::Crashed {
                exit_code,
                stderr_tail,
            } => (
                backend_termination::Cause::BtCrashed as i32,
                exit_code.unwrap_or(0),
                0,
                stderr_tail,
            ),
            BackendTermination::KilledBySupervisor => (
                backend_termination::Cause::BtKilledBySupervisor as i32,
                0,
                0,
                vec![],
            ),
            BackendTermination::SignaledByOs { signal } => (
                backend_termination::Cause::BtSignaledByOs as i32,
                0,
                signal,
                vec![],
            ),
        };

        let event = SimHealth {
            backend_alive: false,
            termination: Some(BtProto {
                cause,
                exit_code,
                signal,
                stderr_tail,
            }),
            ..Default::default()
        };
        if let Err(e) = health.publish_health(event) {
            log::error!(
                target: "sim_runtime::supervisor",
                "failed to publish final health: {:?}",
                e
            );
        }
    });
}
