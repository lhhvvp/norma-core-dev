//! Health monitor task.
//!
//! Subscribes to `SimulationRuntime::subscribe_health` and watches
//! for `backend_alive == false`. When the sim backend terminates
//! (clean or crash), logs an error and exits — MVP-1 stops short
//! of writing a synthetic offline marker to `st3215/meta` because
//! the exact format requires cross-referencing the legacy
//! `St3215Bus` shape, which is the existing driver's domain. The
//! legacy-bus-serial that never reappears is signal enough for
//! clients today; a proper offline marker is queued for MVP-2.

use crate::errors::BridgeError;
use sim_runtime::SimulationRuntime;
use std::sync::Arc;

pub async fn spawn_health_task(
    sim_runtime: Arc<SimulationRuntime>,
    legacy_bus_serial: String,
) -> Result<tokio::task::JoinHandle<()>, BridgeError> {
    let mut health_rx = sim_runtime.subscribe_health();
    let handle = tokio::spawn(async move {
        loop {
            match health_rx.recv().await {
                Ok(health) => {
                    if !health.backend_alive {
                        log::error!(
                            target: "st3215_compat_bridge::health_task",
                            "sim backend terminated for bus {} (termination={:?})",
                            legacy_bus_serial,
                            health.termination
                        );
                        // TODO (MVP-2): write an offline marker to
                        // st3215/meta so legacy clients can detect
                        // the transition without heuristics.
                        return;
                    }
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                    log::warn!(
                        target: "st3215_compat_bridge::health_task",
                        "health subscriber lagged, dropped {} events", n
                    );
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    log::info!(
                        target: "st3215_compat_bridge::health_task",
                        "health channel closed for bus {}", legacy_bus_serial
                    );
                    return;
                }
            }
        }
    });
    Ok(handle)
}
