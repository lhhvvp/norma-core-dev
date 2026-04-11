//! Compat bridge between the legacy ST3215 byte-level queue surfaces
//! and the generic `SimulationRuntime` API. This crate is the
//! mechanism that makes "existing station clients work unchanged"
//! true for the simulation path.
//!
//! Subsystem boundary: this crate depends on `sim-runtime`'s PUBLIC
//! API (`SimulationRuntime::{subscribe_snapshots, send_actuation,
//! subscribe_health, world_descriptor}`) only — never on
//! `sim-runtime` internals. Adding new bridges in MVP-2 must follow
//! the same discipline.

pub(crate) mod actuator_map;
pub(crate) mod command_task;
pub(crate) mod errors;
pub(crate) mod health_task;
pub mod preset_loader;
pub(crate) mod state_task;

pub use errors::BridgeError;
pub use station_iface::config::{Bridges, St3215CompatBridgeConfig};

use crate::actuator_map::ActuatorMap;
use crate::preset_loader::load_preset;
use normfs::NormFS;
use sim_runtime::SimulationRuntime;
use station_iface::StationEngine;
use std::sync::Arc;

/// Handle returned by `start_st3215_compat_bridge`. Holds the task
/// join handles for graceful shutdown. MVP-1 relies on tokio's
/// task-drop-on-runtime-stop for cleanup; MVP-2 will add explicit
/// cancellation via `tokio_util::sync::CancellationToken`.
pub struct BridgeHandle {
    _state_task: tokio::task::JoinHandle<()>,
    _health_task: tokio::task::JoinHandle<()>,
}

impl BridgeHandle {
    /// Placeholder for MVP-2 graceful shutdown. For now this is a
    /// no-op; dropping the Arc<BridgeHandle> lets the tokio runtime
    /// abort the spawned tasks on its own teardown path.
    pub async fn shutdown(&self) -> Result<(), BridgeError> {
        Ok(())
    }
}

/// Start the st3215-compat bridge against a running `SimulationRuntime`.
///
/// Steps:
///   1. `config.validate()` — enforces legacy_bus_serial sim:// prefix
///   2. `load_preset` — parses the yaml preset file
///   3. Verify the preset's `robot_id` is present in the sim's
///      `WorldDescriptor`
///   4. Verify every preset motor's `actuator_id` is present on
///      that robot in the sim
///   5. Build `ActuatorMap`
///   6. Spawn command_task (inbound legacy command translation)
///   7. Spawn state_task (outbound snapshot → inference bytes)
///   8. Spawn health_task (termination watchdog)
///   9. Return `Arc<BridgeHandle>` owning the task handles
///
/// The `engine` parameter is unused in MVP-1 but is accepted so
/// callers (Chunk 8 Station main) can wire queue registration if
/// MVP-2 decides the bridge owns its own queues (today it writes
/// to queues the real driver owns, relying on the real driver's
/// existing `ensure_queue_exists_for_write`).
pub async fn start_st3215_compat_bridge(
    normfs: Arc<NormFS>,
    engine: Arc<dyn StationEngine>,
    sim_runtime: Arc<SimulationRuntime>,
    config: St3215CompatBridgeConfig,
) -> Result<Arc<BridgeHandle>, BridgeError> {
    config
        .validate()
        .map_err(|e| BridgeError::InvalidConfig(e.into()))?;

    let preset = load_preset(&config.preset_path)?;

    // Verify the sim world contains the expected robot.
    let descriptor = sim_runtime.world_descriptor();
    let robot = descriptor
        .robots
        .iter()
        .find(|r| r.robot_id == config.robot_id)
        .ok_or_else(|| BridgeError::RobotNotInWorld(config.robot_id.clone()))?;

    // Verify every preset actuator is present on that robot.
    for m in &preset.motors {
        if !robot.actuators.iter().any(|a| a.actuator_id == m.actuator_id) {
            return Err(BridgeError::ActuatorNotInWorld(m.actuator_id.clone()));
        }
    }

    let actuator_map = Arc::new(ActuatorMap::from_preset(&preset));

    command_task::spawn_command_task(
        normfs.clone(),
        sim_runtime.clone(),
        actuator_map.clone(),
        config.robot_id.clone(),
        config.legacy_bus_serial.clone(),
    )
    .await?;

    let state_handle = state_task::spawn_state_task(
        normfs.clone(),
        engine.clone(),
        sim_runtime.clone(),
        actuator_map.clone(),
        config.robot_id.clone(),
        config.legacy_bus_serial.clone(),
    )
    .await?;

    let health_handle =
        health_task::spawn_health_task(sim_runtime.clone(), config.legacy_bus_serial.clone())
            .await?;

    log::info!(
        target: "st3215_compat_bridge",
        "bridge started: robot_id={} legacy_bus_serial={} motors={}",
        config.robot_id,
        config.legacy_bus_serial,
        actuator_map.len(),
    );

    Ok(Arc::new(BridgeHandle {
        _state_task: state_handle,
        _health_task: health_handle,
    }))
}
