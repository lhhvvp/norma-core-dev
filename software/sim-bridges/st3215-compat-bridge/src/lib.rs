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
pub(crate) mod preset_loader;
pub(crate) mod state_task;

pub use errors::BridgeError;
// Config types live in station-iface (avoids circular dep).
pub use station_iface::config::{Bridges, St3215CompatBridgeConfig};
