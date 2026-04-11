//! SimulationRuntime: a first-class simulation subsystem for Station.
//!
//! Architecture invariant (CI-enforced via make check-arch-invariants —
//! see Chunk 8 of the simulation-integration plan): this crate has ZERO
//! references to the legacy servo driver's literal name. Capability-keyed
//! schema is the only vocabulary used inside.

pub mod clock;
pub mod errors;

// These become visible to outside crates (e.g. the compat bridge) only via
// the public `SimulationRuntime` struct that Chunk 4 Task 4.8 adds. The
// module declarations are added in Chunk 3 Task 3.1 so the crate compiles
// with stubs; each Chunk 4 task fills in the actual implementation.
pub(crate) mod actuation_sender;
pub(crate) mod health;
pub(crate) mod registry;
pub(crate) mod runtime;
pub(crate) mod snapshot_broker;
pub(crate) mod supervisor;

pub(crate) mod backend;
pub(crate) mod ipc;

#[allow(clippy::all, non_snake_case)]
pub mod proto;

pub use errors::SimRuntimeError;
pub use runtime::SimulationRuntime;
// Config types live in station-iface to avoid circular crate deps; the
// re-export below is un-commented once Task 3.2 adds them to that crate.
// pub use station_iface::config::{LogCapture, SimMode, SimRuntimeConfig};
