//! ST3215 Feetech servo protocol (pure, zero I/O).
//!
//! This crate contains the wire-format parts of the ST3215 driver that are
//! safe to depend on from both the real hardware driver and the
//! simulation compatibility bridge.
//!
//! Architecture invariants (CI-enforced via make check-arch-invariants —
//! see Chunk 8 of the simulation-integration plan):
//!
//! - Zero dependencies on async-runtime crates, persistent-queue crates,
//!   the station interface crate, or the Station engine type.
//! - No I/O, no `async`, no runtime-specific code. Only bytes in,
//!   structs out (and the reverse).
//!
//! Forbidden dependency names are intentionally not spelled out here —
//! the invariant is enforced by a grep-based CI check that would trip
//! on the mention itself.

pub mod layout;
pub mod pack;
pub mod presets;
pub mod register;
pub mod units;
pub mod unpack;

pub use layout::{EEPROM_BYTES, RAM_BYTES, TOTAL_BYTES};
pub use pack::{pack_state_bytes, MotorInstance, MotorSemanticState};
pub use presets::{MotorModelSpec, ST3215_STANDARD};
pub use register::{EepromRegister, RamRegister};
pub use unpack::{unpack_state_bytes, UnpackError};
