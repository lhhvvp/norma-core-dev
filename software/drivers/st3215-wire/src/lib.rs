//! ST3215 Feetech servo protocol (pure, zero I/O).
//!
//! This crate contains the wire-format parts of the ST3215 driver that are
//! safe to depend on from both the real hardware driver (`st3215`) and the
//! simulation compatibility bridge (`st3215-compat-bridge`).
//!
//! Architecture invariants (CI-enforced):
//!   - No `tokio`, `normfs`, `station_iface`, or `StationEngine` dependencies
//!   - No I/O, async, or runtime-specific code

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
