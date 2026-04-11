mod devices;
mod error;
mod packet;

pub use devices::is_st3215_usbdevice;
pub use error::*;
pub use packet::*;

// Protocol types previously lived in `memory.rs` and `units.rs` and have
// been migrated to the standalone `st3215-wire` crate (Chunk 2 of the
// simulation-integration plan). Re-export them verbatim so existing
// consumers (`port.rs`, `port_meta.rs`, `auto_calibrate/*`) keep compiling
// with no source changes beyond the import path they already use
// (`crate::protocol::{EepromRegister, RamRegister, normal_position, ...}`).
pub use st3215_wire::layout::{EEPROM_BYTES, RAM_BYTES, TOTAL_BYTES};
pub use st3215_wire::units::*;
pub use st3215_wire::{EepromRegister, RamRegister};

#[cfg(test)]
mod packet_test;
