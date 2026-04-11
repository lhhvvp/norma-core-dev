//! ST3215 byte-layout constants.
//!
//! The ST3215 memory map has two segments (derived from `register.rs`):
//!   - EEPROM: 0x00..0x28 (40 bytes, non-volatile config)
//!   - RAM:    0x28..0x47 (31 bytes, runtime state)
//!   - TOTAL:  71 bytes
//!
//! These constants exist here (not in `register.rs`) so downstream code
//! that only needs the byte boundaries — e.g. `pack.rs`'s buffer sizing
//! and `unpack.rs`'s bounds checks — does not pull in the register enums.
//!
//! `DEFAULT_EEPROM` is the zero baseline that `pack_state_bytes` starts
//! from before overlaying spec + instance values. Bytes not explicitly
//! written by pack remain 0 — this is fine for sim where the compat
//! bridge never reads static EEPROM fields like `Offset` or PID coeffs.

pub const EEPROM_BYTES: usize = 40;
pub const RAM_BYTES: usize = 31;
pub const TOTAL_BYTES: usize = EEPROM_BYTES + RAM_BYTES;

pub const DEFAULT_EEPROM: [u8; EEPROM_BYTES] = [0u8; EEPROM_BYTES];
