// Task 2.1 stub — byte-layout constants for ST3215 memory map.
// Replaced in Task 2.2 with the real values + DEFAULT_EEPROM table.

pub const EEPROM_BYTES: usize = 40;
pub const RAM_BYTES: usize = 31;
pub const TOTAL_BYTES: usize = EEPROM_BYTES + RAM_BYTES;
pub const DEFAULT_EEPROM: [u8; EEPROM_BYTES] = [0u8; EEPROM_BYTES];
