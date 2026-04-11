//! Motor model presets: static per-model parameters.
//!
//! A `MotorModelSpec` captures everything that is identical across all
//! physical instances of a given servo model. Per-servo variation
//! (calibration offset, angle limits, torque limit) lives in
//! `MotorInstance` in `pack.rs`.

#[derive(Debug, Clone)]
pub struct MotorModelSpec {
    pub model_number: u16,
    pub firmware_version: u8,
    /// Encoded baud-rate code stored at EEPROM 0x06. The ST3215 uses a
    /// small integer index into its baud table, NOT the raw bps value:
    ///   0 = 1,000,000 bps  (factory default; matches
    ///                       `st3215::protocol::SUPPORTED_BAUD_RATES[0]`)
    ///   1 = 500,000  bps
    ///   2 = 250,000  bps
    ///   3 = 128,000  bps
    ///   4 = 115,200  bps
    ///   5 = 76,800   bps
    ///   6 = 57,600   bps
    ///   7 = 38,400   bps
    pub baud_rate_code: u8,
    pub steps_per_rev: u32,
}

/// Factory-default ST3215 (1 Mbps, model number 777, firmware 10).
///
/// Verified against `software/drivers/st3215/src/protocol/packet.rs:9`
/// (`SUPPORTED_BAUD_RATES[0] = 1_000_000`) and the driver's use of
/// index 0 when opening the serial port (`driver.rs:258`).
pub const ST3215_STANDARD: MotorModelSpec = MotorModelSpec {
    model_number: 777,
    firmware_version: 10,
    baud_rate_code: 0,
    steps_per_rev: 4096,
};
