//! Pure parser: ST3215 71-byte memory dump → `MotorSemanticState`.
//!
//! This is the inverse of `pack.rs`. Real hardware reads produce the same
//! byte layout that `pack_state_bytes` synthesises, so a pack → unpack
//! round-trip is the P0 correctness guarantee (see `pack.rs` tests).
//!
//! No I/O, no async, no register I/O — only byte-slice in, struct out.

use crate::layout::TOTAL_BYTES;
use crate::pack::{MotorInstance, MotorSemanticState};
use crate::presets::MotorModelSpec;
use crate::units::{sign_magnitude_to_i16, steps_to_rad, STEPS_PER_REV};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum UnpackError {
    #[error("buffer too short: expected {expected}, got {actual}")]
    TooShort { expected: usize, actual: usize },
    #[error("invalid model number: expected {expected}, got {actual}")]
    ModelMismatch { expected: u16, actual: u16 },
}

/// Decode a 71-byte ST3215 memory dump into a semantic state struct.
///
/// Byte layout (little-endian u16 fields):
///   EEPROM 0x00..0x28 (40 bytes): model, firmware, id, limits, PID, etc.
///   RAM    0x28..0x47 (31 bytes): torque enable, goal, present, moving.
///
/// The caller passes the `spec` (motor model) and `instance` (per-servo
/// calibration); we need `offset_steps` from the instance to translate
/// raw step counts to radians.
pub fn unpack_state_bytes(
    bytes: &[u8],
    spec: &MotorModelSpec,
    instance: &MotorInstance,
) -> Result<MotorSemanticState, UnpackError> {
    if bytes.len() < TOTAL_BYTES {
        return Err(UnpackError::TooShort {
            expected: TOTAL_BYTES,
            actual: bytes.len(),
        });
    }
    let model = u16::from_le_bytes([bytes[0x00], bytes[0x01]]);
    if model != spec.model_number {
        return Err(UnpackError::ModelMismatch {
            expected: spec.model_number,
            actual: model,
        });
    }
    // RAM addresses in the concatenated buffer (EEPROM 0x00..0x28 is first,
    // so RAM bytes start at offset 0x28):
    //   0x28 TorqueEnable (1)
    //   0x2A GoalPosition (2)
    //   0x38 PresentPosition (2)
    //   0x3A PresentSpeed   (2)
    //   0x3F PresentTemperature (1)
    //   0x42 Moving (1)
    let present_pos_raw = u16::from_le_bytes([bytes[0x38], bytes[0x39]]);
    let present_speed_raw = u16::from_le_bytes([bytes[0x3A], bytes[0x3B]]);
    let goal_pos_raw = u16::from_le_bytes([bytes[0x2A], bytes[0x2B]]);

    let position_rad = steps_to_rad(present_pos_raw, instance.offset_steps);
    let velocity_steps = sign_magnitude_to_i16(present_speed_raw);
    let velocity_rad_s =
        (velocity_steps as f32) * (2.0 * std::f32::consts::PI / STEPS_PER_REV as f32);
    let goal_position_rad = steps_to_rad(goal_pos_raw, instance.offset_steps);

    Ok(MotorSemanticState {
        position_rad,
        velocity_rad_s,
        // MVP-1: no meaningful load conversion yet (sim doesn't simulate
        // torque sensing, real driver Present Load is raw mA).
        load_nm: 0.0,
        temperature_c: bytes[0x3F] as f32,
        torque_enabled: bytes[0x28] != 0,
        moving: bytes[0x42] != 0,
        goal_position_rad,
        goal_speed_rad_s: 0.0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::presets::ST3215_STANDARD;

    #[test]
    fn test_unpack_too_short() {
        let err = unpack_state_bytes(
            &[0u8; 10],
            &ST3215_STANDARD,
            &MotorInstance::default_test(),
        )
        .unwrap_err();
        assert!(matches!(err, UnpackError::TooShort { .. }));
    }

    #[test]
    fn test_unpack_model_mismatch() {
        let mut buf = vec![0u8; TOTAL_BYTES];
        buf[0x00] = 0xAA;
        buf[0x01] = 0x00; // model = 0x00AA (not 777)
        let err = unpack_state_bytes(&buf, &ST3215_STANDARD, &MotorInstance::default_test())
            .unwrap_err();
        assert!(matches!(err, UnpackError::ModelMismatch { .. }));
    }
}
