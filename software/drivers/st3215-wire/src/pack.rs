//! Pure packer: `MotorSemanticState` → 71-byte ST3215 memory dump.
//!
//! This is the inverse of `unpack.rs`. Used by the simulation compatibility
//! bridge (`st3215-compat-bridge`, Chunk 7) to synthesise byte-level
//! ST3215 state queue payloads from semantic sim snapshots, so existing
//! Station clients (which speak raw ST3215 bytes) work unchanged.
//!
//! The real `st3215` driver does NOT call `pack_state_bytes` — on real
//! hardware, state bytes come directly from the servo. Pack exists solely
//! for the sim path.

use crate::layout::{DEFAULT_EEPROM, RAM_BYTES, TOTAL_BYTES};
use crate::presets::MotorModelSpec;
use crate::units::{i16_to_sign_magnitude, rad_to_steps, STEPS_PER_REV};
use bytes::{Bytes, BytesMut, BufMut};

/// Per-servo calibration and static parameters.
///
/// Stored alongside the model `MotorModelSpec` because different physical
/// servos of the same model may have different offsets, angle limits, or
/// nominal voltages.
#[derive(Debug, Clone)]
pub struct MotorInstance {
    pub min_angle_steps: u16,
    pub max_angle_steps: u16,
    pub offset_steps: i16,
    pub torque_limit: u16,
    pub voltage_nominal_v: f32,
}

#[cfg(test)]
impl MotorInstance {
    /// Test fixture: a centered-range motor with typical calibration.
    /// Used by `pack.rs` and `unpack.rs` unit tests.
    pub fn default_test() -> Self {
        Self {
            min_angle_steps: 0,
            max_angle_steps: 4095,
            offset_steps: 2048,
            torque_limit: 500,
            voltage_nominal_v: 12.0,
        }
    }
}

/// Semantic runtime state of a single ST3215 motor.
///
/// All angles are in **radians** relative to the instance's
/// `offset_steps`-defined zero point, velocities in rad/s.
#[derive(Debug, Clone, Copy, Default)]
pub struct MotorSemanticState {
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub load_nm: f32,
    pub temperature_c: f32,
    pub torque_enabled: bool,
    pub moving: bool,
    pub goal_position_rad: f32,
    pub goal_speed_rad_s: f32,
}

/// Serialise a motor's full state (EEPROM + RAM) into a 71-byte buffer
/// in the exact layout the real ST3215 reports.
pub fn pack_state_bytes(
    motor_id: u8,
    spec: &MotorModelSpec,
    instance: &MotorInstance,
    state: &MotorSemanticState,
) -> Bytes {
    let mut buf = BytesMut::with_capacity(TOTAL_BYTES);

    // ── EEPROM segment (40 bytes, starts at absolute offset 0x00) ──
    let mut eeprom = DEFAULT_EEPROM;
    // 0x00-0x01 ModelNumber (u16 LE)
    eeprom[0x00] = (spec.model_number & 0xFF) as u8;
    eeprom[0x01] = ((spec.model_number >> 8) & 0xFF) as u8;
    // 0x02 FirmwareVersion
    eeprom[0x02] = spec.firmware_version;
    // 0x05 ID
    eeprom[0x05] = motor_id;
    // 0x06 BaudRate
    eeprom[0x06] = spec.baud_rate_code;
    // 0x09-0x0A MinAngleLimit (u16 LE)
    eeprom[0x09] = (instance.min_angle_steps & 0xFF) as u8;
    eeprom[0x0A] = ((instance.min_angle_steps >> 8) & 0xFF) as u8;
    // 0x0B-0x0C MaxAngleLimit (u16 LE)
    eeprom[0x0B] = (instance.max_angle_steps & 0xFF) as u8;
    eeprom[0x0C] = ((instance.max_angle_steps >> 8) & 0xFF) as u8;
    buf.put_slice(&eeprom);

    // ── RAM segment (31 bytes, starts at absolute offset 0x28) ──
    // RAM-relative offsets below are (absolute - 0x28).
    let mut ram = [0u8; RAM_BYTES];
    // 0x28 TorqueEnable
    ram[0x00] = if state.torque_enabled { 1 } else { 0 };
    // 0x2A-0x2B GoalPosition (u16 LE)
    let goal_pos = rad_to_steps(state.goal_position_rad, instance.offset_steps);
    ram[0x02] = (goal_pos & 0xFF) as u8;
    ram[0x03] = ((goal_pos >> 8) & 0xFF) as u8;
    // 0x30-0x31 TorqueLimit (u16 LE)
    ram[0x08] = (instance.torque_limit & 0xFF) as u8;
    ram[0x09] = ((instance.torque_limit >> 8) & 0xFF) as u8;
    // 0x38-0x39 PresentPosition (u16 LE)
    let present_pos = rad_to_steps(state.position_rad, instance.offset_steps);
    ram[0x10] = (present_pos & 0xFF) as u8;
    ram[0x11] = ((present_pos >> 8) & 0xFF) as u8;
    // 0x3A-0x3B PresentSpeed (sign-magnitude u16 LE)
    let present_speed =
        (state.velocity_rad_s / (2.0 * std::f32::consts::PI / STEPS_PER_REV as f32)) as i16;
    let speed_sm = i16_to_sign_magnitude(present_speed);
    ram[0x12] = (speed_sm & 0xFF) as u8;
    ram[0x13] = ((speed_sm >> 8) & 0xFF) as u8;
    // 0x3E PresentVoltage (tenths of a volt)
    ram[0x16] = (instance.voltage_nominal_v * 10.0) as u8;
    // 0x3F PresentTemperature
    ram[0x17] = state.temperature_c as u8;
    // 0x42 Moving
    ram[0x1A] = if state.moving { 1 } else { 0 };
    buf.put_slice(&ram);

    debug_assert_eq!(buf.len(), TOTAL_BYTES);
    buf.freeze()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::presets::ST3215_STANDARD;
    use crate::unpack::unpack_state_bytes;
    use crate::units::rad_to_steps;

    fn sample_state(rad: f32) -> MotorSemanticState {
        MotorSemanticState {
            position_rad: rad,
            velocity_rad_s: 0.0,
            load_nm: 0.0,
            temperature_c: 25.0,
            torque_enabled: true,
            moving: false,
            goal_position_rad: rad,
            goal_speed_rad_s: 0.0,
        }
    }

    #[test]
    fn test_pack_length_71_bytes() {
        let b = pack_state_bytes(
            1,
            &ST3215_STANDARD,
            &MotorInstance::default_test(),
            &sample_state(0.0),
        );
        assert_eq!(b.len(), TOTAL_BYTES);
    }

    #[test]
    fn test_pack_present_position_at_0x38() {
        let b = pack_state_bytes(
            1,
            &ST3215_STANDARD,
            &MotorInstance::default_test(),
            &sample_state(1.0),
        );
        let raw = u16::from_le_bytes([b[0x38], b[0x39]]);
        let expected = rad_to_steps(1.0, 2048);
        assert_eq!(raw, expected);
    }

    #[test]
    fn test_pack_motor_id() {
        let b = pack_state_bytes(
            7,
            &ST3215_STANDARD,
            &MotorInstance::default_test(),
            &sample_state(0.0),
        );
        assert_eq!(b[0x05], 7);
    }

    #[test]
    fn test_pack_torque_enable_boolean() {
        let mut s = sample_state(0.0);
        s.torque_enabled = true;
        assert_eq!(
            pack_state_bytes(1, &ST3215_STANDARD, &MotorInstance::default_test(), &s)[0x28],
            1
        );
        s.torque_enabled = false;
        assert_eq!(
            pack_state_bytes(1, &ST3215_STANDARD, &MotorInstance::default_test(), &s)[0x28],
            0
        );
    }

    #[test]
    fn test_pack_negative_speed_sign_bit() {
        let mut s = sample_state(0.0);
        s.velocity_rad_s = -1.0;
        let b = pack_state_bytes(1, &ST3215_STANDARD, &MotorInstance::default_test(), &s);
        let raw = u16::from_le_bytes([b[0x3A], b[0x3B]]);
        assert!(
            raw & 0x8000 != 0,
            "negative speed should have sign bit set, got {:04x}",
            raw
        );
    }

    #[test]
    fn test_pack_eeprom_model_number() {
        let b = pack_state_bytes(
            1,
            &ST3215_STANDARD,
            &MotorInstance::default_test(),
            &sample_state(0.0),
        );
        assert_eq!(
            u16::from_le_bytes([b[0x00], b[0x01]]),
            ST3215_STANDARD.model_number
        );
    }

    /// ★ P0: pack → unpack must preserve the semantic state within
    /// tolerance. This is the correctness guarantee that sim snapshots
    /// round-trip cleanly through the raw ST3215 queue to legacy clients.
    #[test]
    fn test_pack_roundtrip_via_unpack() {
        let instance = MotorInstance::default_test();
        for rad in [-1.0, -0.5, 0.0, 0.5, 1.0] {
            let s = sample_state(rad);
            let bytes = pack_state_bytes(1, &ST3215_STANDARD, &instance, &s);
            let decoded = unpack_state_bytes(&bytes, &ST3215_STANDARD, &instance).unwrap();
            assert!(
                (decoded.position_rad - rad).abs() < 0.002,
                "rad={} decoded={}",
                rad,
                decoded.position_rad
            );
            assert_eq!(decoded.torque_enabled, s.torque_enabled);
            assert!(
                (decoded.temperature_c - s.temperature_c).abs() < 1.0,
                "temp lost precision"
            );
        }
    }
}
