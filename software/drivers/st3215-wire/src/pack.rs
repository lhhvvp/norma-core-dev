// Task 2.1 stub — replaced in Task 2.5 with real pack_state_bytes + tests.

#![allow(dead_code, unused_variables)]

use crate::presets::MotorModelSpec;

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

pub fn pack_state_bytes(
    _motor_id: u8,
    _spec: &MotorModelSpec,
    _instance: &MotorInstance,
    _state: &MotorSemanticState,
) -> bytes::Bytes {
    bytes::Bytes::new()
}
