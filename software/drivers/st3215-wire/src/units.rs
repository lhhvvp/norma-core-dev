//! ST3215 unit conversions and byte-level state extractors.
//!
//! Two kinds of helpers live here:
//!
//! 1. **Byte-slice extractors** (`get_motor_position`, `is_torque_enabled`,
//!    etc.) — migrated from `software/drivers/st3215/src/protocol/units.rs`.
//!    These are used by the real ST3215 driver on data read from hardware.
//!
//! 2. **Semantic conversions** (`steps_to_rad`, `rad_to_steps`,
//!    `sign_magnitude_to_i16`, `i16_to_sign_magnitude`) — new helpers
//!    required by `pack.rs` / `unpack.rs` for the sim-side wire format
//!    round-trip. The real driver does not use these directly.

use crate::register::RamRegister;

// ---------------------------------------------------------------------------
// Migrated from st3215/src/protocol/units.rs (verbatim)
// ---------------------------------------------------------------------------

pub const MAX_ANGLE_STEP: u16 = 4095;
const SIGN_BIT_MASK: u16 = 0x8000;

pub fn normal_position(position: u16) -> u16 {
    if position & SIGN_BIT_MASK != 0 {
        let magnitude = position & MAX_ANGLE_STEP;
        (MAX_ANGLE_STEP + 1 - magnitude) & MAX_ANGLE_STEP
    } else {
        position & MAX_ANGLE_STEP
    }
}

/// Extract motor position from state bytes
pub fn get_motor_position(state: &[u8]) -> u16 {
    let addr = RamRegister::PresentPosition.address() as usize;
    if state.len() < addr + 2 {
        return 0;
    }
    normal_position(u16::from_le_bytes([state[addr], state[addr + 1]]))
}

/// Extract motor goal position from state bytes
pub fn get_motor_goal_position(state: &[u8]) -> u16 {
    let addr = RamRegister::GoalPosition.address() as usize;
    if state.len() < addr + 2 {
        return 0;
    }
    normal_position(u16::from_le_bytes([state[addr], state[addr + 1]]))
}

/// Extract motor current from state bytes (in milliamps)
pub fn get_motor_current(state: &[u8]) -> u16 {
    let addr = RamRegister::PresentCurrent.address() as usize;
    if state.len() < addr + 2 {
        return 0;
    }
    u16::from_le_bytes([state[addr], state[addr + 1]])
}

/// Extract motor velocity from state bytes
pub fn get_motor_velocity(state: &[u8]) -> u16 {
    let addr = RamRegister::PresentSpeed.address() as usize;
    if state.len() < addr + 2 {
        return 0;
    }
    u16::from_le_bytes([state[addr], state[addr + 1]])
}

/// Normalize motor position to [0.0, 1.0] range
pub fn normalize_motor_position(value: u16, min: u16, max: u16) -> f32 {
    if min == max {
        return 0.0;
    }

    let motor_range = if max > min {
        (max - min) as f32
    } else {
        (4096 + max - min) as f32
    };

    let threshold = motor_range * 0.04;

    // Check if out of bounds
    if max > min {
        if value < min {
            let delta = (min - value) as f32;
            if delta < threshold {
                return 0.0;
            }
        } else if value > max {
            let delta = (value - max) as f32;
            if delta < threshold {
                return 1.0;
            }
        }
    } else if value > max && value < min {
        let delta_to_max = (value - max) as f32;
        let delta_to_min = (min - value) as f32;
        if delta_to_min <= delta_to_max && delta_to_min < threshold {
            return 0.0;
        }
        if delta_to_max < delta_to_min && delta_to_max < threshold {
            return 1.0;
        }
    }

    // Normal position calculation
    let pos = if max > min || value >= min {
        (value - min) as f32
    } else {
        (4096 + value - min) as f32
    };

    pos / motor_range
}

/// Check if motor is in error state
pub fn is_motor_error(state: &[u8]) -> bool {
    let addr = RamRegister::Status.address() as usize;
    if state.len() < addr + 1 {
        return true; // Insufficient data is also an error
    }
    state[addr] != 0
}

/// Check if motor has torque enabled
pub fn is_torque_enabled(state: &[u8]) -> bool {
    let addr = RamRegister::TorqueEnable.address() as usize;
    if state.len() < addr + 1 {
        return false;
    }
    state[addr] != 0
}

// ---------------------------------------------------------------------------
// New helpers (for sim-side pack/unpack). Not used by the real driver today.
// ---------------------------------------------------------------------------

pub const STEPS_PER_REV: u32 = 4096;

/// Convert raw step count to radians relative to the motor's zero offset.
///
/// `offset_steps` is the raw step value that corresponds to 0 rad (typically
/// 2048 for a bidirectional joint centered in its step range).
pub fn steps_to_rad(steps: u16, offset_steps: i16) -> f32 {
    let centered = steps as i32 - offset_steps as i32;
    (centered as f32) * (2.0 * std::f32::consts::PI / STEPS_PER_REV as f32)
}

/// Inverse of `steps_to_rad`. Clamps to the raw step range `[0, STEPS_PER_REV-1]`.
pub fn rad_to_steps(rad: f32, offset_steps: i16) -> u16 {
    let centered = rad / (2.0 * std::f32::consts::PI / STEPS_PER_REV as f32);
    let raw = centered.round() as i32 + offset_steps as i32;
    raw.clamp(0, (STEPS_PER_REV - 1) as i32) as u16
}

/// Decode the ST3215's sign-magnitude representation of a signed 16-bit value.
/// The top bit is the sign, the lower 15 bits are the magnitude.
pub fn sign_magnitude_to_i16(raw: u16) -> i16 {
    let magnitude = (raw & 0x7FFF) as i16;
    if raw & 0x8000 != 0 {
        -magnitude
    } else {
        magnitude
    }
}

/// Encode an `i16` in the ST3215's sign-magnitude wire format.
pub fn i16_to_sign_magnitude(value: i16) -> u16 {
    if value >= 0 {
        value as u16
    } else {
        ((-value) as u16) | 0x8000
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Migrated from st3215/src/protocol/units_test.rs

    #[test]
    fn test_normal_position_positive_values() {
        assert_eq!(normal_position(0), 0);
        assert_eq!(normal_position(1), 1);
        assert_eq!(normal_position(100), 100);
        assert_eq!(normal_position(4095), 4095);
        assert_eq!(normal_position(4096), 0);
        assert_eq!(normal_position(4097), 1);
        assert_eq!(normal_position(8191), 4095);
    }

    #[test]
    fn test_normal_position_negative_values() {
        assert_eq!(normal_position(0x8001), 4095);
        assert_eq!(normal_position(0x8002), 4094);
        assert_eq!(normal_position(0x800A), 4086);
        assert_eq!(normal_position(0x8064), 3996);
        assert_eq!(normal_position(0x8FFF), 1);
        assert_eq!(normal_position(0x9000), 0);
    }

    #[test]
    fn test_normal_position_edge_cases() {
        assert_eq!(normal_position(0x7FFF), 4095);
        assert_eq!(normal_position(0x8000), 0);
        assert_eq!(normal_position(0xFFFF), 1);
    }

    #[test]
    fn test_normal_position_wraparound() {
        for i in 0..4096u16 {
            let result = normal_position(i);
            assert!(result <= 4095);
            assert_eq!(result, i & 4095);
        }
        for i in 1..4096u16 {
            let negative_value = 0x8000 | i;
            let result = normal_position(negative_value);
            assert!(result <= 4095);
            assert_eq!(result, (4096 - i) & 4095);
        }
    }

    // New helpers

    #[test]
    fn test_steps_rad_roundtrip() {
        let offset = 2048_i16;
        for rad in [-1.5, -0.5, 0.0, 0.5, 1.5] {
            let steps = rad_to_steps(rad, offset);
            let back = steps_to_rad(steps, offset);
            assert!(
                (back - rad).abs() < 0.002,
                "rad={} round-trip to {} via steps={}",
                rad,
                back,
                steps
            );
        }
    }

    #[test]
    fn test_sign_magnitude_symmetric() {
        for &v in &[-1000i16, -1, 0, 1, 1000] {
            assert_eq!(sign_magnitude_to_i16(i16_to_sign_magnitude(v)), v);
        }
    }
}
