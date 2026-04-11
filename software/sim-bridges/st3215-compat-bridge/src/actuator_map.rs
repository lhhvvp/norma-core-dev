//! Bidirectional lookup: `motor_id` ↔ `actuator_id` for a single
//! ElRobot-style preset. Built once at bridge startup from the
//! loaded `RobotPreset`; queried on every inbound ST3215 command
//! (command_task) and every outbound WorldSnapshot entry
//! (state_task), so the O(1) HashMap lookups matter.

use crate::preset_loader::{MotorEntry, RobotPreset};
use std::collections::HashMap;

pub struct ActuatorMap {
    pub by_motor_id: HashMap<u8, MotorEntry>,
    pub by_actuator_id: HashMap<String, u8>,
}

impl ActuatorMap {
    pub fn from_preset(preset: &RobotPreset) -> Self {
        let mut by_motor_id = HashMap::with_capacity(preset.motors.len());
        let mut by_actuator_id = HashMap::with_capacity(preset.motors.len());
        for m in &preset.motors {
            by_motor_id.insert(m.motor_id, m.clone());
            by_actuator_id.insert(m.actuator_id.clone(), m.motor_id);
        }
        Self {
            by_motor_id,
            by_actuator_id,
        }
    }

    pub fn get_by_motor_id(&self, motor_id: u8) -> Option<&MotorEntry> {
        self.by_motor_id.get(&motor_id)
    }

    pub fn get_motor_id_by_actuator(&self, actuator_id: &str) -> Option<u8> {
        self.by_actuator_id.get(actuator_id).copied()
    }

    pub fn len(&self) -> usize {
        self.by_motor_id.len()
    }

    /// Companion to `len()` — clippy's `len_without_is_empty` lint
    /// requires both; call sites only use `len()`.
    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.by_motor_id.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::preset_loader::MotorEntry;

    fn motor(id: u8, act: &str) -> MotorEntry {
        MotorEntry {
            actuator_id: act.into(),
            motor_id: id,
            min_angle_steps: 0,
            max_angle_steps: 4095,
            offset_steps: 2048,
            torque_limit: 500,
            voltage_nominal_v: 12.0,
        }
    }

    #[test]
    fn test_bidirectional_lookup() {
        let preset = RobotPreset {
            robot_id: "test".into(),
            legacy_bus_serial: "sim://test".into(),
            motors: vec![motor(1, "rev_motor_01"), motor(8, "rev_motor_08")],
        };
        let map = ActuatorMap::from_preset(&preset);
        assert_eq!(map.len(), 2);
        assert_eq!(map.get_by_motor_id(1).unwrap().actuator_id, "rev_motor_01");
        assert_eq!(map.get_by_motor_id(8).unwrap().actuator_id, "rev_motor_08");
        assert_eq!(map.get_motor_id_by_actuator("rev_motor_01"), Some(1));
        assert_eq!(map.get_motor_id_by_actuator("rev_motor_08"), Some(8));
        // Nonexistent
        assert!(map.get_by_motor_id(99).is_none());
        assert_eq!(map.get_motor_id_by_actuator("nope"), None);
    }

    #[test]
    fn test_empty_preset() {
        let preset = RobotPreset {
            robot_id: "empty".into(),
            legacy_bus_serial: "sim://empty".into(),
            motors: vec![],
        };
        let map = ActuatorMap::from_preset(&preset);
        assert!(map.is_empty());
        assert_eq!(map.len(), 0);
    }
}
