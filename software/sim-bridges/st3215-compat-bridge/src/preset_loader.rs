//! YAML preset loader. One preset describes a single legacy ST3215
//! bus's worth of motors — their ids, step ranges, offsets, and
//! torque limits — and the `sim://`-prefixed synthetic bus serial
//! the bridge presents to legacy clients.

use crate::errors::BridgeError;
use serde::Deserialize;
use std::path::Path;

#[derive(Debug, Clone, Deserialize)]
pub struct RobotPreset {
    pub robot_id: String,
    pub legacy_bus_serial: String,
    pub motors: Vec<MotorEntry>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct MotorEntry {
    pub actuator_id: String,
    pub motor_id: u8,
    pub min_angle_steps: u16,
    pub max_angle_steps: u16,
    pub offset_steps: i16,
    pub torque_limit: u16,
    pub voltage_nominal_v: f32,
}

pub fn load_preset(path: &Path) -> Result<RobotPreset, BridgeError> {
    let content = std::fs::read_to_string(path).map_err(|e| {
        BridgeError::PresetLoad(format!("read {}: {}", path.display(), e))
    })?;
    let preset: RobotPreset = serde_yaml::from_str(&content).map_err(|e| {
        BridgeError::PresetLoad(format!("parse {}: {}", path.display(), e))
    })?;
    if !preset.legacy_bus_serial.starts_with("sim://") {
        return Err(BridgeError::PresetLoad(format!(
            "preset '{}' legacy_bus_serial '{}' missing 'sim://' prefix",
            path.display(),
            preset.legacy_bus_serial
        )));
    }
    // Sanity: duplicate motor_id or actuator_id would shadow entries
    // in ActuatorMap and cause silent routing mistakes.
    let mut seen_ids = std::collections::HashSet::new();
    let mut seen_actuators = std::collections::HashSet::new();
    for m in &preset.motors {
        if !seen_ids.insert(m.motor_id) {
            return Err(BridgeError::PresetLoad(format!(
                "preset '{}' duplicate motor_id {}",
                path.display(),
                m.motor_id
            )));
        }
        if !seen_actuators.insert(m.actuator_id.clone()) {
            return Err(BridgeError::PresetLoad(format!(
                "preset '{}' duplicate actuator_id '{}'",
                path.display(),
                m.actuator_id
            )));
        }
    }
    Ok(preset)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    fn write_yaml(contents: &str) -> NamedTempFile {
        let mut f = NamedTempFile::new().unwrap();
        f.write_all(contents.as_bytes()).unwrap();
        f
    }

    #[test]
    fn test_load_preset_happy() {
        let f = write_yaml(
            "robot_id: test\n\
             legacy_bus_serial: \"sim://test\"\n\
             motors:\n\
             - {actuator_id: m1, motor_id: 1, min_angle_steps: 0, max_angle_steps: 4095, offset_steps: 2048, torque_limit: 500, voltage_nominal_v: 12.0}\n",
        );
        let p = load_preset(f.path()).unwrap();
        assert_eq!(p.robot_id, "test");
        assert_eq!(p.motors.len(), 1);
        assert_eq!(p.motors[0].motor_id, 1);
        assert_eq!(p.motors[0].offset_steps, 2048);
    }

    #[test]
    fn test_load_preset_missing_sim_prefix() {
        let f = write_yaml(
            "robot_id: test\nlegacy_bus_serial: \"bus0\"\nmotors: []\n",
        );
        let err = load_preset(f.path()).unwrap_err();
        match err {
            BridgeError::PresetLoad(msg) => assert!(msg.contains("sim://")),
            _ => panic!("wrong error variant"),
        }
    }

    #[test]
    fn test_load_preset_duplicate_motor_id_rejected() {
        let f = write_yaml(
            "robot_id: test\n\
             legacy_bus_serial: \"sim://test\"\n\
             motors:\n\
             - {actuator_id: a, motor_id: 1, min_angle_steps: 0, max_angle_steps: 4095, offset_steps: 0, torque_limit: 500, voltage_nominal_v: 12.0}\n\
             - {actuator_id: b, motor_id: 1, min_angle_steps: 0, max_angle_steps: 4095, offset_steps: 0, torque_limit: 500, voltage_nominal_v: 12.0}\n",
        );
        let err = load_preset(f.path()).unwrap_err();
        match err {
            BridgeError::PresetLoad(msg) => assert!(msg.contains("duplicate motor_id")),
            _ => panic!("wrong error variant"),
        }
    }

    #[test]
    fn test_load_elrobot_preset_file() {
        let p = load_preset(
            Path::new("presets/elrobot-follower.yaml"),
        )
        .expect("load elrobot preset");
        assert_eq!(p.robot_id, "elrobot_follower");
        assert_eq!(p.legacy_bus_serial, "sim://bus0");
        assert_eq!(p.motors.len(), 8);
        // Motor 8 is the gripper with offset_steps=0.
        let m8 = p.motors.iter().find(|m| m.motor_id == 8).unwrap();
        assert_eq!(m8.offset_steps, 0);
    }
}
