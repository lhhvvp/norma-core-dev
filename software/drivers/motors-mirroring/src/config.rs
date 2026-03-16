use std::{collections::HashMap, time::Duration};

pub const MIRRORING_REFRESH_INTERVAL: Duration = Duration::from_millis(20);

#[derive(Clone)]
pub struct MotorConfig {
    pub safety_margin: u16,
    pub deadband: u16,
    pub max_speed: u16,
    pub min_speed: u16,
    pub max_accel: u16,
    pub min_accel: u16,
    pub max_steps: u16,
    /// Default current threshold (in raw units). When target motor's current exceeds this,
    /// set goal to current position to prevent overload.
    /// 0 means disabled.
    pub current_threshold: u16,
    /// Per-motor current threshold overrides. Key is motor_id (0-255).
    /// If a motor_id is in this map, its value overrides the default current_threshold.
    pub per_motor_current_threshold: HashMap<u8, u16>,
}

impl MotorConfig {
    /// Get the effective current threshold for a specific motor.
    /// Returns the per-motor override if set, otherwise the default threshold.
    pub fn get_current_threshold(&self, motor_id: u8) -> u16 {
        self.per_motor_current_threshold
            .get(&motor_id)
            .copied()
            .unwrap_or(self.current_threshold)
    }

    /// Set a per-motor current threshold override.
    pub fn set_motor_current_threshold(&mut self, motor_id: u8, threshold: u16) {
        self.per_motor_current_threshold.insert(motor_id, threshold);
    }

    /// Clear a per-motor current threshold override, reverting to default.
    pub fn clear_motor_current_threshold(&mut self, motor_id: u8) {
        self.per_motor_current_threshold.remove(&motor_id);
    }
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            safety_margin: 20,
            deadband: 20,
            max_speed: 3300,
            min_speed: 300,
            max_accel: 100,
            min_accel: 5,
            max_steps: 4096,
            current_threshold: 100, // enabled by default with threshold of 100
            per_motor_current_threshold: HashMap::new(),
        }
    }
}

impl From<&station_iface::config::St3215Config> for MotorConfig {
    fn from(config: &station_iface::config::St3215Config) -> Self {
        Self {
            current_threshold: config.current_threshold,
            deadband: config.deadband,
            per_motor_current_threshold: config.motor_current_thresholds.clone().unwrap_or_default(),
            ..Default::default()
        }
    }
}