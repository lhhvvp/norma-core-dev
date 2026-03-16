/// ST3215 motor settings presets

// ============================================================================
// Default Motor Settings
// ============================================================================

/// Default max torque
pub const DEFAULT_MAX_TORQUE: u16 = 500;

/// Default protection current
pub const DEFAULT_PROTECTION_CURRENT: u16 = 260;

/// Default overload torque
pub const DEFAULT_OVERLOAD_TORQUE: u8 = 25;

/// Default acceleration
pub const DEFAULT_ACCEL: u8 = 254;

/// Default torque limit
pub const DEFAULT_TORQUE_LIMIT: u16 = 500;

/// Default PID P gain
pub const DEFAULT_PID_P: u8 = 16;

/// Default PID I gain
pub const DEFAULT_PID_I: u8 = 8;

/// Default PID D gain
pub const DEFAULT_PID_D: u8 = 32;

// ============================================================================
// Calibration Motor Settings
// ============================================================================

/// Sweep speed during calibration
pub const CALIBRATION_SPEED: u16 = 365;

/// Sweep acceleration during calibration
pub const CALIBRATION_ACCEL: u8 = 50;

/// Torque limit during calibration
pub const CALIBRATION_TORQUE_LIMIT: u16 = 300;
