//! Bridge error types.
//!
//! Partial scaffold in Task 7.3 (PresetLoad variant needed by the
//! preset_loader tests); Task 7.8 fills in the remaining variants.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum BridgeError {
    #[error("invalid config: {0}")]
    InvalidConfig(String),

    #[error("preset load: {0}")]
    PresetLoad(String),

    #[error("robot '{0}' not in world")]
    RobotNotInWorld(String),

    #[error("actuator '{0}' not in world")]
    ActuatorNotInWorld(String),

    #[error("unknown motor_id {0}")]
    UnknownMotorId(u8),

    #[error("normfs subscribe: {0}")]
    NormfsSubscribe(String),

    #[error("sim runtime error: {0}")]
    SimRuntime(#[from] sim_runtime::SimRuntimeError),
}
