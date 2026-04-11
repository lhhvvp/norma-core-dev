// Task 2.1 stub — replaced in Task 2.4 with real unpack_state_bytes + tests.

#![allow(dead_code, unused_variables)]

use crate::pack::{MotorInstance, MotorSemanticState};
use crate::presets::MotorModelSpec;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum UnpackError {
    #[error("stub")]
    Stub,
}

pub fn unpack_state_bytes(
    _bytes: &[u8],
    _spec: &MotorModelSpec,
    _instance: &MotorInstance,
) -> Result<MotorSemanticState, UnpackError> {
    Ok(MotorSemanticState::default())
}
