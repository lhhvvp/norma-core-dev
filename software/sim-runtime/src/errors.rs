//! `SimRuntimeError` — every operational failure the subsystem can surface.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum SimRuntimeError {
    #[error("runtime dir create failed: {0}")]
    RuntimeDirCreate(String),

    #[error("backend spawn failed: {0}")]
    BackendSpawn(String),

    #[error("handshake timeout")]
    HandshakeTimeout,

    #[error("protocol version mismatch: ours={ours}, theirs={theirs}")]
    ProtocolMismatch { ours: u32, theirs: u32 },

    #[error("backend crashed")]
    BackendCrashed,

    #[error("ipc channel closed")]
    IpcClosed,

    #[error("config validation: {0}")]
    ConfigValidation(String),

    #[error("normfs error: {0}")]
    NormfsError(String),

    #[error("backpressure: reliable lane full")]
    Backpressure,
}
