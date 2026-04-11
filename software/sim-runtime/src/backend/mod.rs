//! Backend abstraction: the strategy the runtime uses to obtain a
//! capability-keyed world simulation.
//!
//! IMPORTANT ARCHITECTURE INVARIANT (CI-enforced): the `WorldBackend`
//! trait is `pub(crate)`, NOT `pub`. Outside crates interact with the
//! simulation runtime exclusively through the public `SimulationRuntime`
//! façade (see `runtime.rs`). The CI check
//!     grep -q "^pub trait WorldBackend" software/sim-runtime/src/backend/mod.rs
//! MUST return exit code 1 (no match) — otherwise the subsystem boundary
//! has leaked.

pub(crate) mod mock;
pub(crate) mod runtime_dir;
pub(crate) mod transport;

use crate::errors::SimRuntimeError;
use crate::proto::{Envelope, WorldDescriptor};
use async_trait::async_trait;
use std::time::Duration;
use tokio::sync::mpsc;

#[async_trait]
pub(crate) trait WorldBackend: Send + 'static {
    /// Start the backend and complete the `Hello`/`Welcome` handshake.
    /// Returns the descriptor and the two-ended IPC channel.
    async fn start(
        &mut self,
        startup_timeout: Duration,
    ) -> Result<BackendStarted, SimRuntimeError>;

    /// Block until the backend terminates. Consumed by the supervisor.
    async fn wait_terminated(self: Box<Self>) -> BackendTermination;

    /// Request graceful shutdown. Must be idempotent.
    async fn shutdown(&mut self, grace: Duration) -> Result<(), SimRuntimeError>;
}

/// Payload returned by a successful `WorldBackend::start` — the runtime
/// plugs the channels into its snapshot broker and actuation sender.
pub(crate) struct BackendStarted {
    pub descriptor: WorldDescriptor,
    pub outbound_tx: mpsc::Sender<Envelope>,
    pub inbound_rx: mpsc::Receiver<Envelope>,
}

#[derive(Debug, Clone)]
pub(crate) enum BackendTermination {
    Clean,
    Crashed {
        exit_code: Option<i32>,
        stderr_tail: Vec<u8>,
    },
    KilledBySupervisor,
    SignaledByOs {
        signal: i32,
    },
}
