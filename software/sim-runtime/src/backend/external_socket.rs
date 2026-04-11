//! `ExternalSocketBackend` — connects to an already-running sim process
//! via an existing Unix-domain socket. Used when the developer manages
//! the sim server's lifecycle out-of-band (e.g. `python -m norma_sim
//! --socket /tmp/norma-sim-dev.sock` in one terminal, `station -c
//! station-sim-external.yaml` in another).
//!
//! Compared with `ChildProcessBackend` this variant:
//!   - does not spawn or own a subprocess
//!   - does not create a TempRuntimeDir (the path comes from config)
//!   - `wait_terminated` returns `Clean` once the IPC channel closes,
//!     since we have no child to observe

use super::transport::spawn_transport;
use super::{BackendStarted, BackendTermination, WorldBackend};
use crate::clock::new_session_id;
use crate::errors::SimRuntimeError;
use crate::ipc::handshake::perform_client_handshake;
use async_trait::async_trait;
use std::path::PathBuf;
use std::time::Duration;
use tokio::net::UnixStream;
use tokio::sync::Notify;
use std::sync::Arc;

pub(crate) struct ExternalSocketBackend {
    socket_path: PathBuf,
    session_id: String,
    terminated: Arc<Notify>,
}

impl ExternalSocketBackend {
    #[allow(dead_code)] // constructed in runtime.rs (Task 4.8)
    pub fn new(socket_path: PathBuf, session_id: String) -> Self {
        Self {
            socket_path,
            session_id,
            terminated: Arc::new(Notify::new()),
        }
    }
}

#[async_trait]
impl WorldBackend for ExternalSocketBackend {
    async fn start(
        &mut self,
        startup_timeout: Duration,
    ) -> Result<BackendStarted, SimRuntimeError> {
        let stream = UnixStream::connect(&self.socket_path).await.map_err(|e| {
            SimRuntimeError::BackendSpawn(format!(
                "connect {:?}: {}",
                self.socket_path, e
            ))
        })?;
        let (outbound_tx, mut inbound_rx) = spawn_transport(stream);
        let descriptor = perform_client_handshake(
            &outbound_tx,
            &mut inbound_rx,
            new_session_id(),
            startup_timeout,
        )
        .await?;
        // Sanity: reference session_id so the compiler doesn't warn if
        // downstream usage changes. Session id is currently informational.
        let _ = &self.session_id;
        Ok(BackendStarted {
            descriptor,
            outbound_tx,
            inbound_rx,
        })
    }

    async fn wait_terminated(self: Box<Self>) -> BackendTermination {
        // No child to observe — block until someone calls shutdown().
        self.terminated.notified().await;
        BackendTermination::Clean
    }

    async fn shutdown(&mut self, _grace: Duration) -> Result<(), SimRuntimeError> {
        self.terminated.notify_waiters();
        Ok(())
    }
}
