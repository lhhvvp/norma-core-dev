//! `ChildProcessBackend` — launches a sim backend subprocess (typically
//! `python3 -m norma_sim ...`) as a child of Station, waits for the
//! socket to appear inside the `TempRuntimeDir`, connects, and performs
//! the client-side handshake.
//!
//! Lifetime:
//!   - `new(config, session_id)` stores the launcher argv / env plan
//!     without running anything yet.
//!   - `start(timeout)` spawns the subprocess, polls for the socket,
//!     connects and handshakes. On any failure the child is killed.
//!   - `wait_terminated(self)` (consumed) awaits the child's exit and
//!     returns the appropriate `BackendTermination` variant.
//!   - `shutdown(grace)` is called by the runtime for graceful
//!     teardown; currently just drops the channels so the dispatch
//!     loop exits and Drop (via `kill_on_drop`) reaps the child.

use super::runtime_dir::TempRuntimeDir;
use super::transport::spawn_transport;
use super::{BackendStarted, BackendTermination, WorldBackend};
use crate::clock::new_session_id;
use crate::errors::SimRuntimeError;
use crate::ipc::handshake::perform_client_handshake;
use async_trait::async_trait;
use station_iface::config::{LogCapture, SimRuntimeConfig};
use std::process::Stdio;
use std::sync::Arc;
use std::time::Duration;
use tokio::net::UnixStream;
use tokio::process::{Child, Command};
use tokio::sync::Mutex;

pub(crate) struct ChildProcessBackend {
    launcher: Vec<String>,
    session_id: String,
    runtime_dir: Option<Arc<TempRuntimeDir>>,
    child: Arc<Mutex<Option<Child>>>,
    log_capture: LogCapture,
    log_file: Option<std::path::PathBuf>,
    handshake_timeout_override: Option<Duration>,
}

impl ChildProcessBackend {
    #[allow(dead_code)] // constructed in runtime.rs (Task 4.8)
    pub fn new(config: &SimRuntimeConfig, session_id: String) -> Self {
        Self {
            launcher: config.launcher.clone().unwrap_or_default(),
            session_id,
            runtime_dir: None,
            child: Arc::new(Mutex::new(None)),
            log_capture: config.log_capture,
            log_file: config.log_file.clone(),
            handshake_timeout_override: None,
        }
    }
}

#[async_trait]
impl WorldBackend for ChildProcessBackend {
    async fn start(
        &mut self,
        startup_timeout: Duration,
    ) -> Result<BackendStarted, SimRuntimeError> {
        // 1. Create a runtime dir owned by this backend instance.
        let station_pid = std::process::id();
        let dir = TempRuntimeDir::create(None, station_pid)
            .map_err(|e| SimRuntimeError::RuntimeDirCreate(e.to_string()))?;
        let socket_path = dir.socket_path();
        self.runtime_dir = Some(Arc::new(dir));

        // 2. Resolve log capture
        let (stdout_cfg, stderr_cfg) = match self.log_capture {
            LogCapture::Inherit => (Stdio::inherit(), Stdio::inherit()),
            LogCapture::Null => (Stdio::null(), Stdio::null()),
            LogCapture::File => {
                if let Some(path) = &self.log_file {
                    match std::fs::File::create(path) {
                        Ok(f) => {
                            // Clone the file handle for stdout / stderr.
                            let f2 = match f.try_clone() {
                                Ok(c) => c,
                                Err(e) => {
                                    return Err(SimRuntimeError::BackendSpawn(format!(
                                        "log file clone: {}",
                                        e
                                    )))
                                }
                            };
                            (Stdio::from(f), Stdio::from(f2))
                        }
                        Err(e) => {
                            return Err(SimRuntimeError::BackendSpawn(format!(
                                "log file create {:?}: {}",
                                path, e
                            )))
                        }
                    }
                } else {
                    (Stdio::null(), Stdio::null())
                }
            }
        };

        // 3. Build the Command
        if self.launcher.is_empty() {
            return Err(SimRuntimeError::ConfigValidation(
                "launcher cannot be empty".into(),
            ));
        }
        let mut cmd = Command::new(&self.launcher[0]);
        cmd.args(&self.launcher[1..])
            .env("NORMA_SIM_SOCKET_PATH", &socket_path)
            .env("NORMA_SIM_SESSION_ID", &self.session_id)
            .stdout(stdout_cfg)
            .stderr(stderr_cfg)
            .kill_on_drop(true);

        let child = cmd
            .spawn()
            .map_err(|e| SimRuntimeError::BackendSpawn(format!("spawn: {}", e)))?;
        *self.child.lock().await = Some(child);

        // 4. Poll for the socket up to startup_timeout.
        let poll_start = tokio::time::Instant::now();
        let poll_interval = Duration::from_millis(50);
        loop {
            if socket_path.exists() {
                break;
            }
            if poll_start.elapsed() >= startup_timeout {
                // Kill the child to avoid a zombie.
                if let Some(ch) = self.child.lock().await.as_mut() {
                    let _ = ch.kill().await;
                }
                return Err(SimRuntimeError::BackendSpawn(format!(
                    "socket {:?} did not appear within {:?}",
                    socket_path, startup_timeout
                )));
            }
            tokio::time::sleep(poll_interval).await;
        }

        // 5. Connect + handshake
        let stream = UnixStream::connect(&socket_path).await.map_err(|e| {
            SimRuntimeError::BackendSpawn(format!("connect {:?}: {}", socket_path, e))
        })?;
        let (outbound_tx, mut inbound_rx) = spawn_transport(stream);
        let handshake_timeout = self
            .handshake_timeout_override
            .unwrap_or_else(|| startup_timeout.saturating_sub(poll_start.elapsed()));
        let descriptor = perform_client_handshake(
            &outbound_tx,
            &mut inbound_rx,
            new_session_id(),
            handshake_timeout,
        )
        .await?;

        Ok(BackendStarted {
            descriptor,
            outbound_tx,
            inbound_rx,
        })
    }

    async fn wait_terminated(self: Box<Self>) -> BackendTermination {
        // Take the child out of the mutex and await it.
        let mut guard = self.child.lock().await;
        if let Some(mut child) = guard.take() {
            match child.wait().await {
                Ok(status) => {
                    if status.success() {
                        BackendTermination::Clean
                    } else {
                        BackendTermination::Crashed {
                            exit_code: status.code(),
                            stderr_tail: vec![],
                        }
                    }
                }
                Err(_) => BackendTermination::Crashed {
                    exit_code: None,
                    stderr_tail: vec![],
                },
            }
        } else {
            BackendTermination::Clean
        }
    }

    async fn shutdown(&mut self, _grace: Duration) -> Result<(), SimRuntimeError> {
        // Drop the channels (implicit — caller owns them) and kill the
        // child if still alive. MVP-1 uses `kill_on_drop(true)` as the
        // fallback so we don't need explicit SIGTERM-then-SIGKILL logic.
        if let Some(ch) = self.child.lock().await.as_mut() {
            let _ = ch.kill().await;
        }
        // The runtime_dir's Drop impl removes /tmp/norma-sim-<pid>.
        self.runtime_dir = None;
        Ok(())
    }
}
