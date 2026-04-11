//! `SimulationRuntime` — the public façade of the sim subsystem.
//!
//! Stations construct one `SimulationRuntime` per `SimRuntimeConfig`
//! (Scenario A internal, B external, or C shadow). Everything else in
//! the crate — backends, IPC, broker, sender, health, supervisor — is
//! orchestrated through this single entry point.

use crate::actuation_sender::ActuationSender;
use crate::backend::child_process::ChildProcessBackend;
use crate::backend::external_socket::ExternalSocketBackend;
use crate::backend::WorldBackend;
use crate::clock::new_session_id;
use crate::errors::SimRuntimeError;
use crate::health::HealthPublisher;
use crate::proto::{
    envelope::Payload, ActuationBatch, SimHealth, WorldDescriptor, WorldSnapshot,
};
use crate::registry::WorldRegistry;
use crate::snapshot_broker::SnapshotBroker;
use crate::SimRuntimeConfig;
use normfs::NormFS;
use station_iface::config::SimMode;
use station_iface::StationEngine;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::broadcast;

pub struct SimulationRuntime {
    #[allow(dead_code)] // used by future registry() getter + compat bridge
    registry: WorldRegistry,
    descriptor: WorldDescriptor,
    broker: Arc<SnapshotBroker>,
    actuation: Arc<ActuationSender>,
    health: Arc<HealthPublisher>,
    session_id: String,
    _dispatch_task: tokio::task::JoinHandle<()>,
}

impl SimulationRuntime {
    /// Start a sim subsystem from user config. Dispatches on
    /// `SimMode` to the appropriate `WorldBackend`, performs the
    /// handshake, publishes the descriptor to `/sim/descriptor`, and
    /// spawns the dispatch loop.
    pub async fn start(
        normfs: Arc<NormFS>,
        _engine: Arc<dyn StationEngine>,
        config: SimRuntimeConfig,
    ) -> Result<Arc<Self>, SimRuntimeError> {
        config
            .validate()
            .map_err(|e| SimRuntimeError::ConfigValidation(e.into()))?;
        let session_id = new_session_id();

        let backend: Box<dyn WorldBackend> = match config.mode {
            SimMode::Internal => {
                Box::new(ChildProcessBackend::new(&config, session_id.clone()))
            }
            SimMode::External => Box::new(ExternalSocketBackend::new(
                config
                    .socket_path
                    .clone()
                    .expect("validate() ensures socket_path is set in External mode"),
                session_id.clone(),
            )),
        };

        Self::bootstrap(normfs, backend, session_id, config.startup_timeout_ms).await
    }

    /// Test-only constructor that accepts any `WorldBackend`
    /// (typically `MockBackend`). Skips the `SimRuntimeConfig` dispatch
    /// so integration tests don't need to assemble a full config.
    #[cfg(test)]
    pub(crate) async fn start_with_backend(
        normfs: Arc<NormFS>,
        backend: Box<dyn WorldBackend>,
        session_id: String,
    ) -> Result<Arc<Self>, SimRuntimeError> {
        Self::bootstrap(normfs, backend, session_id, 1_000).await
    }

    async fn bootstrap(
        normfs: Arc<NormFS>,
        mut backend: Box<dyn WorldBackend>,
        session_id: String,
        startup_timeout_ms: u64,
    ) -> Result<Arc<Self>, SimRuntimeError> {
        // Handshake
        let started = backend
            .start(Duration::from_millis(startup_timeout_ms))
            .await?;
        let descriptor = started.descriptor;
        let outbound_tx = started.outbound_tx;
        let mut inbound_rx = started.inbound_rx;

        // Build subsystem components
        let registry = WorldRegistry::from_descriptor(&descriptor);
        let broker = Arc::new(SnapshotBroker::new(256));
        let actuation = Arc::new(ActuationSender::new(outbound_tx));
        let health = Arc::new(HealthPublisher::new(normfs.clone(), session_id.clone()).await?);

        // Publish descriptor to the NormFS /sim/descriptor queue
        // exactly once, before returning to the caller. This gives
        // downstream replay tooling a single authoritative snapshot.
        health.publish_descriptor(&descriptor)?;

        // Supervise backend termination: the boxed backend is consumed
        // by the supervisor task, so the runtime keeps its own clone-
        // free references to the other components only.
        crate::supervisor::spawn_supervisor(backend, health.clone());

        // Dispatch loop: route inbound Envelopes.
        //   - Snapshot  → broker.publish(snap)
        //   - Error     → log at warn
        //   - other     → trace
        let broker_for_dispatch = broker.clone();
        let dispatch = tokio::spawn(async move {
            while let Some(env) = inbound_rx.recv().await {
                match env.payload {
                    Some(Payload::Snapshot(snap)) => {
                        broker_for_dispatch.publish(snap);
                    }
                    Some(Payload::Error(e)) => {
                        log::warn!(
                            target: "sim_runtime::dispatch",
                            "backend error: code={} msg={}",
                            e.code,
                            e.message
                        );
                    }
                    _ => {
                        log::trace!(
                            target: "sim_runtime::dispatch",
                            "unhandled envelope"
                        );
                    }
                }
            }
            log::info!(
                target: "sim_runtime::dispatch",
                "dispatch loop exited"
            );
        });

        Ok(Arc::new(Self {
            registry,
            descriptor,
            broker,
            actuation,
            health,
            session_id,
            _dispatch_task: dispatch,
        }))
    }

    /// Graceful shutdown — publishes one final `SimHealth` event with
    /// `backend_alive = false` so downstream subscribers see the
    /// transition promptly, then lets the supervisor observe the
    /// backend's termination separately.
    pub async fn shutdown(self: Arc<Self>) -> Result<(), SimRuntimeError> {
        let final_health = SimHealth {
            backend_alive: false,
            runtime_session_id: self.session_id.clone(),
            ..Default::default()
        };
        self.health.publish_health(final_health)?;
        Ok(())
    }

    pub fn world_descriptor(&self) -> &WorldDescriptor {
        &self.descriptor
    }

    pub fn subscribe_snapshots(&self) -> broadcast::Receiver<Arc<WorldSnapshot>> {
        self.broker.subscribe()
    }

    pub async fn send_actuation(&self, batch: ActuationBatch) -> Result<(), SimRuntimeError> {
        self.actuation.send(batch).await
    }

    pub fn subscribe_health(&self) -> broadcast::Receiver<SimHealth> {
        self.health.subscribe()
    }
}
