//! ★★★ E2E: legacy station_py commands flow through the bridge.
//!
//! This is the codex-requested acceptance test for the Chunk 7
//! compat story: an unmodified legacy client enqueuing a
//! StationCommandsPack with a ST3215WriteCommand must result in a
//! capability-keyed ActuationBatch arriving at the sim backend.
//!
//! Harness:
//!   1. tempdir-backed NormFS + no-op StationEngine
//!   2. MockBackend with canned WorldDescriptor that the preset file
//!      (rev_motor_01..rev_motor_08) validates against
//!   3. SimulationRuntime::start_with_mock (feature `test-util`)
//!   4. start_st3215_compat_bridge with sim://bus0 legacy serial
//!   5. Direct NormFS::enqueue into the "commands" queue bypasses any
//!      station_py wire-level dependency — we use the same wire
//!      format station_py would and the bridge's subscribe callback
//!      triggers on the entry
//!   6. MockBackend.outbound_observer channel receives the
//!      translated Envelope; we decode and assert on the fields.

use bytes::Bytes;
use normfs::{NormFS, NormFsSettings};
use prost::Message;
use sim_runtime::proto::{actuation_command::Intent, envelope::Payload, QosLane};
use sim_runtime::test_util::MockBackend;
use sim_runtime::SimulationRuntime;
use station_iface::iface_proto::{commands, drivers};
use station_iface::{config::St3215CompatBridgeConfig, COMMANDS_QUEUE_ID};
use station_iface::StationEngine;
use st3215::st3215_proto::{Command as StCommand, St3215WriteCommand};
use st3215_compat_bridge::start_st3215_compat_bridge;
use st3215_wire::RamRegister;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

// ─── shared helpers ──────────────────────────────────────────────────

struct NoopEngine;
impl StationEngine for NoopEngine {
    fn register_queue(
        &self,
        _queue_id: &normfs::QueueId,
        _queue_data_type: station_iface::iface_proto::drivers::QueueDataType,
        _opts: Vec<station_iface::iface_proto::envelope::QueueOpt>,
    ) {
        // No-op; this integration test doesn't observe the engine.
    }
}

async fn make_normfs() -> (Arc<NormFS>, tempfile::TempDir) {
    let tmp = tempfile::tempdir().expect("tempdir");
    let settings = NormFsSettings::default();
    let fs = NormFS::new(tmp.path().to_path_buf(), settings)
        .await
        .expect("NormFS::new");
    (Arc::new(fs), tmp)
}

fn fake_world_descriptor() -> sim_runtime::proto::WorldDescriptor {
    use sim_runtime::proto::{
        actuator_capability, ActuatorCapability, ActuatorDescriptor, RobotDescriptor,
        WorldClock, WorldDescriptor,
    };

    let actuators: Vec<ActuatorDescriptor> = (1..=8)
        .map(|i| ActuatorDescriptor {
            actuator_id: format!("rev_motor_{:02}", i),
            display_name: format!("Motor {}", i),
            capability: Some(ActuatorCapability {
                kind: actuator_capability::Kind::CapRevolutePosition as i32,
                limit_min: -3.0,
                limit_max: 3.0,
                effort_limit: 2.94,
                velocity_limit: 4.71,
            }),
        })
        .collect();

    WorldDescriptor {
        world_name: "elrobot_follower_empty".into(),
        robots: vec![RobotDescriptor {
            robot_id: "elrobot_follower".into(),
            actuators,
            sensors: vec![],
        }],
        initial_clock: Some(WorldClock {
            world_tick: 0,
            sim_time_ns: 0,
            wall_time_ns: 0,
        }),
        publish_hz: 100,
        physics_hz: 500,
    }
}

fn preset_path() -> PathBuf {
    // Walk up from CARGO_MANIFEST_DIR = software/station/bin/station/
    // to the repo root (4 levels: station → bin → station → software
    // → repo-root), then down into the bridge preset.
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    manifest
        .ancestors()
        .nth(4)
        .expect("repo root")
        .join("software/sim-bridges/st3215-compat-bridge/presets/elrobot-follower.yaml")
}

fn build_write_pack(
    target_bus_serial: &str,
    motor_id: u32,
    steps: u16,
) -> Bytes {
    let st_cmd = StCommand {
        target_bus_serial: target_bus_serial.into(),
        write: Some(St3215WriteCommand {
            motor_id,
            address: RamRegister::GoalPosition.address() as u32,
            value: Bytes::from(steps.to_le_bytes().to_vec()),
        }),
        reg_write: None,
        action: None,
        reset: None,
        reset_calibration: None,
        freeze_calibration: None,
        sync_write: None,
        auto_calibrate: None,
        stop_auto_calibrate: None,
    };

    let mut body_bytes = Vec::with_capacity(st_cmd.encoded_len());
    st_cmd.encode(&mut body_bytes).expect("encode StCommand");

    let driver_cmd = commands::DriverCommand {
        command_id: Bytes::from(vec![0u8; 8]),
        r#type: drivers::StationCommandType::StcSt3215Command as i32,
        body: Bytes::from(body_bytes),
    };

    let pack = commands::StationCommandsPack {
        inference_state_id: Bytes::new(),
        pack_id: Bytes::from(vec![0u8; 8]),
        commands: vec![driver_cmd],
        tags: vec![],
    };

    let mut pack_bytes = Vec::with_capacity(pack.encoded_len());
    pack.encode(&mut pack_bytes).expect("encode pack");
    Bytes::from(pack_bytes)
}

async fn boot_sim_and_bridge(
    normfs: Arc<NormFS>,
    legacy_bus_serial: &str,
) -> (
    Arc<SimulationRuntime>,
    tokio::sync::mpsc::Receiver<sim_runtime::proto::Envelope>,
    Arc<st3215_compat_bridge::BridgeHandle>,
) {
    // MockBackend with outbound observer so we can see what the bridge
    // produced.
    let (obs_tx, obs_rx) = tokio::sync::mpsc::channel(64);
    let mut mock = MockBackend::new(fake_world_descriptor());
    mock.outbound_observer = Some(obs_tx);

    let sim = SimulationRuntime::start_with_mock(
        normfs.clone(),
        mock,
        "test-session".into(),
    )
    .await
    .expect("SimulationRuntime starts");

    // The commands queue must exist before the bridge subscribes.
    let commands_qid = normfs.resolve(COMMANDS_QUEUE_ID);
    normfs
        .ensure_queue_exists_for_write(&commands_qid)
        .await
        .expect("commands queue");

    let bridge_cfg = St3215CompatBridgeConfig {
        enabled: true,
        robot_id: "elrobot_follower".into(),
        preset_path: preset_path(),
        legacy_bus_serial: legacy_bus_serial.into(),
    };
    let engine: Arc<dyn StationEngine> = Arc::new(NoopEngine);
    let bridge = start_st3215_compat_bridge(
        normfs.clone(),
        engine,
        sim.clone(),
        bridge_cfg,
    )
    .await
    .expect("bridge starts");

    (sim, obs_rx, bridge)
}

async fn expect_actuation(
    obs_rx: &mut tokio::sync::mpsc::Receiver<sim_runtime::proto::Envelope>,
    timeout: Duration,
) -> sim_runtime::proto::ActuationBatch {
    let env = tokio::time::timeout(timeout, obs_rx.recv())
        .await
        .expect("observer did not see actuation within timeout")
        .expect("observer channel closed");
    match env.payload {
        Some(Payload::Actuation(b)) => b,
        other => panic!("expected Actuation, got {:?}", other),
    }
}

// ─── tests ───────────────────────────────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn test_legacy_station_py_commands_flow_through_bridge() {
    let (normfs, _tmp) = make_normfs().await;
    let (_sim, mut obs_rx, _bridge) =
        boot_sim_and_bridge(normfs.clone(), "sim://bus0").await;

    // Enqueue a StationCommandsPack the way station_py would — a
    // DriverCommand{type=StcSt3215Command, body=StCommand} wrapping
    // a GoalPosition write on rev_motor_01 (motor_id=1 per preset).
    // Target steps = rad_to_steps(0.5, offset=2048) ≈ 2374.
    let steps = st3215_wire::units::rad_to_steps(0.5, 2048);
    let pack_bytes = build_write_pack("sim://bus0", 1, steps);

    let commands_qid = normfs.resolve(COMMANDS_QUEUE_ID);
    normfs
        .enqueue(&commands_qid, pack_bytes)
        .expect("enqueue pack");

    // Observer must see exactly one translated actuation.
    let batch = expect_actuation(&mut obs_rx, Duration::from_secs(2)).await;
    assert_eq!(batch.lane, QosLane::QosLossySetpoint as i32);
    assert_eq!(batch.commands.len(), 1);
    let cmd = &batch.commands[0];
    let r = cmd.r#ref.as_ref().expect("ref set");
    assert_eq!(r.robot_id, "elrobot_follower");
    assert_eq!(r.actuator_id, "rev_motor_01");
    match &cmd.intent {
        Some(Intent::SetPosition(sp)) => {
            assert!(
                (sp.value - 0.5).abs() < 0.002,
                "SetPosition.value = {}, expected ≈ 0.5",
                sp.value
            );
        }
        other => panic!("expected SetPosition, got {:?}", other),
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn test_shadow_mode_target_bus_serial_routing() {
    // Shadow mode: bridge is bound to sim://elrobot-shadow. A command
    // targeting that bus reaches the MockBackend; a command targeting
    // the real EEPROM serial does NOT.
    let (normfs, _tmp) = make_normfs().await;
    let (_sim, mut obs_rx, _bridge) =
        boot_sim_and_bridge(normfs.clone(), "sim://elrobot-shadow").await;

    let commands_qid = normfs.resolve(COMMANDS_QUEUE_ID);
    let steps = st3215_wire::units::rad_to_steps(0.5, 2048);

    // Command 1: targets the REAL hardware bus — MUST NOT reach us.
    normfs
        .enqueue(
            &commands_qid,
            build_write_pack("ST3215-BUS-A1B2C3", 1, steps),
        )
        .expect("enqueue real");

    // Command 2: targets the sim shadow bus — MUST reach us.
    normfs
        .enqueue(
            &commands_qid,
            build_write_pack("sim://elrobot-shadow", 1, steps),
        )
        .expect("enqueue sim");

    // We should observe exactly one actuation — the sim one.
    let batch = expect_actuation(&mut obs_rx, Duration::from_secs(2)).await;
    assert_eq!(batch.commands.len(), 1);

    // Drain any stragglers (expect none within a short window).
    let stragglers =
        tokio::time::timeout(Duration::from_millis(250), obs_rx.recv()).await;
    assert!(
        stragglers.is_err(),
        "bridge forwarded a command destined for the real hardware bus: {:?}",
        stragglers
    );
}
