//! Outbound snapshot translation task.
//!
//! Subscribes to `SimulationRuntime::subscribe_snapshots`, packs
//! each WorldSnapshot's ActuatorStates into the exact 71-byte
//! ST3215 memory dumps legacy station clients expect, wraps them
//! in the `InferenceState { buses: [BusState { motors: [...] }] }`
//! envelope the real driver emits, and enqueues the serialised
//! bytes into the `st3215/inference` NormFS queue.

use crate::actuator_map::ActuatorMap;
use crate::errors::BridgeError;
use bytes::Bytes;
use normfs::NormFS;
use prost::Message;
use sim_runtime::proto::WorldSnapshot;
use sim_runtime::SimulationRuntime;
use station_iface::iface_proto::drivers::QueueDataType;
use station_iface::StationEngine;
use st3215::st3215_proto::{
    inference_state::{BusState, MotorState},
    InferenceState, St3215Bus,
};
use st3215_wire::{
    pack::MotorInstance, pack_state_bytes, presets::ST3215_STANDARD, MotorSemanticState,
};
use std::sync::Arc;

pub(crate) const INFERENCE_QUEUE_ID: &str = "st3215/inference";

pub async fn spawn_state_task(
    normfs: Arc<NormFS>,
    engine: Arc<dyn StationEngine>,
    sim_runtime: Arc<SimulationRuntime>,
    actuator_map: Arc<ActuatorMap>,
    robot_id: String,
    legacy_bus_serial: String,
) -> Result<tokio::task::JoinHandle<()>, BridgeError> {
    let inference_qid = normfs.resolve(INFERENCE_QUEUE_ID);
    normfs
        .ensure_queue_exists_for_write(&inference_qid)
        .await
        .map_err(|e| BridgeError::NormfsSubscribe(format!("inference queue: {:?}", e)))?;

    // CRITICAL: register the queue with Station's inference aggregator
    // (mirrors `software/drivers/st3215/src/driver.rs:54-59`). Without
    // this, Station's `Inference` module never subscribes to our queue,
    // never includes our bridge's writes in the `InferenceRx` published
    // to `inference-states`, and the web UI's `inferenceState.st3215`
    // field stays empty — triggering the "connect a robot" empty state.
    engine.register_queue(&inference_qid, QueueDataType::QdtSt3215Inference, vec![]);

    let mut snapshot_rx = sim_runtime.subscribe_snapshots();
    let handle = tokio::spawn(async move {
        loop {
            match snapshot_rx.recv().await {
                Ok(snapshot) => {
                    let payload = match build_inference_bytes(
                        &snapshot,
                        &actuator_map,
                        &robot_id,
                        &legacy_bus_serial,
                    ) {
                        Ok(bytes) => bytes,
                        Err(e) => {
                            log::warn!(
                                target: "st3215_compat_bridge::state_task",
                                "build_inference_bytes: {:?}", e
                            );
                            continue;
                        }
                    };
                    if let Err(e) = normfs.enqueue(&inference_qid, payload) {
                        log::warn!(
                            target: "st3215_compat_bridge::state_task",
                            "enqueue st3215/inference: {:?}", e
                        );
                    }
                }
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                    log::warn!(
                        target: "st3215_compat_bridge::state_task",
                        "snapshot subscriber lagged, dropped {} frames", n
                    );
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    log::info!(
                        target: "st3215_compat_bridge::state_task",
                        "snapshot channel closed, exiting"
                    );
                    return;
                }
            }
        }
    });
    Ok(handle)
}

/// Pure builder used by both the task and unit tests. Takes a
/// WorldSnapshot and produces the serialised InferenceState bytes
/// that legacy clients would observe on `st3215/inference`.
pub(crate) fn build_inference_bytes(
    snapshot: &WorldSnapshot,
    actuator_map: &ActuatorMap,
    robot_id: &str,
    legacy_bus_serial: &str,
) -> Result<Bytes, BridgeError> {
    let mut motors: Vec<MotorState> = Vec::with_capacity(actuator_map.len());

    for actuator_state in &snapshot.actuators {
        let Some(ref_) = actuator_state.r#ref.as_ref() else {
            continue;
        };
        if ref_.robot_id != robot_id {
            continue;
        }
        let Some(motor_id) = actuator_map.get_motor_id_by_actuator(&ref_.actuator_id) else {
            continue;
        };
        let Some(entry) = actuator_map.get_by_motor_id(motor_id) else {
            continue;
        };

        let instance = MotorInstance {
            min_angle_steps: entry.min_angle_steps,
            max_angle_steps: entry.max_angle_steps,
            offset_steps: entry.offset_steps,
            torque_limit: entry.torque_limit,
            voltage_nominal_v: entry.voltage_nominal_v,
        };
        let semantic = MotorSemanticState {
            position_rad: actuator_state.position_value as f32,
            velocity_rad_s: actuator_state.velocity_value as f32,
            load_nm: actuator_state.effort_value as f32,
            temperature_c: 25.0,
            torque_enabled: actuator_state.torque_enabled,
            moving: actuator_state.moving,
            goal_position_rad: actuator_state.goal_position_value as f32,
            goal_speed_rad_s: 0.0,
        };

        let bytes = pack_state_bytes(motor_id, &ST3215_STANDARD, &instance, &semantic);
        motors.push(MotorState {
            id: motor_id as u32,
            rx_pointer: Bytes::new(),
            monotonic_stamp_ns: 0,
            system_stamp_ns: 0,
            app_start_id: 0,
            state: bytes,
            error: None,
            range_min: entry.min_angle_steps as u32,
            range_max: entry.max_angle_steps as u32,
            range_freezed: true,
            last_command: None,
        });
    }

    let inference = InferenceState {
        last_inference_queue_ptr: Bytes::new(),
        buses: vec![BusState {
            bus: Some(St3215Bus {
                port_name: String::new(),
                vid: 0,
                pid: 0,
                serial_number: legacy_bus_serial.to_string(),
                manufacturer: "norma-sim".into(),
                product: "st3215-compat-bridge".into(),
                port_baud_rate: 0,
            }),
            monotonic_stamp_ns: 0,
            system_stamp_ns: 0,
            app_start_id: 0,
            motors,
            auto_calibration: None,
        }],
    };
    let mut buf = Vec::with_capacity(inference.encoded_len());
    inference
        .encode(&mut buf)
        .map_err(|e| BridgeError::NormfsSubscribe(format!("encode InferenceState: {}", e)))?;
    Ok(Bytes::from(buf))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::preset_loader::{MotorEntry, RobotPreset};
    use sim_runtime::proto::{ActuatorRef, ActuatorState, WorldClock};

    fn two_motor_map() -> ActuatorMap {
        ActuatorMap::from_preset(&RobotPreset {
            robot_id: "elrobot_follower".into(),
            legacy_bus_serial: "sim://bus0".into(),
            motors: vec![
                MotorEntry {
                    actuator_id: "rev_motor_01".into(),
                    motor_id: 1,
                    min_angle_steps: 0,
                    max_angle_steps: 4095,
                    offset_steps: 2048,
                    torque_limit: 500,
                    voltage_nominal_v: 12.0,
                },
                MotorEntry {
                    actuator_id: "rev_motor_08".into(),
                    motor_id: 8,
                    min_angle_steps: 0,
                    max_angle_steps: 4095,
                    offset_steps: 0,
                    torque_limit: 500,
                    voltage_nominal_v: 12.0,
                },
            ],
        })
    }

    fn snapshot_with(pos1: f32, pos8: f32) -> WorldSnapshot {
        WorldSnapshot {
            clock: Some(WorldClock {
                world_tick: 10,
                sim_time_ns: 20_000_000,
                wall_time_ns: 0,
            }),
            actuators: vec![
                ActuatorState {
                    r#ref: Some(ActuatorRef {
                        robot_id: "elrobot_follower".into(),
                        actuator_id: "rev_motor_01".into(),
                    }),
                    position_value: pos1 as f64,
                    velocity_value: 0.0,
                    effort_value: 0.0,
                    torque_enabled: true,
                    moving: false,
                    goal_position_value: pos1 as f64,
                },
                ActuatorState {
                    r#ref: Some(ActuatorRef {
                        robot_id: "elrobot_follower".into(),
                        actuator_id: "rev_motor_08".into(),
                    }),
                    position_value: pos8 as f64,
                    velocity_value: 0.0,
                    effort_value: 0.0,
                    torque_enabled: true,
                    moving: false,
                    goal_position_value: pos8 as f64,
                },
            ],
            sensors: vec![],
        }
    }

    #[test]
    fn test_build_inference_bytes_shape() {
        let map = two_motor_map();
        let snap = snapshot_with(0.5, 1.0);
        let bytes = build_inference_bytes(&snap, &map, "elrobot_follower", "sim://bus0").unwrap();

        let inference = InferenceState::decode(bytes.as_ref()).unwrap();
        assert_eq!(inference.buses.len(), 1);
        let bus = &inference.buses[0];
        assert_eq!(
            bus.bus.as_ref().unwrap().serial_number,
            "sim://bus0"
        );
        assert_eq!(bus.motors.len(), 2);
        // Every motor's state payload is exactly 71 bytes.
        for m in &bus.motors {
            assert_eq!(
                m.state.len(),
                st3215_wire::TOTAL_BYTES,
                "motor {} state payload wrong size",
                m.id
            );
        }
    }

    #[test]
    fn test_build_inference_bytes_roundtrip_position() {
        let map = two_motor_map();
        let snap = snapshot_with(0.5, 1.0);
        let bytes = build_inference_bytes(&snap, &map, "elrobot_follower", "sim://bus0").unwrap();

        let inference = InferenceState::decode(bytes.as_ref()).unwrap();
        let bus = &inference.buses[0];

        // Unpack motor 1 and verify the position round-trips through
        // the ST3215 wire format.
        let m1 = bus.motors.iter().find(|m| m.id == 1).unwrap();
        let instance = MotorInstance {
            min_angle_steps: 0,
            max_angle_steps: 4095,
            offset_steps: 2048,
            torque_limit: 500,
            voltage_nominal_v: 12.0,
        };
        let decoded = st3215_wire::unpack_state_bytes(
            m1.state.as_ref(),
            &ST3215_STANDARD,
            &instance,
        )
        .unwrap();
        assert!(
            (decoded.position_rad - 0.5).abs() < 0.002,
            "rev_motor_01 roundtrip: got {} rad",
            decoded.position_rad
        );

        // Unpack motor 8 (gripper, offset_steps=0)
        let m8 = bus.motors.iter().find(|m| m.id == 8).unwrap();
        let instance_gripper = MotorInstance {
            min_angle_steps: 0,
            max_angle_steps: 4095,
            offset_steps: 0,
            torque_limit: 500,
            voltage_nominal_v: 12.0,
        };
        let decoded_gripper = st3215_wire::unpack_state_bytes(
            m8.state.as_ref(),
            &ST3215_STANDARD,
            &instance_gripper,
        )
        .unwrap();
        assert!(
            (decoded_gripper.position_rad - 1.0).abs() < 0.002,
            "rev_motor_08 roundtrip: got {} rad",
            decoded_gripper.position_rad
        );
    }

    #[test]
    fn test_build_inference_bytes_filters_wrong_robot() {
        let map = two_motor_map();
        let mut snap = snapshot_with(0.0, 0.0);
        snap.actuators[0].r#ref.as_mut().unwrap().robot_id = "other_robot".into();
        let bytes = build_inference_bytes(&snap, &map, "elrobot_follower", "sim://bus0").unwrap();
        let inference = InferenceState::decode(bytes.as_ref()).unwrap();
        // Only motor 8 should survive — rev_motor_01 had robot_id=other_robot.
        assert_eq!(inference.buses[0].motors.len(), 1);
        assert_eq!(inference.buses[0].motors[0].id, 8);
    }
}
