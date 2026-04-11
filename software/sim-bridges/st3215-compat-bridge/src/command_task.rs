//! Inbound command translation task.
//!
//! Subscribes to the global `commands` queue, filters entries to
//! the legacy ST3215 command type on our specific `target_bus_serial`,
//! and translates each into a capability-keyed `ActuationBatch` that
//! is forwarded to `SimulationRuntime::send_actuation`.
//!
//! Handled subcommands (MVP-1):
//!   - `write  { motor_id, address=GoalPosition }` → SetPosition
//!   - `write  { motor_id, address=TorqueEnable  }` → Enable/DisableTorque
//!   - `sync_write { address=GoalPosition, motors: [...] }` → batch SetPosition
//!   - `sync_write { address=TorqueEnable,  motors: [...] }` → batch Enable/DisableTorque
//!   - `reset { motor_id }` → ResetActuator
//!
//! Unrecognised addresses and `reg_write`/`action`/calibration commands
//! are silently skipped — station clients that need those features
//! cannot be run against the sim bus today (MVP-2 concern).
//!
//! QoS lane selection:
//!   - continuous setpoints (position) → `QosLossySetpoint` (drop-oldest)
//!   - discrete actions (torque enable, reset) → `QosReliableControl`

use crate::actuator_map::ActuatorMap;
use crate::errors::BridgeError;
use bytes::Bytes;
use normfs::NormFS;
use prost::Message;
use sim_runtime::proto::{
    actuation_command::Intent, ActuationBatch, ActuationCommand, ActuatorRef,
    DisableTorque, EnableTorque, QosLane, ResetActuator, SetPosition,
};
use sim_runtime::SimulationRuntime;
use station_iface::iface_proto::{commands, drivers};
use station_iface::COMMANDS_QUEUE_ID;
use st3215::st3215_proto::Command as StCommand;
use st3215_wire::RamRegister;
use std::sync::Arc;

pub async fn spawn_command_task(
    normfs: Arc<NormFS>,
    sim_runtime: Arc<SimulationRuntime>,
    actuator_map: Arc<ActuatorMap>,
    robot_id: String,
    legacy_bus_serial: String,
) -> Result<(), BridgeError> {
    let commands_queue_id = normfs.resolve(COMMANDS_QUEUE_ID);

    let rt_clone = sim_runtime.clone();
    let map_clone = actuator_map.clone();
    let bus_serial_clone = legacy_bus_serial.clone();
    let robot_id_clone = robot_id.clone();

    normfs
        .subscribe(
            &commands_queue_id,
            Box::new(move |entries: &[(normfs::UintN, Bytes)]| {
                for (_, data) in entries {
                    let pack = match commands::StationCommandsPack::decode(data.as_ref()) {
                        Ok(p) => p,
                        Err(e) => {
                            log::warn!(
                                target: "st3215_compat_bridge::command_task",
                                "decode StationCommandsPack: {}", e
                            );
                            continue;
                        }
                    };
                    for cmd in &pack.commands {
                        // Exactly mirror the real driver's filter
                        // (software/drivers/st3215/src/driver.rs:77).
                        if cmd.r#type() != drivers::StationCommandType::StcSt3215Command {
                            continue;
                        }
                        let st_cmd = match StCommand::decode(cmd.body.clone()) {
                            Ok(c) => c,
                            Err(e) => {
                                log::warn!(
                                    target: "st3215_compat_bridge::command_task",
                                    "decode st3215_proto::Command: {}", e
                                );
                                continue;
                            }
                        };
                        // Filter by target_bus_serial. This is required
                        // because in shadow mode the real driver AND the
                        // bridge both subscribe to the same `commands`
                        // queue; without this check the bridge would
                        // duplicate the real driver's work.
                        if st_cmd.target_bus_serial != bus_serial_clone {
                            continue;
                        }
                        let batch = match translate_command(
                            &st_cmd,
                            &map_clone,
                            &robot_id_clone,
                        ) {
                            Ok(b) => b,
                            Err(e) => {
                                log::warn!(
                                    target: "st3215_compat_bridge::command_task",
                                    "translate failed: {:?}", e
                                );
                                continue;
                            }
                        };
                        if batch.commands.is_empty() {
                            continue; // unhandled address
                        }
                        let rt = rt_clone.clone();
                        tokio::spawn(async move {
                            if let Err(e) = rt.send_actuation(batch).await {
                                log::error!(
                                    target: "st3215_compat_bridge::command_task",
                                    "send_actuation failed: {:?}", e
                                );
                            }
                        });
                    }
                }
                true // keep subscription alive
            }),
        )
        .map_err(|e| BridgeError::NormfsSubscribe(format!("{:?}", e)))?;

    Ok(())
}

/// Pure translation function — no I/O, no async, fully unit-testable.
pub(crate) fn translate_command(
    cmd: &StCommand,
    actuator_map: &ActuatorMap,
    robot_id: &str,
) -> Result<ActuationBatch, BridgeError> {
    let mut actuation_commands: Vec<ActuationCommand> = Vec::new();
    let mut lane = QosLane::QosLossySetpoint;

    // --- write: single motor_id + single address + single value ---
    if let Some(write) = &cmd.write {
        let motor_id = write.motor_id as u8;
        let addr = write.address as u8;
        let entry = actuator_map
            .get_by_motor_id(motor_id)
            .ok_or(BridgeError::UnknownMotorId(motor_id))?;

        if addr == RamRegister::GoalPosition.address() {
            let value_bytes = write.value.as_ref();
            let steps = u16::from_le_bytes([
                value_bytes.first().copied().unwrap_or(0),
                value_bytes.get(1).copied().unwrap_or(0),
            ]);
            let rad = st3215_wire::units::steps_to_rad(steps, entry.offset_steps);
            actuation_commands.push(build_set_position(robot_id, &entry.actuator_id, rad));
        } else if addr == RamRegister::TorqueEnable.address() {
            lane = QosLane::QosReliableControl;
            let enabled = write.value.as_ref().first().copied().unwrap_or(0) != 0;
            actuation_commands.push(build_torque(robot_id, &entry.actuator_id, enabled));
        }
        // Other addresses (GoalSpeed, Acc, …) are silently skipped in MVP-1.
    }

    // --- sync_write: single address + repeated per-motor values ---
    if let Some(sync) = &cmd.sync_write {
        let addr = sync.address as u8;
        if addr == RamRegister::GoalPosition.address() {
            for motor_write in &sync.motors {
                let motor_id = motor_write.motor_id as u8;
                let Some(entry) = actuator_map.get_by_motor_id(motor_id) else {
                    log::warn!(
                        target: "st3215_compat_bridge::command_task",
                        "sync_write: unknown motor_id {}, skipping", motor_id
                    );
                    continue;
                };
                let value_bytes = motor_write.value.as_ref();
                let steps = u16::from_le_bytes([
                    value_bytes.first().copied().unwrap_or(0),
                    value_bytes.get(1).copied().unwrap_or(0),
                ]);
                let rad = st3215_wire::units::steps_to_rad(steps, entry.offset_steps);
                actuation_commands.push(build_set_position(robot_id, &entry.actuator_id, rad));
            }
        } else if addr == RamRegister::TorqueEnable.address() {
            lane = QosLane::QosReliableControl;
            for motor_write in &sync.motors {
                let motor_id = motor_write.motor_id as u8;
                let Some(entry) = actuator_map.get_by_motor_id(motor_id) else {
                    continue;
                };
                let enabled =
                    motor_write.value.as_ref().first().copied().unwrap_or(0) != 0;
                actuation_commands.push(build_torque(robot_id, &entry.actuator_id, enabled));
            }
        }
    }

    // --- reset: single motor ---
    if let Some(reset) = &cmd.reset {
        lane = QosLane::QosReliableControl;
        let motor_id = reset.motor_id as u8;
        if let Some(entry) = actuator_map.get_by_motor_id(motor_id) {
            actuation_commands.push(ActuationCommand {
                r#ref: Some(ActuatorRef {
                    robot_id: robot_id.to_string(),
                    actuator_id: entry.actuator_id.clone(),
                }),
                intent: Some(Intent::ResetActuator(ResetActuator {})),
            });
        }
    }

    // reg_write / action / calibration commands are MVP-2 concerns.

    Ok(ActuationBatch {
        as_of: None,
        commands: actuation_commands,
        lane: lane as i32,
    })
}

fn build_set_position(robot_id: &str, actuator_id: &str, rad: f32) -> ActuationCommand {
    ActuationCommand {
        r#ref: Some(ActuatorRef {
            robot_id: robot_id.to_string(),
            actuator_id: actuator_id.to_string(),
        }),
        intent: Some(Intent::SetPosition(SetPosition {
            value: rad as f64,
            max_velocity: 0.0,
        })),
    }
}

fn build_torque(robot_id: &str, actuator_id: &str, enabled: bool) -> ActuationCommand {
    ActuationCommand {
        r#ref: Some(ActuatorRef {
            robot_id: robot_id.to_string(),
            actuator_id: actuator_id.to_string(),
        }),
        intent: Some(if enabled {
            Intent::EnableTorque(EnableTorque {})
        } else {
            Intent::DisableTorque(DisableTorque {})
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::preset_loader::{MotorEntry, RobotPreset};
    use st3215::st3215_proto::{
        st3215_sync_write_command::MotorWrite, St3215ResetCommand, St3215SyncWriteCommand,
        St3215WriteCommand,
    };

    fn single_motor_preset() -> RobotPreset {
        RobotPreset {
            robot_id: "elrobot_follower".into(),
            legacy_bus_serial: "sim://bus0".into(),
            motors: vec![MotorEntry {
                actuator_id: "rev_motor_01".into(),
                motor_id: 1,
                min_angle_steps: 0,
                max_angle_steps: 4095,
                offset_steps: 2048,
                torque_limit: 500,
                voltage_nominal_v: 12.0,
            }],
        }
    }

    fn two_motor_preset() -> RobotPreset {
        let mut p = single_motor_preset();
        p.motors.push(MotorEntry {
            actuator_id: "rev_motor_02".into(),
            motor_id: 2,
            min_angle_steps: 0,
            max_angle_steps: 4095,
            offset_steps: 2048,
            torque_limit: 500,
            voltage_nominal_v: 12.0,
        });
        p
    }

    fn make_write_cmd(motor_id: u32, address: u32, value: Vec<u8>) -> StCommand {
        StCommand {
            target_bus_serial: "sim://bus0".into(),
            write: Some(St3215WriteCommand {
                motor_id,
                address,
                value: Bytes::from(value),
            }),
            reg_write: None,
            action: None,
            reset: None,
            reset_calibration: None,
            freeze_calibration: None,
            sync_write: None,
            auto_calibrate: None,
            stop_auto_calibrate: None,
        }
    }

    #[test]
    fn test_translate_set_position_single_motor() {
        let preset = single_motor_preset();
        let map = ActuatorMap::from_preset(&preset);
        // rad_to_steps(0.5, 2048) ≈ 2374; encode little-endian
        let target_steps = st3215_wire::units::rad_to_steps(0.5, 2048);
        let steps_bytes = target_steps.to_le_bytes().to_vec();
        let cmd = make_write_cmd(1, RamRegister::GoalPosition.address() as u32, steps_bytes);

        let batch = translate_command(&cmd, &map, "elrobot_follower").unwrap();

        assert_eq!(batch.commands.len(), 1);
        assert_eq!(batch.lane, QosLane::QosLossySetpoint as i32);
        let c = &batch.commands[0];
        let r = c.r#ref.as_ref().unwrap();
        assert_eq!(r.robot_id, "elrobot_follower");
        assert_eq!(r.actuator_id, "rev_motor_01");
        match &c.intent {
            Some(Intent::SetPosition(sp)) => {
                assert!(
                    (sp.value - 0.5).abs() < 0.002,
                    "sp.value = {}",
                    sp.value
                );
            }
            other => panic!("expected SetPosition, got {:?}", other),
        }
    }

    #[test]
    fn test_translate_torque_enable_uses_reliable_lane() {
        let preset = single_motor_preset();
        let map = ActuatorMap::from_preset(&preset);
        let cmd = make_write_cmd(1, RamRegister::TorqueEnable.address() as u32, vec![1u8]);
        let batch = translate_command(&cmd, &map, "elrobot_follower").unwrap();
        assert_eq!(batch.lane, QosLane::QosReliableControl as i32);
        assert_eq!(batch.commands.len(), 1);
        assert!(matches!(
            &batch.commands[0].intent,
            Some(Intent::EnableTorque(_))
        ));
    }

    #[test]
    fn test_translate_torque_disable() {
        let preset = single_motor_preset();
        let map = ActuatorMap::from_preset(&preset);
        let cmd = make_write_cmd(1, RamRegister::TorqueEnable.address() as u32, vec![0u8]);
        let batch = translate_command(&cmd, &map, "elrobot_follower").unwrap();
        assert_eq!(batch.lane, QosLane::QosReliableControl as i32);
        assert!(matches!(
            &batch.commands[0].intent,
            Some(Intent::DisableTorque(_))
        ));
    }

    #[test]
    fn test_translate_unknown_motor_id_errors() {
        let preset = single_motor_preset();
        let map = ActuatorMap::from_preset(&preset);
        let cmd = make_write_cmd(99, RamRegister::GoalPosition.address() as u32, vec![0u8; 2]);
        assert!(matches!(
            translate_command(&cmd, &map, "elrobot_follower"),
            Err(BridgeError::UnknownMotorId(99))
        ));
    }

    #[test]
    fn test_translate_unknown_address_ignored() {
        let preset = single_motor_preset();
        let map = ActuatorMap::from_preset(&preset);
        // Address 0xFF is not one we handle; result should be an
        // empty batch (not an error).
        let cmd = make_write_cmd(1, 0xFF, vec![0u8; 2]);
        let batch = translate_command(&cmd, &map, "elrobot_follower").unwrap();
        assert!(batch.commands.is_empty());
    }

    #[test]
    fn test_translate_sync_write_multi_motor() {
        let preset = two_motor_preset();
        let map = ActuatorMap::from_preset(&preset);
        let steps = st3215_wire::units::rad_to_steps(0.25, 2048)
            .to_le_bytes()
            .to_vec();
        let cmd = StCommand {
            target_bus_serial: "sim://bus0".into(),
            write: None,
            reg_write: None,
            action: None,
            reset: None,
            reset_calibration: None,
            freeze_calibration: None,
            sync_write: Some(St3215SyncWriteCommand {
                address: RamRegister::GoalPosition.address() as u32,
                motors: vec![
                    MotorWrite {
                        motor_id: 1,
                        value: Bytes::from(steps.clone()),
                    },
                    MotorWrite {
                        motor_id: 2,
                        value: Bytes::from(steps),
                    },
                ],
            }),
            auto_calibrate: None,
            stop_auto_calibrate: None,
        };
        let batch = translate_command(&cmd, &map, "elrobot_follower").unwrap();
        assert_eq!(batch.commands.len(), 2);
        assert_eq!(batch.lane, QosLane::QosLossySetpoint as i32);
        for c in &batch.commands {
            match &c.intent {
                Some(Intent::SetPosition(sp)) => {
                    assert!((sp.value - 0.25).abs() < 0.002);
                }
                other => panic!("expected SetPosition, got {:?}", other),
            }
        }
    }

    #[test]
    fn test_translate_reset_uses_reliable_lane() {
        let preset = single_motor_preset();
        let map = ActuatorMap::from_preset(&preset);
        let cmd = StCommand {
            target_bus_serial: "sim://bus0".into(),
            write: None,
            reg_write: None,
            action: None,
            reset: Some(St3215ResetCommand {
                port_name: "".into(),
                motor_id: 1,
            }),
            reset_calibration: None,
            freeze_calibration: None,
            sync_write: None,
            auto_calibrate: None,
            stop_auto_calibrate: None,
        };
        let batch = translate_command(&cmd, &map, "elrobot_follower").unwrap();
        assert_eq!(batch.lane, QosLane::QosReliableControl as i32);
        assert_eq!(batch.commands.len(), 1);
        assert!(matches!(
            &batch.commands[0].intent,
            Some(Intent::ResetActuator(_))
        ));
    }
}
