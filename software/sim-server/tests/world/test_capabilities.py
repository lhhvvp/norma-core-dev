"""★ P0 tests for world.capabilities conversions.

These tests are the correctness guarantee for the unit mapping
between the sim-runtime's capability-keyed command vocabulary and
MuJoCo's native actuator control values. Breaking them means
clients will see silent physics/control misalignment.
"""
from dataclasses import replace

import pytest

from norma_sim.world.capabilities import command_value_to_ctrl, qpos_to_position_value
from norma_sim.world.manifest import (
    ActuatorCapability,
    ActuatorManifest,
    GripperMeta,
    GripperMimic,
)


def _revolute() -> ActuatorManifest:
    return ActuatorManifest(
        actuator_id="rev_motor_01",
        display_name="Test",
        mjcf_joint="rev_motor_01",
        mjcf_actuator="act_motor_01",
        capability=ActuatorCapability(kind="REVOLUTE_POSITION"),
        actuator_gains={"kp": 15.0, "kv": 0.5},
    )


def _gripper() -> ActuatorManifest:
    return ActuatorManifest(
        actuator_id="rev_motor_08",
        display_name="Gripper",
        mjcf_joint="rev_motor_08",
        mjcf_actuator="act_motor_08",
        capability=ActuatorCapability(kind="GRIPPER_PARALLEL"),
        actuator_gains={"kp": 10.0, "kv": 0.3},
        gripper=GripperMeta(
            primary_joint_range_rad=(0.0, 2.2028),
            normalized_range=(0.0, 1.0),
            mimic_joints=(
                GripperMimic(joint="rev_motor_08_1", multiplier=-0.0115),
                GripperMimic(joint="rev_motor_08_2", multiplier=0.0115),
            ),
        ),
    )


def test_capabilities_revolute_identity():
    a = _revolute()
    for v in [-1.5, -0.25, 0.0, 0.5, 1.5]:
        assert command_value_to_ctrl(v, a) == v
        assert qpos_to_position_value(v, a) == v


def test_capabilities_gripper_roundtrip():
    """★ P0: normalized command 0..1 → primary joint rad → normalized
    again must recover the original within float eps."""
    a = _gripper()
    for v in [0.0, 0.25, 0.5, 0.75, 1.0]:
        ctrl = command_value_to_ctrl(v, a)
        assert 0.0 <= ctrl <= 2.2028 + 1e-9, f"ctrl out of range for v={v}: {ctrl}"
        back = qpos_to_position_value(ctrl, a)
        assert abs(back - v) < 1e-9, f"roundtrip failed for v={v}: back={back}"


def test_capabilities_gripper_endpoints():
    a = _gripper()
    # 0.0 normalized → joint_lo
    assert abs(command_value_to_ctrl(0.0, a) - 0.0) < 1e-12
    # 1.0 normalized → joint_hi
    assert abs(command_value_to_ctrl(1.0, a) - 2.2028) < 1e-12
    # joint_lo → 0.0 normalized
    assert abs(qpos_to_position_value(0.0, a) - 0.0) < 1e-12
    # joint_hi → 1.0 normalized
    assert abs(qpos_to_position_value(2.2028, a) - 1.0) < 1e-12


def test_capabilities_gripper_missing_metadata_raises():
    """If gripper metadata is missing, a clear error is better than a
    silent NaN from None attribute access."""
    a = replace(_gripper(), gripper=None)
    with pytest.raises(ValueError, match="gripper metadata"):
        command_value_to_ctrl(0.5, a)
    with pytest.raises(ValueError, match="gripper metadata"):
        qpos_to_position_value(0.5, a)


def test_capabilities_unknown_kind_raises():
    a = replace(_revolute(), capability=ActuatorCapability(kind="CAMERA_RGB"))
    with pytest.raises(ValueError, match="unsupported capability kind"):
        command_value_to_ctrl(0.0, a)
