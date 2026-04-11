"""Tests for world.manifest loader."""
from pathlib import Path

import pytest
import yaml

from norma_sim.world.manifest import (
    ActuatorCapability,
    GripperMeta,
    GripperMimic,
    WorldManifest,
    load_manifest,
)


def test_manifest_load_happy(world_yaml_path):
    manifest = load_manifest(world_yaml_path)
    assert isinstance(manifest, WorldManifest)
    assert manifest.world_name == "elrobot_follower_empty"
    assert len(manifest.robots) == 1
    robot = manifest.robots[0]
    assert robot.robot_id == "elrobot_follower"
    assert len(robot.actuators) == 8
    # Motor 8 is the gripper with explicit metadata.
    m8 = robot.actuators[7]
    assert m8.actuator_id == "rev_motor_08"
    assert m8.capability.kind == "GRIPPER_PARALLEL"
    assert m8.gripper is not None
    assert m8.gripper.primary_joint_range_rad == (0.0, 2.2028)
    assert len(m8.gripper.mimic_joints) == 2
    multipliers = sorted(m.multiplier for m in m8.gripper.mimic_joints)
    assert multipliers == [-0.0115, 0.0115]


def test_manifest_scene_config(world_yaml_path):
    manifest = load_manifest(world_yaml_path)
    assert manifest.scene.timestep == 0.002
    assert manifest.scene.integrator == "RK4"
    assert manifest.scene.solver == "Newton"
    assert manifest.scene.iterations == 50


def test_manifest_missing_gripper_fields_raises(tmp_path):
    """A GRIPPER_PARALLEL capability without a `gripper:` block must
    raise a clear ValueError so users know how to fix it."""
    bad = {
        "world_name": "bad",
        "urdf_source": "../elrobot_follower.urdf",
        "mjcf_output": "./out.xml",
        "scene": {
            "timestep": 0.002,
            "gravity": [0, 0, -9.81],
            "integrator": "RK4",
            "solver": "Newton",
            "iterations": 50,
        },
        "robots": [
            {
                "robot_id": "x",
                "actuators": [
                    {
                        "actuator_id": "rev_motor_08",
                        "display_name": "Gripper",
                        "urdf_joint": "rev_motor_08",
                        "mjcf_actuator": "act_motor_08",
                        "capability": {"kind": "GRIPPER_PARALLEL"},
                        "actuator_gains": {"kp": 10.0, "kv": 0.3},
                        # INTENTIONALLY no 'gripper' block
                    }
                ],
            }
        ],
    }
    p = tmp_path / "bad.yaml"
    p.write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError, match="GRIPPER_PARALLEL"):
        load_manifest(p)
