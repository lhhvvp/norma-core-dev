"""Tests for MuJoCoWorld wrapper."""
from norma_sim.world.model import MuJoCoWorld


def test_mujoco_world_loads_chunk1_mjcf(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    assert world.model.nu == 8, f"expected 8 actuators, got {world.model.nu}"
    assert world.model.neq == 2, f"expected 2 equality constraints, got {world.model.neq}"
    # Tendon-based mimic from Chunk 1: two fixed tendons for mimic joints.
    assert world.model.ntendon == 2


def test_mujoco_world_actuator_lookups(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    # act_motor_01..act_motor_08 must all resolve to a non-None actuator idx.
    for i in range(1, 9):
        name = f"act_motor_{i:02d}"
        idx = world.actuator_id_for(name)
        assert idx is not None, f"{name} missing from cache"
        assert 0 <= idx < world.model.nu


def test_mujoco_world_joint_qposadr_lookups(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    # All 8 manifest revolute/gripper primary joints are indexed.
    for i in range(1, 9):
        name = f"rev_motor_{i:02d}"
        addr = world.joint_qposadr_for(name)
        assert addr is not None, f"qposadr for {name} missing"


def test_mujoco_world_step_advances_time(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    t0 = float(world.data.time)
    for _ in range(10):
        world.step()
    t1 = float(world.data.time)
    assert t1 > t0


def test_mujoco_world_actuator_by_mjcf_name(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    gripper = world.actuator_by_mjcf_name("act_motor_08")
    assert gripper is not None
    assert gripper.capability.kind == "GRIPPER_PARALLEL"
    assert gripper.gripper is not None
    assert world.actuator_by_mjcf_name("nonexistent") is None
