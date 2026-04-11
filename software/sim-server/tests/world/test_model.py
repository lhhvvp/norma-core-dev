"""Tests for MuJoCoWorld wrapper. Split into ElRobot-strict (assert
specific 8-actuator shape) and Menagerie-loose (assert any valid
MuJoCoWorld) variants. The ElRobot variants skip until Chunk 5."""
from norma_sim.world.model import MuJoCoWorld


# --- ElRobot-strict tests (skip until Chunk 5) --------------------

def test_mujoco_world_loads_elrobot_mjcf(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    assert world.model.nu == 8
    assert world.model.neq == 2
    assert world.model.ntendon == 2


def test_mujoco_world_elrobot_actuator_lookups(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    for i in range(1, 9):
        name = f"act_motor_{i:02d}"
        idx = world.actuator_id_for(name)
        assert idx is not None, f"{name} missing from cache"
        assert 0 <= idx < world.model.nu


def test_mujoco_world_elrobot_joint_qposadr_lookups(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    for i in range(1, 9):
        name = f"rev_motor_{i:02d}"
        addr = world.joint_qposadr_for(name)
        assert addr is not None, f"qposadr for {name} missing"


def test_mujoco_world_elrobot_actuator_by_mjcf_name(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    gripper = world.actuator_by_mjcf_name("act_motor_08")
    assert gripper is not None
    assert gripper.capability.kind == "GRIPPER_PARALLEL"
    assert gripper.gripper is not None
    assert world.actuator_by_mjcf_name("nonexistent") is None


# --- Menagerie-loose tests (run immediately) ----------------------

def test_mujoco_world_loads_menagerie_mjcf(menagerie_scene_yaml):
    """Menagerie trs_so_arm100 should load as a valid MuJoCoWorld with
    at least 5 actuators (SO-ARM100 class). No gripper assertion —
    Menagerie's gripper is a plain single-DOF revolute + <position>."""
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    assert world.model.nu >= 5


def test_mujoco_world_menagerie_actuator_lookups(menagerie_scene_yaml):
    """Every auto-synthesized revolute actuator should resolve to a
    valid cache entry."""
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    for robot in world.manifest.robots:
        for act in robot.actuators:
            idx = world.actuator_id_for(act.mjcf_actuator)
            assert idx is not None
            addr = world.joint_qposadr_for(act.mjcf_joint)
            assert addr is not None


def test_mujoco_world_step_advances_time(menagerie_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    t0 = float(world.data.time)
    for _ in range(10):
        world.step()
    t1 = float(world.data.time)
    assert t1 > t0
