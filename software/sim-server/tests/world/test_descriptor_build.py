"""Tests for build_world_descriptor: manifest → proto mapping.

Skip-gate: if the gremlin-generated proto module isn't importable
(e.g. `make protobuf` hasn't been run), the whole file is skipped
with an actionable reason.

Split into ElRobot-strict (assert 8 actuators, specific names,
M8 gripper limits) and Menagerie-loose (assert any valid descriptor
encodes). The sensors test is deleted because MVP-2 defers sensor
enumeration (spec §2.3)."""
import pytest

try:
    from norma_sim.world._proto import world_pb  # noqa: F401
    from norma_sim.world.descriptor import build_world_descriptor
    from norma_sim.world.manifest import load_manifest
    _PROTO_OK = True
    _PROTO_ERR = ""
except Exception as e:  # pragma: no cover
    _PROTO_OK = False
    _PROTO_ERR = str(e)


pytestmark = pytest.mark.skipif(
    not _PROTO_OK,
    reason=f"gremlin proto not importable; run 'make protobuf' first: {_PROTO_ERR}",
)


# --- ElRobot-strict (skipped until Chunk 5) ---

def test_build_world_descriptor_happy_elrobot(elrobot_scene_yaml):
    manifest = load_manifest(elrobot_scene_yaml)
    desc = build_world_descriptor(manifest)
    assert desc.world_name == "elrobot_follower"
    assert desc.publish_hz == 100
    assert desc.physics_hz == 500
    assert len(desc.robots) == 1
    robot = desc.robots[0]
    assert robot.robot_id == "elrobot_follower"
    assert len(robot.actuators) == 8


def test_build_world_descriptor_actuator_kinds_elrobot(elrobot_scene_yaml):
    manifest = load_manifest(elrobot_scene_yaml)
    desc = build_world_descriptor(manifest)
    robot = desc.robots[0]

    for i in range(7):
        kind = robot.actuators[i].capability.kind
        assert kind == world_pb.ActuatorCapability_Kind.CAP_REVOLUTE_POSITION, (
            f"actuator {i} unexpected kind {kind}"
        )

    m8 = robot.actuators[7]
    assert m8.capability.kind == world_pb.ActuatorCapability_Kind.CAP_GRIPPER_PARALLEL
    assert m8.capability.limit_min == 0.0
    assert m8.capability.limit_max == 1.0
    assert abs(m8.capability.effort_limit - 2.94) < 1e-9
    assert abs(m8.capability.velocity_limit - 4.71) < 1e-9


def test_build_world_descriptor_encodes_elrobot(elrobot_scene_yaml):
    """ElRobot strict: 8-actuator descriptor encodes to non-empty bytes."""
    manifest = load_manifest(elrobot_scene_yaml)
    desc = build_world_descriptor(manifest)
    buf = desc.encode()
    assert isinstance(buf, (bytes, bytearray))
    assert len(buf) > 0


# --- Menagerie-loose ---

def test_build_world_descriptor_happy_menagerie(menagerie_scene_yaml):
    """Generic: any valid MJCF produces a non-empty descriptor."""
    manifest = load_manifest(menagerie_scene_yaml)
    desc = build_world_descriptor(manifest)
    assert desc.world_name == "menagerie_test"
    assert desc.publish_hz == 100
    assert desc.physics_hz == 500
    assert len(desc.robots) == 1
    robot = desc.robots[0]
    assert len(robot.actuators) >= 5
    # default robot_id applies since Menagerie scene yaml doesn't set one
    assert robot.robot_id == "default_robot"


def test_build_world_descriptor_encodes_menagerie(menagerie_scene_yaml):
    """Generic: Menagerie descriptor encodes to non-empty bytes."""
    manifest = load_manifest(menagerie_scene_yaml)
    desc = build_world_descriptor(manifest)
    buf = desc.encode()
    assert isinstance(buf, (bytes, bytearray))
    assert len(buf) > 0


# test_build_world_descriptor_sensors DELETED:
# MVP-2 load_manifest hard-codes sensors=() because sensor enumeration
# from MJCF is deferred (spec §2.3). The "joint_state_all" sensor that
# MVP-1 gen.py placed in the yaml no longer exists in the manifest.
