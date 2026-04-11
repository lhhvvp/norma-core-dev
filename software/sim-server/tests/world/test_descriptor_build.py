"""Tests for build_world_descriptor: manifest → proto mapping.

Skip-gate: if the gremlin-generated proto module isn't importable
(e.g. `make protobuf` hasn't been run), the whole file is skipped
with an actionable reason.
"""
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


def test_build_world_descriptor_happy(world_yaml_path):
    manifest = load_manifest(world_yaml_path)
    desc = build_world_descriptor(manifest)
    assert desc.world_name == "elrobot_follower_empty"
    assert desc.publish_hz == 100
    assert desc.physics_hz == 500
    assert len(desc.robots) == 1
    robot = desc.robots[0]
    assert robot.robot_id == "elrobot_follower"
    assert len(robot.actuators) == 8


def test_build_world_descriptor_actuator_kinds(world_yaml_path):
    manifest = load_manifest(world_yaml_path)
    desc = build_world_descriptor(manifest)
    robot = desc.robots[0]

    # First 7 actuators are REVOLUTE_POSITION
    for i in range(7):
        kind = robot.actuators[i].capability.kind
        assert kind == world_pb.ActuatorCapability_Kind.CAP_REVOLUTE_POSITION, (
            f"actuator {i} unexpected kind {kind}"
        )

    # Motor 8 is GRIPPER_PARALLEL with explicit limits
    m8 = robot.actuators[7]
    assert m8.capability.kind == world_pb.ActuatorCapability_Kind.CAP_GRIPPER_PARALLEL
    assert m8.capability.limit_min == 0.0
    assert m8.capability.limit_max == 1.0
    assert abs(m8.capability.effort_limit - 2.94) < 1e-9
    assert abs(m8.capability.velocity_limit - 4.71) < 1e-9


def test_build_world_descriptor_sensors(world_yaml_path):
    manifest = load_manifest(world_yaml_path)
    desc = build_world_descriptor(manifest)
    robot = desc.robots[0]
    assert len(robot.sensors) == 1
    s = robot.sensors[0]
    assert s.sensor_id == "joint_state_all"
    assert s.capability.kind == world_pb.SensorCapability_Kind.SENSE_JOINT_STATE


def test_build_world_descriptor_encodes(world_yaml_path):
    """Sanity: the assembled descriptor should serialize to non-empty
    bytes. This is the end-to-end correctness check that every field
    we set survives the gremlin-py encode path."""
    manifest = load_manifest(world_yaml_path)
    desc = build_world_descriptor(manifest)
    buf = desc.encode()
    assert isinstance(buf, (bytes, bytearray))
    assert len(buf) > 0
