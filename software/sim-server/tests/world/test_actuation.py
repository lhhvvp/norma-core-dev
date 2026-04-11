"""Tests for ActuationApplier: proto batch → MjData.ctrl writes."""
import pytest

try:
    from norma_sim.world._proto import world_pb  # noqa: F401
    from norma_sim.world.actuation import ActuationApplier
    from norma_sim.world.model import MuJoCoWorld
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"proto not importable: {_ERR}")


def _ref(actuator_id: str) -> "world_pb.ActuatorRef":
    return world_pb.ActuatorRef(robot_id="elrobot_follower", actuator_id=actuator_id)


def test_apply_set_position_revolute(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    applier = ActuationApplier(world)

    batch = world_pb.ActuationBatch(
        as_of=None,
        commands=[
            world_pb.ActuationCommand(
                ref=_ref("rev_motor_01"),
                set_position=world_pb.SetPosition(value=0.5, max_velocity=0.0),
            ),
        ],
        lane=world_pb.QosLane.QOS_LOSSY_SETPOINT,
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 1
    assert stats.unknown_actuator == 0
    idx = world.actuator_id_for("act_motor_01")
    assert world.data.ctrl[idx] == pytest.approx(0.5, abs=1e-9)


def test_apply_set_position_gripper_normalized(world_yaml_path):
    """Gripper ctrl should receive the de-normalized rad value."""
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    applier = ActuationApplier(world)

    batch = world_pb.ActuationBatch(
        commands=[
            world_pb.ActuationCommand(
                ref=_ref("rev_motor_08"),
                set_position=world_pb.SetPosition(value=0.5, max_velocity=0.0),
            ),
        ],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 1
    idx = world.actuator_id_for("act_motor_08")
    # 0.5 normalized → midpoint of primary_joint_range_rad [0, 2.2028]
    assert world.data.ctrl[idx] == pytest.approx(2.2028 / 2, abs=1e-6)


def test_apply_unknown_actuator_increments_counter(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        commands=[
            world_pb.ActuationCommand(
                ref=_ref("rev_motor_99"),
                set_position=world_pb.SetPosition(value=0.0, max_velocity=0.0),
            ),
        ],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 0
    assert stats.unknown_actuator == 1


def test_apply_command_without_intent_counts_unsupported(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        commands=[
            world_pb.ActuationCommand(ref=_ref("rev_motor_01")),
        ],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 0
    assert stats.unsupported_intent == 1


def test_apply_multi_command_batch(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        commands=[
            world_pb.ActuationCommand(
                ref=_ref("rev_motor_01"),
                set_position=world_pb.SetPosition(value=0.3, max_velocity=0.0),
            ),
            world_pb.ActuationCommand(
                ref=_ref("rev_motor_02"),
                set_position=world_pb.SetPosition(value=-0.2, max_velocity=0.0),
            ),
        ],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 2
    assert world.data.ctrl[world.actuator_id_for("act_motor_01")] == pytest.approx(0.3)
    assert world.data.ctrl[world.actuator_id_for("act_motor_02")] == pytest.approx(-0.2)
