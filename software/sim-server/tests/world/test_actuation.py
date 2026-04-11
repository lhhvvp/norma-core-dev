"""Tests for ActuationApplier: proto batch → MjData.ctrl writes.

Split into ElRobot-strict variants (assert rev_motor_01..08 shape,
skipped until Chunk 5) and a Menagerie-loose variant covering the
robot-agnostic "unknown actuator" counter path."""
import pytest

try:
    from norma_sim.world._proto import world_pb  # noqa: F401
    from norma_sim.world.actuation import ActuationApplier
    from norma_sim.world.manifest import DEFAULT_ROBOT_ID
    from norma_sim.world.model import MuJoCoWorld
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"proto not importable: {_ERR}")


# --- ElRobot helpers ---

def _ref_elrobot(actuator_id: str) -> "world_pb.ActuatorRef":
    return world_pb.ActuatorRef(robot_id="elrobot_follower", actuator_id=actuator_id)


# --- ElRobot-strict tests (skipped until Chunk 5) ---

def test_apply_set_position_revolute_elrobot(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        as_of=None,
        commands=[
            world_pb.ActuationCommand(
                ref=_ref_elrobot("rev_motor_01"),
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


def test_apply_set_position_gripper_normalized_elrobot(elrobot_scene_yaml):
    """Gripper ctrl should receive the de-normalized rad value.
    ElRobot-specific: the 2.2028 rad value is ElRobot's primary joint range."""
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        commands=[
            world_pb.ActuationCommand(
                ref=_ref_elrobot("rev_motor_08"),
                set_position=world_pb.SetPosition(value=0.5, max_velocity=0.0),
            ),
        ],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 1
    idx = world.actuator_id_for("act_motor_08")
    assert world.data.ctrl[idx] == pytest.approx(2.2028 / 2, abs=1e-6)


def test_apply_command_without_intent_counts_unsupported_elrobot(elrobot_scene_yaml):
    """Uses rev_motor_01 ref but only asserts the unsupported_intent counter."""
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        commands=[world_pb.ActuationCommand(ref=_ref_elrobot("rev_motor_01"))],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 0
    assert stats.unsupported_intent == 1


def test_apply_multi_command_batch_elrobot(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        commands=[
            world_pb.ActuationCommand(
                ref=_ref_elrobot("rev_motor_01"),
                set_position=world_pb.SetPosition(value=0.3, max_velocity=0.0),
            ),
            world_pb.ActuationCommand(
                ref=_ref_elrobot("rev_motor_02"),
                set_position=world_pb.SetPosition(value=-0.2, max_velocity=0.0),
            ),
        ],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 2
    assert world.data.ctrl[world.actuator_id_for("act_motor_01")] == pytest.approx(0.3)
    assert world.data.ctrl[world.actuator_id_for("act_motor_02")] == pytest.approx(-0.2)


# --- Menagerie-loose tests (run immediately) ---

def test_apply_unknown_actuator_increments_counter_menagerie(menagerie_scene_yaml):
    """Robot-agnostic: send a command with a nonexistent actuator_id
    and verify the counter increments."""
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        commands=[
            world_pb.ActuationCommand(
                ref=world_pb.ActuatorRef(
                    robot_id=DEFAULT_ROBOT_ID,
                    actuator_id="definitely_not_an_actuator_name_xyz",
                ),
                set_position=world_pb.SetPosition(value=0.0, max_velocity=0.0),
            ),
        ],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 0
    assert stats.unknown_actuator == 1


def test_apply_set_position_revolute_menagerie(menagerie_scene_yaml):
    """Robot-agnostic: drive the FIRST REVOLUTE_POSITION actuator found
    in the MJCF to 0.1 rad and verify ctrl receives the value."""
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    rev_acts = [
        a for r in world.manifest.robots for a in r.actuators
        if a.capability.kind == "REVOLUTE_POSITION"
    ]
    assert rev_acts, "Menagerie MJCF should have at least one REVOLUTE_POSITION"
    target = rev_acts[0]
    applier = ActuationApplier(world)
    applier.drain_and_apply(world_pb.ActuationBatch(commands=[
        world_pb.ActuationCommand(
            ref=world_pb.ActuatorRef(
                robot_id=DEFAULT_ROBOT_ID,
                actuator_id=target.actuator_id,
            ),
            set_position=world_pb.SetPosition(value=0.1, max_velocity=0.0),
        ),
    ]))
    idx = world.actuator_id_for(target.mjcf_actuator)
    assert world.data.ctrl[idx] == pytest.approx(0.1, abs=1e-9)
