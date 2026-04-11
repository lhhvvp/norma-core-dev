"""Tests for SnapshotBuilder."""
import pytest

try:
    from norma_sim.world._proto import world_pb  # noqa: F401
    from norma_sim.world.actuation import ActuationApplier
    from norma_sim.world.model import MuJoCoWorld
    from norma_sim.world.snapshot import SnapshotBuilder
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"proto not importable: {_ERR}")


def test_snapshot_initial_state(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    builder = SnapshotBuilder(world)
    snap = builder.build(clock=None)
    assert len(snap.actuators) == 8
    # Every actuator state has the matching ref identity.
    ids = sorted(a.ref.actuator_id for a in snap.actuators)
    assert ids == [f"rev_motor_{i:02d}" for i in range(1, 9)]
    for a in snap.actuators:
        assert a.ref.robot_id == "elrobot_follower"


def test_snapshot_tracks_ctrl_goal(world_yaml_path):
    """After writing data.ctrl, the snapshot's goal_position_value
    should reflect the new target (converted back through
    capabilities)."""
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    applier = ActuationApplier(world)
    applier.drain_and_apply(
        world_pb.ActuationBatch(
            commands=[
                world_pb.ActuationCommand(
                    ref=world_pb.ActuatorRef(
                        robot_id="elrobot_follower",
                        actuator_id="rev_motor_01",
                    ),
                    set_position=world_pb.SetPosition(value=0.7, max_velocity=0.0),
                ),
            ],
        )
    )
    snap = SnapshotBuilder(world).build(clock=None)
    rev1 = next(a for a in snap.actuators if a.ref.actuator_id == "rev_motor_01")
    assert rev1.goal_position_value == pytest.approx(0.7, abs=1e-9)


def test_snapshot_gripper_reports_normalized(world_yaml_path):
    """Driving the gripper ctrl to joint mid-range should come back
    as 0.5 in the snapshot (since normalized_range=[0,1])."""
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    idx = world.actuator_id_for("act_motor_08")
    world.data.ctrl[idx] = 2.2028 / 2  # joint midpoint
    snap = SnapshotBuilder(world).build(clock=None)
    g = next(a for a in snap.actuators if a.ref.actuator_id == "rev_motor_08")
    assert g.goal_position_value == pytest.approx(0.5, abs=1e-6)


def test_snapshot_with_clock(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    clock = world_pb.WorldClock(world_tick=42, sim_time_ns=84_000_000, wall_time_ns=0)
    snap = SnapshotBuilder(world).build(clock=clock)
    assert snap.clock is not None
    assert snap.clock.world_tick == 42
