"""Tests for the stepping scheduler."""
import pytest

try:
    from norma_sim.scheduler.stepping import SteppingScheduler
    from norma_sim.world.actuation import ActuationApplier
    from norma_sim.world.model import MuJoCoWorld
    from norma_sim.world.snapshot import SnapshotBuilder
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"stepping scheduler imports failed: {_ERR}")


def _make_scheduler(scene_yaml) -> "SteppingScheduler":
    world = MuJoCoWorld.from_manifest_path(scene_yaml)
    applier = ActuationApplier(world)
    builder = SnapshotBuilder(world)
    return SteppingScheduler(world, applier=applier, builder=builder, physics_hz=500)


def test_step_advances_tick(menagerie_scene_yaml):
    sched = _make_scheduler(menagerie_scene_yaml)
    assert sched.tick == 0
    snap = sched.step(10)
    assert sched.tick == 10
    assert snap.clock is not None
    assert snap.clock.world_tick == 10


def test_step_returns_actuator_states(menagerie_scene_yaml):
    sched = _make_scheduler(menagerie_scene_yaml)
    snap = sched.step(1)
    # Menagerie SO-ARM100 has actuators; snapshot should have states.
    assert len(snap.actuators) > 0
    for a in snap.actuators:
        assert a.ref is not None
        assert a.ref.actuator_id != ""


def test_reset_zeroes_tick(menagerie_scene_yaml):
    sched = _make_scheduler(menagerie_scene_yaml)
    sched.step(50)
    assert sched.tick == 50
    snap = sched.reset()
    assert sched.tick == 0
    assert snap.clock is not None
    assert snap.clock.world_tick == 0


def test_step_is_deterministic(menagerie_scene_yaml):
    """Two fresh schedulers stepping the same number of ticks
    should produce identical position_value arrays."""
    s1 = _make_scheduler(menagerie_scene_yaml)
    s2 = _make_scheduler(menagerie_scene_yaml)
    snap1 = s1.step(20)
    snap2 = s2.step(20)
    for a1, a2 in zip(snap1.actuators, snap2.actuators):
        assert abs(a1.position_value - a2.position_value) < 1e-12, (
            f"{a1.ref.actuator_id}: {a1.position_value} != {a2.position_value}"
        )


def test_reset_restores_initial_state(menagerie_scene_yaml):
    sched = _make_scheduler(menagerie_scene_yaml)
    snap_initial = sched.step(1)
    sched.reset()
    # After reset + 1 step, state should match the original first step.
    snap_after = sched.step(1)
    for a1, a2 in zip(snap_initial.actuators, snap_after.actuators):
        assert abs(a1.position_value - a2.position_value) < 1e-12


def test_run_forever_raises(menagerie_scene_yaml):
    sched = _make_scheduler(menagerie_scene_yaml)
    with pytest.raises(NotImplementedError):
        sched.run_forever()
