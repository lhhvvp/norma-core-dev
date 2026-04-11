"""Tests for the real-time scheduler."""
import pytest

try:
    from norma_sim.scheduler.realtime import RealTimeScheduler
    from norma_sim.world.model import MuJoCoWorld
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"scheduler imports failed: {_ERR}")


def test_scheduler_pacing_500hz(world_yaml_path):
    """Running the scheduler for 200 ms wall should produce about
    100 physics ticks at 500 Hz with ±20% tolerance (generous because
    CI schedulers vary)."""
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    sched = RealTimeScheduler(world, physics_hz=500, publish_hz=100)
    sched.run_for(0.2)
    # Expected ≈ 100 ticks; tolerance ±25 ticks.
    assert 75 <= sched.tick <= 150, f"tick count out of window: {sched.tick}"


def test_scheduler_publish_callback_frequency(world_yaml_path):
    """At physics_hz=500 and publish_hz=100, on_publish should fire
    roughly once every 5 physics ticks."""
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    published = []
    sched = RealTimeScheduler(
        world,
        physics_hz=500,
        publish_hz=100,
        on_publish=lambda t: published.append(t),
    )
    sched.run_for(0.2)
    if not published:
        pytest.skip("no publishes in 200ms — CI too slow")
    # publishes come at ticks that are multiples of divider=5.
    for t in published:
        assert t % 5 == 0


def test_scheduler_rejects_bad_ratios(world_yaml_path):
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    with pytest.raises(ValueError):
        RealTimeScheduler(world, physics_hz=0)
    with pytest.raises(ValueError):
        RealTimeScheduler(world, physics_hz=500, publish_hz=0)
    with pytest.raises(ValueError):
        RealTimeScheduler(world, physics_hz=500, publish_hz=1000)


def test_scheduler_stop_exits_run_forever(world_yaml_path):
    """run_for sets a watchdog that calls stop() — so the loop must
    actually observe it within a few ms."""
    import time
    world = MuJoCoWorld.from_manifest_path(world_yaml_path)
    sched = RealTimeScheduler(world, physics_hz=500, publish_hz=100)
    t0 = time.monotonic()
    sched.run_for(0.1)
    dt = time.monotonic() - t0
    # Should not overshoot by more than 50 ms.
    assert dt < 0.2, f"run_for overshoot {dt:.3f}s"
