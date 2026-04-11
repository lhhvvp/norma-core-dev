"""Walking skeleton: prove norma_sim infra works with Menagerie SO-ARM100
verbatim. Baseline for assumption A ("infra is robot-agnostic").

MUST remain green indefinitely — if this file breaks, infra has regressed
even if ElRobot still works. The Menagerie MJCF is vendored unmodified,
so any change here is a signal that the change was to norma_sim, not to
ElRobot."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

try:
    import mujoco
    from norma_sim.world.manifest import load_manifest
    from norma_sim.world.model import MuJoCoWorld
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"imports not OK: {_ERR}")


@pytest.fixture
def menagerie_walking_skeleton_yaml() -> Path:
    here = Path(__file__).resolve()
    # parents: [0]=tests/integration, [1]=tests, [2]=sim-server, [3]=software, [4]=repo
    repo_root = here.parents[4]
    p = repo_root / "hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml"
    if not p.exists():
        pytest.skip(
            f"Menagerie scene yaml not found at {p}; run Chunk 4 Task 4.1 first"
        )
    return p


def test_menagerie_scene_yaml_loads(menagerie_walking_skeleton_yaml: Path):
    """The scene yaml parses, the referenced MJCF exists, and load_manifest
    produces a non-empty actuator list."""
    manifest = load_manifest(menagerie_walking_skeleton_yaml)
    assert manifest.world_name == "menagerie_trs_so_arm100"
    assert manifest.mjcf_path.exists()
    assert len(manifest.robots) == 1
    assert len(manifest.robots[0].actuators) >= 5


def test_menagerie_mujoco_world_loads(menagerie_walking_skeleton_yaml: Path):
    """MuJoCoWorld.from_manifest_path succeeds end-to-end: load yaml,
    open MJCF, build lookups."""
    world = MuJoCoWorld.from_manifest_path(menagerie_walking_skeleton_yaml)
    assert world.model.nu >= 5
    assert world.model.nv >= 5
    # Every actuator in the manifest should have a resolved MJCF index
    for robot in world.manifest.robots:
        for act in robot.actuators:
            idx = world.actuator_id_for(act.mjcf_actuator)
            assert idx is not None, f"{act.mjcf_actuator} not cached"


def test_menagerie_no_self_collision_at_rest(menagerie_walking_skeleton_yaml: Path):
    """mj_forward at the default pose should produce zero contacts.
    Menagerie's trs_so_arm100 is hand-tuned to avoid the self-collision
    issues the MVP-1 ElRobot URDF had."""
    world = MuJoCoWorld.from_manifest_path(menagerie_walking_skeleton_yaml)
    mujoco.mj_forward(world.model, world.data)
    assert world.data.ncon == 0, (
        f"Menagerie should have clean collision at rest, got {world.data.ncon} contacts"
    )


def test_menagerie_step_advances_time(menagerie_walking_skeleton_yaml: Path):
    world = MuJoCoWorld.from_manifest_path(menagerie_walking_skeleton_yaml)
    t0 = float(world.data.time)
    for _ in range(100):
        world.step()
    t1 = float(world.data.time)
    assert t1 > t0
    # All qpos values still finite after 100 steps at rest
    assert np.isfinite(world.data.qpos).all()
    assert np.isfinite(world.data.qvel).all()


def test_menagerie_all_actuators_drivable(menagerie_walking_skeleton_yaml: Path):
    """Every actuator should accept a ctrl write and step without NaN.
    Drive each actuator to its ctrlrange midpoint for 200 steps
    (~0.4 sec sim) and verify qpos stays finite."""
    world = MuJoCoWorld.from_manifest_path(menagerie_walking_skeleton_yaml)
    ctrl_mid = (
        world.model.actuator_ctrlrange[:, 0] + world.model.actuator_ctrlrange[:, 1]
    ) / 2
    world.data.ctrl[:] = ctrl_mid
    for _ in range(200):
        world.step()
        assert np.isfinite(world.data.qpos).all(), "NaN during mid-ctrl drive"


def test_menagerie_stress_10000_random_steps_no_nan(menagerie_walking_skeleton_yaml: Path):
    """Stress test: 10000 random-ctrl steps, resampling every 100 steps.
    This is the Floor 3 analog for the Menagerie baseline."""
    world = MuJoCoWorld.from_manifest_path(menagerie_walking_skeleton_yaml)
    rng = np.random.default_rng(42)
    lo = world.model.actuator_ctrlrange[:, 0]
    hi = world.model.actuator_ctrlrange[:, 1]
    for step in range(10000):
        if step % 100 == 0:
            world.data.ctrl[:] = rng.uniform(lo, hi)
        world.step()
        if step % 1000 == 0:
            assert np.isfinite(world.data.qpos).all(), f"NaN at step {step}"
    assert np.isfinite(world.data.qpos).all()
    assert np.isfinite(world.data.qvel).all()
