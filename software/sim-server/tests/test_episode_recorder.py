"""Tests for the EpisodeRecorder Gymnasium wrapper."""
import numpy as np
import pytest

try:
    from norma_sim.episode import Episode
    from norma_sim.episode_recorder import EpisodeRecorder
    from norma_sim.gym_env import NormaSimEnv
    _OK = True
    _ERR = ""
except Exception as e:
    _OK = False
    _ERR = str(e)

pytestmark = pytest.mark.skipif(not _OK, reason=f"recorder imports failed: {_ERR}")


def _get_manifest():
    from pathlib import Path
    repo = Path(__file__).resolve().parents[3]
    candidates = [
        repo / "hardware/elrobot/simulation/manifests/norma/therobotstudio_so101.scene.yaml",
        repo / "hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml",
    ]
    for c in candidates:
        if c.exists():
            return c
    pytest.skip("No scene.yaml found")


def test_recorder_saves_episode(tmp_path):
    manifest = _get_manifest()
    env = NormaSimEnv(manifest_path=manifest, physics_hz=500, action_hz=30)
    env = EpisodeRecorder(env, save_dir=tmp_path)

    obs, info = env.reset()
    for _ in range(10):
        action = env.action_space.sample()
        obs, *_ = env.step(action)
    env.close()

    # Should have saved one episode
    files = list(tmp_path.glob("ep_*.npz"))
    assert len(files) == 1

    ep = Episode.load(files[0])
    assert ep.n_steps == 10
    assert ep.joints_rad.shape[1] > 0  # has joint data
    assert "physics_hz" in ep.metadata


def test_recorder_multiple_episodes(tmp_path):
    manifest = _get_manifest()
    env = NormaSimEnv(manifest_path=manifest, physics_hz=500, action_hz=30)
    env = EpisodeRecorder(env, save_dir=tmp_path)

    # Episode 1
    env.reset()
    for _ in range(5):
        env.step(env.action_space.sample())

    # Episode 2 (reset saves episode 1)
    env.reset()
    for _ in range(3):
        env.step(env.action_space.sample())

    env.close()  # saves episode 2

    files = sorted(tmp_path.glob("ep_*.npz"))
    assert len(files) == 2

    ep0 = Episode.load(files[0])
    ep1 = Episode.load(files[1])
    assert ep0.n_steps == 5
    assert ep1.n_steps == 3
    assert ep0.metadata["episode_index"] == 0
    assert ep1.metadata["episode_index"] == 1
