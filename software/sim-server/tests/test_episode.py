"""Tests for episode format and recorder."""
import numpy as np
import pytest

try:
    from norma_sim.episode import Episode, EpisodeBuilder
    _OK = True
    _ERR = ""
except Exception as e:
    _OK = False
    _ERR = str(e)

pytestmark = pytest.mark.skipif(not _OK, reason=f"episode imports failed: {_ERR}")


def test_episode_builder_and_build():
    builder = EpisodeBuilder(metadata={"test": True})
    for i in range(5):
        builder.add_step(
            obs={"joints": np.array([0.1 * i, 0.2 * i]), "gripper": np.array([0.5])},
            action={"joints": np.array([0.1, 0.2]), "gripper": np.array([1.0])},
            info={"sim_time_ns": i * 1_000_000},
        )
    ep = builder.build()
    assert ep.n_steps == 5
    assert ep.joints_rad.shape == (5, 2)
    assert ep.gripper_normalized.shape == (5, 1)
    assert ep.action_joints_rad.shape == (5, 2)
    assert ep.metadata["test"] is True


def test_episode_save_load_roundtrip(tmp_path):
    ep = Episode(
        timestamp_ns=np.arange(10, dtype=np.int64),
        joints_rad=np.random.randn(10, 5),
        gripper_normalized=np.random.rand(10, 1),
        action_joints_rad=np.random.randn(10, 5),
        action_gripper=np.random.rand(10, 1),
        metadata={"physics_hz": 500, "seed": 42},
    )
    path = ep.save(tmp_path / "test_ep.npz")
    loaded = Episode.load(path)
    assert loaded.n_steps == 10
    np.testing.assert_array_equal(loaded.timestamp_ns, ep.timestamp_ns)
    np.testing.assert_allclose(loaded.joints_rad, ep.joints_rad)
    np.testing.assert_allclose(loaded.gripper_normalized, ep.gripper_normalized)
    assert loaded.metadata["physics_hz"] == 500
    assert loaded.metadata["seed"] == 42


def test_episode_builder_clear():
    builder = EpisodeBuilder()
    builder.add_step(
        obs={"joints": np.zeros(3)},
        action={"joints": np.zeros(3)},
        info={},
    )
    assert len(builder._timestamps) == 1
    builder.clear()
    assert len(builder._timestamps) == 0


def test_episode_empty_build():
    builder = EpisodeBuilder()
    ep = builder.build()
    assert ep.n_steps == 0
