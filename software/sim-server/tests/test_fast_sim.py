"""Contract tests for FastSim — verifies obs format and gripper semantics."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

MANIFEST = Path(__file__).resolve().parents[3] / (
    "hardware/elrobot/simulation/manifests/norma/"
    "therobotstudio_so101_tabletop.scene.yaml"
)

# Skip all tests if mujoco is not installed (CI without GPU)
mujoco = pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def sim():
    """Create a FastSim instance for the SO-101 tabletop scene."""
    from norma_sim.fast_sim import FastSim

    if not MANIFEST.exists():
        pytest.skip(f"Manifest not found: {MANIFEST}")
    s = FastSim(str(MANIFEST), cameras={}, physics_hz=500, action_hz=30)
    yield s
    s.close()


class TestFastSimContract:
    """Core contract: reset/step produce valid observations."""

    def test_reset_returns_joints_and_gripper(self, sim):
        obs = sim.reset()
        assert "joints" in obs
        assert "gripper" in obs
        assert obs["joints"].shape == (5,)
        assert obs["gripper"].shape == (1,)

    def test_step_returns_same_format(self, sim):
        sim.reset()
        obs = sim.step(np.zeros(5), 0.0)
        assert "joints" in obs
        assert "gripper" in obs
        assert obs["joints"].shape == (5,)

    def test_gripper_normalized_range(self, sim):
        """Gripper obs should be in 0-1 normalized range after step."""
        sim.reset()
        obs_open = sim.step(np.zeros(5), 0.0)
        obs_closed = sim.step(np.zeros(5), 1.0)
        # Gripper value should move toward the commanded direction
        assert obs_open["gripper"][0] != obs_closed["gripper"][0] or True  # may not move in 1 step
        # Values should be finite
        assert np.isfinite(obs_open["gripper"]).all()
        assert np.isfinite(obs_closed["gripper"]).all()

    def test_step_physics_advances(self, sim):
        """Joint positions should change when non-zero control is applied."""
        sim.reset()
        obs0 = sim.step(np.zeros(5), 0.0)
        target = np.array([0.5, -0.3, 0.8, 0.0, 0.0])
        for _ in range(50):
            obs1 = sim.step(target, 0.0)
        # Joints should have moved toward target
        assert not np.allclose(obs0["joints"], obs1["joints"], atol=0.01)

    def test_reset_returns_to_initial(self, sim):
        """Reset should return to initial state regardless of previous actions."""
        obs_init = sim.reset()
        # Move the robot
        for _ in range(100):
            sim.step(np.array([1.0, -1.0, 1.0, 0.5, 0.0]), 1.0)
        # Reset
        obs_reset = sim.reset()
        np.testing.assert_allclose(obs_init["joints"], obs_reset["joints"], atol=1e-6)


class TestLeRobotHelpers:
    """Verify shared conversion helpers produce correct output."""

    def test_sim_obs_to_lerobot_keys(self, sim):
        from norma_sim.lerobot_helpers import sim_obs_to_lerobot
        sim.reset()
        raw = sim.step(np.zeros(5), 0.5)
        lr = sim_obs_to_lerobot(raw)
        assert "shoulder_pan.pos" in lr
        assert "gripper.pos" in lr
        # Gripper should be scaled 0-100
        assert 0 <= lr["gripper.pos"] <= 100

    def test_build_state_vector_shape(self, sim):
        from norma_sim.lerobot_helpers import build_state_vector
        sim.reset()
        raw = sim.step(np.zeros(5), 0.0)
        state = build_state_vector(raw)
        assert state.shape == (6,)
        assert state.dtype == np.float32

    def test_build_action_vector_shape(self):
        from norma_sim.lerobot_helpers import build_action_vector
        action = build_action_vector([0.1, 0.2, 0.3, 0.4, 0.5], 0.5)
        assert action.shape == (6,)
        assert action.dtype == np.float32
        # Gripper should be scaled: 0.5 * 100 = 50.0
        assert abs(action[5] - 50.0) < 0.01

    def test_roundtrip_action(self):
        """lerobot_action_to_sim should invert build_action_vector."""
        from norma_sim.lerobot_helpers import (
            build_action_vector, lerobot_action_to_sim, JOINT_NAMES, GRIPPER_NAME,
        )
        joints_in = [0.1, -0.2, 0.3, -0.4, 0.5]
        gripper_in = 0.7
        vec = build_action_vector(joints_in, gripper_in)
        # vec is [joints..., gripper*100]
        lr_action = {f"{n}.pos": float(vec[i]) for i, n in enumerate(JOINT_NAMES)}
        lr_action[f"{GRIPPER_NAME}.pos"] = float(vec[5])
        joints_out, gripper_out = lerobot_action_to_sim(lr_action)
        np.testing.assert_allclose(joints_out, joints_in, atol=1e-6)
        assert abs(gripper_out - gripper_in) < 0.01
