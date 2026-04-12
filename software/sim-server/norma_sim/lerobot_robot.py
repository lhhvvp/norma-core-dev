"""LeRobot Robot adapter for NormaSimEnv.

Implements LeRobot's ``Robot`` base class so that LeRobot's record,
train, and eval scripts can use NormaCore's MuJoCo simulation with
TheRobotStudio motor parameters directly.

Usage with LeRobot scripts::

    from norma_sim.lerobot_robot import NormaSimRobot, NormaSimRobotConfig

    config = NormaSimRobotConfig(
        manifest_path="path/to/scene.yaml",
        render_port=8012,  # optional mjviser
    )
    robot = NormaSimRobot(config)
    robot.connect()
    obs = robot.get_observation()
    sent = robot.send_action({"shoulder_pan.pos": 0.5, ...})
    robot.disconnect()

Or via LeRobot's factory::

    robot = make_robot_from_config(config)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Any

import numpy as np

from .gym_env import NormaSimEnv


@dataclass
class NormaSimRobotConfig:
    """Configuration for NormaSimRobot."""

    manifest_path: str = ""
    physics_hz: int = 500
    action_hz: int = 30
    render_port: int = 0


class NormaSimRobot:
    """LeRobot-compatible Robot wrapping NormaSimEnv.

    Bridges NormaCore's Gymnasium env to LeRobot's Robot protocol:
    - ``get_observation()`` → flat dict ``{"shoulder_pan.pos": float, ...}``
    - ``send_action(action)`` → calls ``env.step()``, caches new obs
    - Feature names match LeRobot's SO-Follower convention for policy compatibility
    """

    name = "norma_sim"

    # Motor names matching LeRobot SO-Follower convention
    JOINT_NAMES = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    ]
    GRIPPER_NAME = "gripper"

    def __init__(self, config: NormaSimRobotConfig) -> None:
        self.config = config
        self._env: NormaSimEnv | None = None
        self._current_obs: dict[str, Any] = {}
        self._connected = False

    @cached_property
    def observation_features(self) -> dict:
        """Flat dict of observation feature types (LeRobot contract)."""
        features: dict[str, type] = {}
        for name in self.JOINT_NAMES:
            features[f"{name}.pos"] = float
        features[f"{self.GRIPPER_NAME}.pos"] = float
        return features

    @cached_property
    def action_features(self) -> dict:
        """Flat dict of action feature types (LeRobot contract)."""
        features: dict[str, type] = {}
        for name in self.JOINT_NAMES:
            features[f"{name}.pos"] = float
        features[f"{self.GRIPPER_NAME}.pos"] = float
        return features

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True  # sim doesn't need calibration

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            return
        self._env = NormaSimEnv(
            manifest_path=self.config.manifest_path,
            physics_hz=self.config.physics_hz,
            action_hz=self.config.action_hz,
            render_port=self.config.render_port,
        )
        obs, info = self._env.reset()
        self._cache_obs(obs)
        self._connected = True

    def calibrate(self) -> None:
        pass  # sim doesn't need calibration

    def configure(self) -> None:
        pass  # sim doesn't need motor configuration

    def get_observation(self) -> dict[str, Any]:
        """Return current observation as flat dict (LeRobot contract)."""
        return dict(self._current_obs)  # return copy

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send action to sim, step physics, cache new obs.

        Args:
            action: flat dict ``{"shoulder_pan.pos": float, ...}``

        Returns:
            The action actually applied (same as input for sim).
        """
        assert self._env is not None, "Robot not connected"
        gym_action = self._action_to_gym(action)
        obs, reward, terminated, truncated, info = self._env.step(gym_action)
        self._cache_obs(obs)
        return action

    def disconnect(self) -> None:
        if self._env is not None:
            self._env.close()
            self._env = None
        self._connected = False

    # Context manager support
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # ── Internal conversion ──

    def _cache_obs(self, gym_obs: dict[str, Any]) -> None:
        """Convert NormaSimEnv obs dict → flat LeRobot obs dict."""
        self._current_obs = {}
        joints = gym_obs.get("joints", np.array([]))
        gripper = gym_obs.get("gripper", np.array([]))

        for i, name in enumerate(self.JOINT_NAMES):
            if i < len(joints):
                self._current_obs[f"{name}.pos"] = float(joints[i])

        if len(gripper) > 0:
            # LeRobot SO-Follower uses 0-100 scale for gripper
            # NormaSimEnv uses 0-1 normalized → convert to 0-100
            self._current_obs[f"{self.GRIPPER_NAME}.pos"] = float(gripper[0]) * 100.0

    def _action_to_gym(self, action: dict[str, Any]) -> dict[str, Any]:
        """Convert flat LeRobot action dict → NormaSimEnv action dict."""
        joints = np.array([
            action.get(f"{name}.pos", 0.0)
            for name in self.JOINT_NAMES
        ], dtype=np.float64)

        gripper_val = action.get(f"{self.GRIPPER_NAME}.pos", 50.0)
        # LeRobot uses 0-100 scale → NormaSimEnv expects 0-1
        gripper = np.array([gripper_val / 100.0], dtype=np.float64)

        return {"joints": joints, "gripper": gripper}
