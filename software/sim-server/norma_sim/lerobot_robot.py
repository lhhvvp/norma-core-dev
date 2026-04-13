"""LeRobot Robot adapter — unified interface for sim and real.

Implements LeRobot's ``Robot`` protocol with two backends:
  - ``backend="fast"`` — in-process MuJoCo via FastSim (fast, for data gen + eval)
  - ``backend="ipc"``  — subprocess IPC via NormaSimEnv (for real-time / mjviser)

Both backends produce identical LeRobot-format observations. Callers
never need to know which backend is running.

Usage::

    config = NormaSimRobotConfig(
        manifest_path="path/to/scene.yaml",
        backend="fast",          # or "ipc"
        cameras=["top"],
        camera_size=224,
    )
    robot = NormaSimRobot(config)
    robot.connect()
    obs = robot.get_observation()   # {"shoulder_pan.pos": ..., "observation.images.top": ...}
    robot.send_action({"shoulder_pan.pos": 0.5, "gripper.pos": 50.0})
    robot.disconnect()
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property
from typing import Any

import numpy as np


@dataclass
class NormaSimRobotConfig:
    """Configuration for NormaSimRobot."""

    manifest_path: str = ""
    physics_hz: int = 500
    action_hz: int = 30
    render_port: int = 0
    cameras: list[str] = field(default_factory=list)
    camera_size: int = 0  # 0 = default (480 for ipc, 224 for fast)
    backend: str = "fast"  # "fast" (in-process) or "ipc" (subprocess)


class NormaSimRobot:
    """LeRobot-compatible Robot with pluggable sim backend.

    The single entry point for all robot interaction — data generation,
    policy evaluation, and real-time control all go through this class.
    """

    name = "norma_sim"

    from .lerobot_helpers import JOINT_NAMES, GRIPPER_NAME

    def __init__(self, config: NormaSimRobotConfig) -> None:
        self.config = config
        self._backend: Any = None
        self._current_obs: dict[str, Any] = {}
        self._connected = False
        self._backend_type = config.backend

    @cached_property
    def observation_features(self) -> dict:
        features: dict = {}
        for name in self.JOINT_NAMES:
            features[f"{name}.pos"] = float
        features[f"{self.GRIPPER_NAME}.pos"] = float
        cam_h = self.config.camera_size or (224 if self._backend_type == "fast" else 480)
        cam_w = cam_h  # square for fast, could be different for ipc
        for cam_name in self.config.cameras:
            features[f"observation.images.{cam_name}"] = (cam_h, cam_w, 3)
        return features

    @cached_property
    def action_features(self) -> dict:
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
        return True

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            return

        if self._backend_type == "fast":
            self._connect_fast()
        elif self._backend_type == "ipc":
            self._connect_ipc()
        else:
            raise ValueError(f"Unknown backend: {self._backend_type!r}")

        self._connected = True

    def _connect_fast(self) -> None:
        """Connect using in-process FastSim (no subprocess)."""
        from .fast_sim import FastSim

        cam_size = self.config.camera_size or 224
        cameras = {name: (cam_size, cam_size) for name in self.config.cameras}

        self._backend = FastSim(
            manifest_path=self.config.manifest_path,
            cameras=cameras,
            physics_hz=self.config.physics_hz,
            action_hz=self.config.action_hz,
        )
        obs = self._backend.reset()
        self._cache_obs(obs)

    def _connect_ipc(self) -> None:
        """Connect using subprocess IPC (for real-time / mjviser)."""
        from .gym_env import NormaSimEnv

        self._backend = NormaSimEnv(
            manifest_path=self.config.manifest_path,
            physics_hz=self.config.physics_hz,
            action_hz=self.config.action_hz,
            render_port=self.config.render_port,
            cameras=self.config.cameras if self.config.cameras else None,
        )
        obs, info = self._backend.reset()
        self._cache_obs(obs)

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def reset(self, seed: int | None = None) -> dict[str, Any]:
        """Reset simulation, return observation in LeRobot format."""
        assert self._backend is not None, "Robot not connected"

        if self._backend_type == "fast":
            obs = self._backend.reset()
        else:
            obs, info = self._backend.reset(seed=seed)

        self._cache_obs(obs)
        return dict(self._current_obs)

    def get_observation(self) -> dict[str, Any]:
        """Return current observation as LeRobot flat dict."""
        return dict(self._current_obs)

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Send LeRobot-format action, step physics, cache new obs."""
        assert self._backend is not None, "Robot not connected"
        from .lerobot_helpers import lerobot_action_to_sim

        joints, gripper = lerobot_action_to_sim(action)

        if self._backend_type == "fast":
            obs = self._backend.step(joints, gripper)
        else:
            gym_action = {"joints": joints, "gripper": np.array([gripper], dtype=np.float64)}
            obs, reward, terminated, truncated, info = self._backend.step(gym_action)

        self._cache_obs(obs)
        return action

    def disconnect(self) -> None:
        if self._backend is not None:
            self._backend.close()
            self._backend = None
        self._connected = False

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # ── Internal ──

    def _cache_obs(self, sim_obs: dict[str, Any]) -> None:
        """Convert any backend's obs → LeRobot flat dict."""
        from .lerobot_helpers import sim_obs_to_lerobot
        self._current_obs = sim_obs_to_lerobot(sim_obs)
