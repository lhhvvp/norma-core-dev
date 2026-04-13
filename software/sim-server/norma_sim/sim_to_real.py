"""Sim-to-real adapter — degrades sim observations to train robust policies.

Wraps any sim backend (FastSim, NormaSimEnv) and injects realistic
imperfections: sensor noise, action delay, calibration offset, camera
latency. Forces the policy to learn behaviors that transfer to real
hardware.

The adapter implements the same interface as FastSim (RobotEnv Protocol),
so it's transparent to NormaSimRobot.

Usage::

    from norma_sim.sim_to_real import SimToRealAdapter, SimToRealConfig

    adapter = SimToRealAdapter(
        backend=FastSim("scene.yaml", cameras={"top": (224, 224)}),
        config=SimToRealConfig(joint_noise_std=0.02, action_delay_steps=1),
    )
    obs = adapter.reset()
    obs = adapter.step(joints, gripper)  # obs has noise, delay, offset

Training strategy:
    Phase 1: adapter OFF  → validate pipeline, measure sim performance
    Phase 2: adapter ON   → train robust policy for real deployment
    Phase 3: real hardware → policy already handles noise, no adapter needed
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class SimToRealConfig:
    """Sim-to-real gap parameters. Tune per robot platform.

    Start conservative (small values), increase based on real-robot eval.
    """

    # ── Observation noise ──
    joint_noise_std: float = 0.02       # rad — encoder quantization + noise
    gripper_noise_std: float = 0.03     # normalized — force sensor noise

    # ── Action imperfections ──
    action_delay_steps: int = 1         # frames of control latency (USB + loop)
    action_noise_std: float = 0.01      # rad — motor imprecision

    # ── Calibration ──
    calibration_offset_std: float = 0.05  # rad — per-episode homing variance

    # ── Camera ──
    camera_latency_frames: int = 1      # USB capture lag
    image_noise_std: float = 5.0        # pixel noise (uint8 scale)
    brightness_jitter: float = 0.1      # ±10% brightness variation

    # ── Dropout ──
    obs_drop_prob: float = 0.01         # 1% stale observation

    @classmethod
    def off(cls) -> "SimToRealConfig":
        """All noise disabled — pure sim."""
        return cls(
            joint_noise_std=0, gripper_noise_std=0,
            action_delay_steps=0, action_noise_std=0,
            calibration_offset_std=0,
            camera_latency_frames=0, image_noise_std=0,
            brightness_jitter=0, obs_drop_prob=0,
        )

    @classmethod
    def mild(cls) -> "SimToRealConfig":
        """Conservative noise — good starting point."""
        return cls()  # defaults are already conservative

    @classmethod
    def aggressive(cls) -> "SimToRealConfig":
        """Strong noise — for maximum robustness."""
        return cls(
            joint_noise_std=0.05, gripper_noise_std=0.08,
            action_delay_steps=2, action_noise_std=0.03,
            calibration_offset_std=0.1,
            camera_latency_frames=2, image_noise_std=10.0,
            brightness_jitter=0.2, obs_drop_prob=0.03,
        )


class SimToRealAdapter:
    """Transparent degradation layer between sim backend and policy.

    Implements RobotEnv Protocol (reset/step/close) so it can be
    used as a drop-in replacement for any sim backend.
    """

    def __init__(
        self,
        backend: Any,
        config: SimToRealConfig | None = None,
        seed: int = 0,
    ) -> None:
        self.backend = backend
        self.config = config or SimToRealConfig.mild()
        self.rng = np.random.default_rng(seed)

        self._action_buffer: deque = deque(maxlen=self.config.action_delay_steps + 1)
        self._camera_buffers: dict[str, deque] = {}
        self._calibration_offsets: np.ndarray | None = None
        self._prev_obs: dict[str, Any] | None = None

    def reset(self) -> dict[str, Any]:
        """Reset backend and initialize per-episode randomization."""
        obs = self.backend.reset()

        # Fresh calibration offset for this episode
        n_joints = len(obs.get("joints", []))
        self._calibration_offsets = self.rng.normal(
            0, self.config.calibration_offset_std, size=n_joints
        )

        self._action_buffer.clear()
        self._camera_buffers.clear()
        self._prev_obs = None

        degraded = self._degrade(obs)
        self._prev_obs = obs
        return degraded

    def step(
        self,
        joint_positions: np.ndarray,
        gripper_normalized: float,
    ) -> dict[str, Any]:
        """Step with action delay and noise injection."""
        # Buffer current action
        self._action_buffer.append((
            np.array(joint_positions, dtype=np.float64),
            float(gripper_normalized),
        ))

        # Execute delayed action
        if self.config.action_delay_steps > 0 and len(self._action_buffer) <= self.config.action_delay_steps:
            # Not enough buffered — hold position (zero velocity)
            delayed_j, delayed_g = self._action_buffer[0]
        else:
            delayed_j, delayed_g = self._action_buffer[0]

        # Action noise
        if self.config.action_noise_std > 0:
            delayed_j = delayed_j + self.rng.normal(
                0, self.config.action_noise_std, size=len(delayed_j)
            )

        obs = self.backend.step(delayed_j, delayed_g)
        degraded = self._degrade(obs)
        self._prev_obs = obs
        return degraded

    def _degrade(self, obs: dict[str, Any]) -> dict[str, Any]:
        """Apply all degradation to one observation."""
        out: dict[str, Any] = {}

        # ── Joint noise + calibration offset ──
        if "joints" in obs:
            joints = np.array(obs["joints"], dtype=np.float64)
            if self._calibration_offsets is not None:
                joints = joints + self._calibration_offsets
            if self.config.joint_noise_std > 0:
                joints = joints + self.rng.normal(0, self.config.joint_noise_std, size=len(joints))
            # Observation dropout
            if self._prev_obs is not None and self.rng.random() < self.config.obs_drop_prob:
                joints = np.array(self._prev_obs["joints"], dtype=np.float64)
            out["joints"] = joints

        # ── Gripper noise ──
        if "gripper" in obs:
            gripper = np.array(obs["gripper"], dtype=np.float64)
            if self.config.gripper_noise_std > 0:
                gripper = gripper + self.rng.normal(
                    0, self.config.gripper_noise_std, size=gripper.shape
                )
            out["gripper"] = np.clip(gripper, 0.0, 1.0)

        # ── Camera degradation ──
        for key, val in obs.items():
            if key.startswith("camera.") and isinstance(val, np.ndarray):
                img = val.astype(np.float32)

                # Camera latency
                if self.config.camera_latency_frames > 0:
                    if key not in self._camera_buffers:
                        self._camera_buffers[key] = deque(
                            maxlen=self.config.camera_latency_frames + 1
                        )
                    self._camera_buffers[key].append(img.copy())
                    img = self._camera_buffers[key][0]

                # Image noise
                if self.config.image_noise_std > 0:
                    img = img + self.rng.normal(0, self.config.image_noise_std, size=img.shape)

                # Brightness jitter
                if self.config.brightness_jitter > 0:
                    brightness = 1.0 + self.rng.uniform(
                        -self.config.brightness_jitter,
                        self.config.brightness_jitter,
                    )
                    img = img * brightness

                out[key] = np.clip(img, 0, 255).astype(np.uint8)
            elif key not in out:
                out[key] = val  # pass through non-camera, non-joint keys

        return out

    def close(self) -> None:
        self.backend.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
