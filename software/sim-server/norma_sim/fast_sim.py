"""In-process MuJoCo simulation for fast data generation.

Bypasses the subprocess/IPC/protobuf stack entirely. Reuses the
manifest loader and MuJoCoWorld from ``norma_sim.world`` but renders
cameras and returns observations as raw numpy arrays — no proto
serialization, no async, no UDS.

Typical speedup vs NormaSimEnv: 5-10x (mainly from avoiding per-frame
protobuf encode of camera blobs + subprocess IPC).

Usage::

    sim = FastSim("path/to/scene.yaml", cameras={"top": (224, 224)})
    obs = sim.reset()
    for _ in range(260):
        obs = sim.step(joint_positions, gripper_normalized)
        img = obs["camera.top"]  # (224, 224, 3) uint8
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .cameras import DEFAULT_CAMERAS
from .world.capabilities import (
    command_value_to_ctrl,
    gripper_command_to_ctrl,
    gripper_qpos_to_normalized,
)
from .world.model import MuJoCoWorld


class FastSim:
    """In-process MuJoCo sim — no subprocess, no IPC."""

    def __init__(
        self,
        manifest_path: str | Path,
        cameras: dict[str, tuple[int, int]] | None = None,
        physics_hz: int = 500,
        action_hz: int = 30,
        tracked_objects: list[str] | None = None,
    ) -> None:
        self.world = MuJoCoWorld.from_manifest_path(manifest_path)
        self.model = self.world.model
        self.data = self.world.data
        self.substeps = physics_hz // action_hz

        # ── Tracked object body IDs (for task success evaluation) ──
        # Caller passes a list of MJCF body names (e.g., ["cube"]).
        # Missing bodies are silently skipped so scenes without the
        # object still work.
        self._tracked_object_ids: dict[str, int] = {}
        for name in (tracked_objects or []):
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id >= 0:
                self._tracked_object_ids[name] = int(body_id)

        # ── Actuator mapping (from MuJoCoWorld — single source of truth) ──
        self._joint_manifests = list(self.world.joint_actuators)
        self._gripper_manifests = list(self.world.gripper_actuators)
        self._joint_indices = [self.world.actuator_id_for(a.mjcf_actuator) for a in self._joint_manifests]
        self._gripper_indices = [self.world.actuator_id_for(a.mjcf_actuator) for a in self._gripper_manifests]

        # ── Joint qpos addresses for reading state ──
        self._joint_qposadr = [self.world.joint_qposadr_for(a.mjcf_joint) for a in self._joint_manifests]
        self._gripper_qposadr = [self.world.joint_qposadr_for(a.mjcf_joint) for a in self._gripper_manifests]

        # ── Gripper ctrlrange for [0,1] ↔ ctrl mapping ──
        # Matches gym_env.py behavior: gripper normalized [0,1] maps to
        # MJCF ctrlrange [lo, hi]. This is distinct from capabilities.py
        # which only handles GRIPPER_PARALLEL metadata.
        self._gripper_ctrlrange = []
        for mj_idx in self._gripper_indices:
            lo = float(self.model.actuator_ctrlrange[mj_idx, 0])
            hi = float(self.model.actuator_ctrlrange[mj_idx, 1])
            self._gripper_ctrlrange.append((lo, hi))

        # ── Camera renderers ──
        self._cameras = cameras or {}
        self._renderers: dict[str, mujoco.Renderer] = {}
        self._cam_configs: dict[str, tuple[int, int]] = {}
        for cam_name, (h, w) in self._cameras.items():
            self._renderers[cam_name] = mujoco.Renderer(self.model, height=h, width=w)
            self._cam_configs[cam_name] = (h, w)

        # Camera pose fallbacks from shared presets
        self._camera_poses: dict[str, dict] = {
            name: dict(
                lookat=cfg.lookat, distance=cfg.distance,
                azimuth=cfg.azimuth, elevation=cfg.elevation,
            )
            for name, cfg in DEFAULT_CAMERAS.items()
        }

    def reset(self) -> dict[str, Any]:
        """Reset to initial state, return observation."""
        self.world.reset()
        return self._build_obs()

    def step(self, joint_positions: np.ndarray, gripper_normalized: float) -> dict[str, Any]:
        """Set controls, advance physics, render, return observation.

        Args:
            joint_positions: array of joint angles (rad), length = n_joints
            gripper_normalized: 0.0 (open) to 1.0 (closed)
        """
        # Set joint controls (REVOLUTE_POSITION → identity)
        for i, mj_idx in enumerate(self._joint_indices):
            if i < len(joint_positions):
                act = self._joint_manifests[i]
                self.data.ctrl[mj_idx] = command_value_to_ctrl(
                    float(joint_positions[i]), act
                )

        # Set gripper control via capabilities (single source of truth)
        for i, mj_idx in enumerate(self._gripper_indices):
            act = self._gripper_manifests[i]
            self.data.ctrl[mj_idx] = gripper_command_to_ctrl(
                float(np.clip(gripper_normalized, 0.0, 1.0)),
                act,
                self._gripper_ctrlrange[i],
            )

        # Physics substeps
        for _ in range(self.substeps):
            self.world.step()

        return self._build_obs()

    def _build_obs(self) -> dict[str, Any]:
        """Build observation dict from current state."""
        obs: dict[str, Any] = {}

        # Joint positions
        joints = np.array(
            [float(self.data.qpos[adr]) for adr in self._joint_qposadr],
            dtype=np.float64,
        )
        obs["joints"] = joints

        # Gripper position via capabilities (single source of truth)
        if self._gripper_qposadr:
            grippers = np.zeros(len(self._gripper_qposadr), dtype=np.float64)
            for i, adr in enumerate(self._gripper_qposadr):
                raw = float(self.data.qpos[adr])
                act = self._gripper_manifests[i]
                grippers[i] = gripper_qpos_to_normalized(
                    raw, act, self._gripper_ctrlrange[i],
                )
            obs["gripper"] = grippers
        else:
            obs["gripper"] = np.array([0.0], dtype=np.float64)

        # Tracked object poses (for task success evaluation)
        for name, body_id in self._tracked_object_ids.items():
            obs[f"object.{name}.pos"] = np.array(
                self.data.xpos[body_id], dtype=np.float64
            ).copy()
            obs[f"object.{name}.quat"] = np.array(
                self.data.xquat[body_id], dtype=np.float64
            ).copy()

        # Cameras
        for cam_name, renderer in self._renderers.items():
            mjcf_cam_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name
            )
            if mjcf_cam_id >= 0:
                renderer.update_scene(self.data, camera=cam_name)
            else:
                pose = self._camera_poses.get(cam_name, {})
                cam = mujoco.MjvCamera()
                cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                cam.lookat[:] = pose.get("lookat", (0, 0, 0.1))
                cam.distance = pose.get("distance", 0.8)
                cam.azimuth = pose.get("azimuth", 135.0)
                cam.elevation = pose.get("elevation", -30.0)
                renderer.update_scene(self.data, camera=cam)
            obs[f"camera.{cam_name}"] = renderer.render().copy()

        return obs

    def close(self) -> None:
        """Release renderer resources."""
        for r in self._renderers.values():
            try:
                r.close()
            except Exception:
                pass
        self._renderers.clear()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
