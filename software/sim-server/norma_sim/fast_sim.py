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

from .world.capabilities import command_value_to_ctrl, qpos_to_position_value
from .world.model import MuJoCoWorld


class FastSim:
    """In-process MuJoCo sim — no subprocess, no IPC."""

    def __init__(
        self,
        manifest_path: str | Path,
        cameras: dict[str, tuple[int, int]] | None = None,
        physics_hz: int = 500,
        action_hz: int = 30,
    ) -> None:
        self.world = MuJoCoWorld.from_manifest_path(manifest_path)
        self.model = self.world.model
        self.data = self.world.data
        self.substeps = physics_hz // action_hz

        # ── Actuator mapping (same heuristic as gym_env.py) ──
        self._joint_indices: list[int] = []   # MuJoCo actuator indices
        self._gripper_indices: list[int] = []
        self._joint_manifests: list = []
        self._gripper_manifests: list = []

        for robot in self.world.manifest.robots:
            for act in robot.actuators:
                mj_idx = self.world.actuator_id_for(act.mjcf_actuator)
                is_gripper = (
                    act.capability.kind == "GRIPPER_PARALLEL"
                    or "gripper" in act.actuator_id.lower()
                )
                if is_gripper:
                    self._gripper_indices.append(mj_idx)
                    self._gripper_manifests.append(act)
                else:
                    self._joint_indices.append(mj_idx)
                    self._joint_manifests.append(act)

        # ── Joint qpos addresses for reading state ──
        self._joint_qposadr: list[int] = []
        self._gripper_qposadr: list[int] = []
        for robot in self.world.manifest.robots:
            for act in robot.actuators:
                adr = self.world.joint_qposadr_for(act.mjcf_joint)
                is_gripper = (
                    act.capability.kind == "GRIPPER_PARALLEL"
                    or "gripper" in act.actuator_id.lower()
                )
                if is_gripper:
                    self._gripper_qposadr.append(adr)
                else:
                    self._joint_qposadr.append(adr)

        # ── Camera renderers ──
        self._cameras = cameras or {}
        self._renderers: dict[str, mujoco.Renderer] = {}
        self._cam_configs: dict[str, tuple[int, int]] = {}
        for cam_name, (h, w) in self._cameras.items():
            self._renderers[cam_name] = mujoco.Renderer(self.model, height=h, width=w)
            self._cam_configs[cam_name] = (h, w)

        # Camera pose defaults (same as stepping.py DEFAULT_CAMERAS)
        self._camera_poses: dict[str, dict] = {
            "top": dict(lookat=(0.0, 0.05, 0.1), distance=0.6, azimuth=90.0, elevation=-60.0),
            "wrist.top": dict(lookat=(0.0, 0.05, 0.15), distance=0.4, azimuth=180.0, elevation=-45.0),
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

        # Set gripper control
        for i, mj_idx in enumerate(self._gripper_indices):
            act = self._gripper_manifests[i]
            self.data.ctrl[mj_idx] = command_value_to_ctrl(
                float(gripper_normalized), act
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

        # Gripper position (normalized 0-1)
        if self._gripper_qposadr:
            raw = float(self.data.qpos[self._gripper_qposadr[0]])
            act = self._gripper_manifests[0]
            obs["gripper"] = np.array(
                [qpos_to_position_value(raw, act)], dtype=np.float64
            )
        else:
            obs["gripper"] = np.array([0.0], dtype=np.float64)

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
