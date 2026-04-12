"""`MuJoCoWorld` — the sim server's owning handle to the MjModel/MjData
pair plus manifest metadata needed to resolve capability lookups.

The class is deliberately thin: it loads the MJCF, indexes
actuators/joints, and provides a threading.Lock so the scheduler and
IPC threads can coordinate. Higher-level behaviour (actuation
application, snapshot construction) lives in sibling modules that take
a `MuJoCoWorld` by reference.
"""
from __future__ import annotations

import threading
from functools import cached_property
from typing import Optional

import mujoco

from .manifest import ActuatorManifest, WorldManifest


class MuJoCoWorld:
    """MjModel/MjData container with manifest-driven lookups."""

    def __init__(self, manifest: WorldManifest) -> None:
        self.manifest = manifest
        self.model = mujoco.MjModel.from_xml_path(str(manifest.mjcf_path))
        self.data = mujoco.MjData(self.model)
        self.lock = threading.Lock()
        self._actuator_id_cache: dict[str, int] = {}
        self._joint_qposadr_cache: dict[str, int] = {}
        self._build_lookups()

    @classmethod
    def from_manifest_path(cls, manifest_path) -> "MuJoCoWorld":
        """Canonical constructor: load the scene yaml, open the MJCF it
        references, build manifest + MuJoCo model in one call."""
        from .manifest import load_manifest

        manifest = load_manifest(manifest_path)
        return cls(manifest)

    def _build_lookups(self) -> None:
        for robot in self.manifest.robots:
            for act in robot.actuators:
                idx = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act.mjcf_actuator
                )
                if idx < 0:
                    raise ValueError(
                        f"MJCF has no <position name='{act.mjcf_actuator}'> "
                        f"for manifest actuator '{act.actuator_id}'"
                    )
                self._actuator_id_cache[act.mjcf_actuator] = idx
                joint_idx = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, act.mjcf_joint
                )
                if joint_idx < 0:
                    raise ValueError(
                        f"MJCF has no joint '{act.mjcf_joint}' for "
                        f"manifest actuator '{act.actuator_id}'"
                    )
                self._joint_qposadr_cache[act.mjcf_joint] = int(
                    self.model.jnt_qposadr[joint_idx]
                )

    def actuator_id_for(self, mjcf_actuator: str) -> Optional[int]:
        return self._actuator_id_cache.get(mjcf_actuator)

    def joint_qposadr_for(self, mjcf_joint: str) -> Optional[int]:
        return self._joint_qposadr_cache.get(mjcf_joint)

    def actuator_by_mjcf_name(self, mjcf_actuator: str) -> Optional[ActuatorManifest]:
        for robot in self.manifest.robots:
            for act in robot.actuators:
                if act.mjcf_actuator == mjcf_actuator:
                    return act
        return None

    # ── Actuator classification (single source of truth) ──

    @staticmethod
    def _is_gripper(act: ActuatorManifest) -> bool:
        """Classify an actuator as gripper based on capability or name."""
        return (
            act.capability.kind == "GRIPPER_PARALLEL"
            or "gripper" in act.actuator_id.lower()
        )

    @cached_property
    def joint_actuators(self) -> tuple[ActuatorManifest, ...]:
        """All non-gripper actuators across all robots."""
        return tuple(
            a for r in self.manifest.robots for a in r.actuators
            if not self._is_gripper(a)
        )

    @cached_property
    def gripper_actuators(self) -> tuple[ActuatorManifest, ...]:
        """All gripper actuators across all robots."""
        return tuple(
            a for r in self.manifest.robots for a in r.actuators
            if self._is_gripper(a)
        )

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def reset(self) -> None:
        """Reset simulation state: qpos/qvel/ctrl/act to defaults, then forward."""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
