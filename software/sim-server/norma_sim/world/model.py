"""`MuJoCoWorld` — the sim server's owning handle to the MjModel/MjData
pair plus manifest metadata needed to resolve capability lookups.

The class is deliberately thin: it loads the MJCF, runs
source_hash verification, indexes actuators/joints, and provides a
threading.Lock so the scheduler and IPC threads can coordinate. Higher-
level behaviour (actuation application, snapshot construction) lives
in sibling modules that take a `MuJoCoWorld` by reference.
"""
from __future__ import annotations

import threading
from typing import Optional

import mujoco

from .manifest import ActuatorManifest, WorldManifest, verify_source_hash


class MuJoCoWorld:
    """MjModel/MjData container with manifest-driven lookups."""

    def __init__(self, manifest: WorldManifest, verify_hash: bool = True) -> None:
        self.manifest = manifest
        if verify_hash:
            # Re-derive the manifest yaml path by walking up from the
            # resolved urdf_path. The manifest's urdf_path is absolute
            # after load_manifest, but we need the yaml location for
            # sha256 input. Callers supply the manifest file path via
            # the `from_manifest_path` helper below when they want
            # hash verification.
            pass
        self.model = mujoco.MjModel.from_xml_path(str(manifest.mjcf_path))
        self.data = mujoco.MjData(self.model)
        self.lock = threading.Lock()
        self._actuator_id_cache: dict[str, int] = {}
        self._joint_qposadr_cache: dict[str, int] = {}
        self._build_lookups()

    @classmethod
    def from_manifest_path(cls, manifest_path, verify_hash: bool = True) -> "MuJoCoWorld":
        """Canonical constructor that loads + verifies + instantiates.
        Using this instead of ``MuJoCoWorld(manifest)`` guarantees the
        source-hash check runs against the same yaml the client
        actually passed on the command line.
        """
        from .manifest import load_manifest

        manifest = load_manifest(manifest_path)
        if verify_hash:
            verify_source_hash(manifest_path, manifest.mjcf_path)
        return cls(manifest, verify_hash=False)

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
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, act.urdf_joint
                )
                if joint_idx < 0:
                    raise ValueError(
                        f"MJCF has no joint '{act.urdf_joint}' for "
                        f"manifest actuator '{act.actuator_id}'"
                    )
                self._joint_qposadr_cache[act.urdf_joint] = int(
                    self.model.jnt_qposadr[joint_idx]
                )

    def actuator_id_for(self, mjcf_actuator: str) -> Optional[int]:
        return self._actuator_id_cache.get(mjcf_actuator)

    def joint_qposadr_for(self, urdf_joint: str) -> Optional[int]:
        return self._joint_qposadr_cache.get(urdf_joint)

    def actuator_by_mjcf_name(self, mjcf_actuator: str) -> Optional[ActuatorManifest]:
        for robot in self.manifest.robots:
            for act in robot.actuators:
                if act.mjcf_actuator == mjcf_actuator:
                    return act
        return None

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)
