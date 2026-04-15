"""Shared LeRobot observation/action conversion helpers.

Single source of truth for the mapping between sim-internal
representation (joints ndarray, gripper 0-1) and LeRobot's
flat-dict convention (``shoulder_pan.pos``, ``gripper.pos`` 0-100).

Supports two modes:
  - **Static** (backward compat): uses hardcoded SO-101 joint names
  - **Dynamic** (preferred): derives names from MuJoCoWorld manifest

Usage::

    # Static (SO-101 default)
    from norma_sim.lerobot_helpers import JOINT_NAMES, build_state_vector

    # Dynamic (any robot)
    from norma_sim.lerobot_helpers import RobotSpec
    spec = RobotSpec.from_world(world)
    state = spec.build_state_vector(obs)
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from .world.model import MuJoCoWorld

# ── Static defaults (SO-101 backward compatibility) ──

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
GRIPPER_NAME = "gripper"
ALL_MOTOR_NAMES = [f"{n}.pos" for n in JOINT_NAMES] + [f"{GRIPPER_NAME}.pos"]

# Gripper scaling: LeRobot uses 0-100, sim uses 0-1
GRIPPER_LEROBOT_SCALE = 100.0


# ── Dynamic robot spec (derived from manifest) ──

class RobotSpec:
    """Robot joint/gripper spec derived from manifest — no hardcoded names.

    Use this instead of JOINT_NAMES when supporting multiple robots.
    """

    def __init__(
        self,
        joint_names: list[str],
        gripper_names: list[str],
        gripper_scale: float = GRIPPER_LEROBOT_SCALE,
    ):
        self.joint_names = joint_names
        self.gripper_names = gripper_names
        self.gripper_scale = gripper_scale
        self.n_joints = len(joint_names)
        self.n_grippers = len(gripper_names)
        self.motor_names = (
            [f"{n}.pos" for n in joint_names]
            + [f"{n}.pos" for n in gripper_names]
        )
        self.n_motors = len(self.motor_names)

    @classmethod
    def from_world(cls, world: "MuJoCoWorld") -> "RobotSpec":
        """Derive spec from MuJoCoWorld — reads manifest actuator IDs."""
        return cls(
            joint_names=[a.actuator_id for a in world.joint_actuators],
            gripper_names=[a.actuator_id for a in world.gripper_actuators],
        )

    @classmethod
    def so101(cls) -> "RobotSpec":
        """SO-101 default (same as static JOINT_NAMES)."""
        return cls(joint_names=list(JOINT_NAMES), gripper_names=[GRIPPER_NAME])

    def sim_obs_to_lerobot(self, sim_obs: dict[str, Any]) -> dict[str, Any]:
        """Convert FastSim observation → LeRobot flat dict."""
        obs: dict[str, Any] = {}
        joints = sim_obs.get("joints", np.array([]))
        for i, name in enumerate(self.joint_names):
            if i < len(joints):
                obs[f"{name}.pos"] = float(joints[i])

        gripper = sim_obs.get("gripper", np.array([]))
        for i, name in enumerate(self.gripper_names):
            if i < len(gripper):
                obs[f"{name}.pos"] = float(gripper[i]) * self.gripper_scale

        for key, val in sim_obs.items():
            if key.startswith("camera.") and isinstance(val, np.ndarray):
                cam_name = key[len("camera."):]
                obs[f"observation.images.{cam_name}"] = val
            elif key.startswith("object.") and isinstance(val, np.ndarray):
                obs[key] = val
        return obs

    def lerobot_action_to_sim(
        self, action: dict[str, Any]
    ) -> tuple[np.ndarray, float]:
        """Convert LeRobot action dict → (joints, gripper_normalized)."""
        joints = np.array([
            action.get(f"{name}.pos", 0.0) for name in self.joint_names
        ], dtype=np.float64)
        gripper = action.get(f"{self.gripper_names[0]}.pos", 0.0) if self.gripper_names else 0.0
        return joints, gripper / self.gripper_scale

    def build_state_vector(self, sim_obs: dict[str, Any]) -> np.ndarray:
        """Build (n_motors,) state vector from sim observation."""
        joints = sim_obs.get("joints", np.zeros(self.n_joints))
        gripper = sim_obs.get("gripper", np.array([0.0]))
        vals = list(joints[:self.n_joints])
        for i in range(self.n_grippers):
            g = float(gripper[i]) if i < len(gripper) else 0.0
            vals.append(g * self.gripper_scale)
        return np.array(vals, dtype=np.float32)

    def build_action_vector(
        self, joint_positions: list[float] | np.ndarray, gripper_normalized: float
    ) -> np.ndarray:
        """Build (n_motors,) action vector."""
        vals = list(joint_positions[:self.n_joints])
        vals.append(gripper_normalized * self.gripper_scale)
        return np.array(vals, dtype=np.float32)

    def build_features(self, cameras: dict[str, tuple[int, int]] | None = None) -> dict:
        """Build LeRobotDataset features dict."""
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (self.n_motors,),
                "names": {"motors": self.motor_names},
            },
            "action": {
                "dtype": "float32",
                "shape": (self.n_motors,),
                "names": {"motors": self.motor_names},
            },
        }
        if cameras:
            for cam_name, (h, w) in cameras.items():
                features[f"observation.images.{cam_name}"] = {
                    "dtype": "image",
                    "shape": (h, w, 3),
                    "names": ["height", "width", "channel"],
                }
        return features


# ── Legacy functions (delegate to SO-101 spec) ──

_so101 = RobotSpec.so101()


def sim_obs_to_lerobot(sim_obs: dict[str, Any]) -> dict[str, Any]:
    return _so101.sim_obs_to_lerobot(sim_obs)


def lerobot_action_to_sim(action: dict[str, Any]) -> tuple[np.ndarray, float]:
    return _so101.lerobot_action_to_sim(action)


def build_state_vector(sim_obs: dict[str, Any]) -> np.ndarray:
    return _so101.build_state_vector(sim_obs)


def build_action_vector(
    joint_positions: list[float] | np.ndarray, gripper_normalized: float
) -> np.ndarray:
    return _so101.build_action_vector(joint_positions, gripper_normalized)
