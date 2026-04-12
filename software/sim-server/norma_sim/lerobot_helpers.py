"""Shared LeRobot observation/action conversion helpers.

Single source of truth for the mapping between sim-internal
representation (joints ndarray, gripper 0-1) and LeRobot's
flat-dict convention (``shoulder_pan.pos``, ``gripper.pos`` 0-100).

Used by both NormaSimRobot (runtime path) and batch_generate.py
(training path via FastSim), ensuring identical conversion logic.
"""
from __future__ import annotations

from typing import Any

import numpy as np

# LeRobot SO-Follower joint naming convention
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


def sim_obs_to_lerobot(
    sim_obs: dict[str, Any],
) -> dict[str, Any]:
    """Convert FastSim/NormaSimEnv observation → LeRobot flat dict.

    Input keys: ``joints`` (ndarray), ``gripper`` (ndarray 0-1),
    ``camera.<name>`` (ndarray H×W×3 uint8).

    Output keys: ``shoulder_pan.pos`` (float), ..., ``gripper.pos``
    (float 0-100), ``observation.images.<name>`` (ndarray).
    """
    obs: dict[str, Any] = {}

    joints = sim_obs.get("joints", np.array([]))
    for i, name in enumerate(JOINT_NAMES):
        if i < len(joints):
            obs[f"{name}.pos"] = float(joints[i])

    gripper = sim_obs.get("gripper", np.array([]))
    if len(gripper) > 0:
        obs[f"{GRIPPER_NAME}.pos"] = float(gripper[0]) * GRIPPER_LEROBOT_SCALE

    for key, val in sim_obs.items():
        if key.startswith("camera.") and isinstance(val, np.ndarray):
            cam_name = key[len("camera."):]
            obs[f"observation.images.{cam_name}"] = val

    return obs


def lerobot_action_to_sim(
    action: dict[str, Any],
) -> tuple[np.ndarray, float]:
    """Convert LeRobot flat action dict → (joints ndarray, gripper 0-1).

    Input keys: ``shoulder_pan.pos`` (float), ..., ``gripper.pos`` (0-100).
    Returns: (joint_positions array, gripper_normalized float).
    """
    joints = np.array([
        action.get(f"{name}.pos", 0.0)
        for name in JOINT_NAMES
    ], dtype=np.float64)

    gripper_val = action.get(f"{GRIPPER_NAME}.pos", 0.0)
    gripper_normalized = gripper_val / GRIPPER_LEROBOT_SCALE

    return joints, gripper_normalized


def build_state_vector(sim_obs: dict[str, Any]) -> np.ndarray:
    """Build (6,) float32 state vector from sim observation.

    Order: [joint0, ..., joint4, gripper_0_100].
    """
    joints = sim_obs.get("joints", np.zeros(5))
    gripper = sim_obs.get("gripper", np.array([0.0]))
    state = list(joints[:5]) + [float(gripper[0]) * GRIPPER_LEROBOT_SCALE]
    return np.array(state, dtype=np.float32)


def build_action_vector(
    joint_positions: list[float] | np.ndarray,
    gripper_normalized: float,
) -> np.ndarray:
    """Build (6,) float32 action vector.

    Order: [joint0, ..., joint4, gripper_0_100].
    """
    action = list(joint_positions[:5]) + [gripper_normalized * GRIPPER_LEROBOT_SCALE]
    return np.array(action, dtype=np.float32)
