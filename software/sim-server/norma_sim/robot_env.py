"""Unified interface for all simulation backends.

Three implementations exist:
  - ``FastSim`` (fast_sim.py) — CPU in-process MuJoCo, for data gen + eval
  - ``NormaSimEnv`` (gym_env.py) — subprocess IPC, for runtime control
  - ``FastSimMJX`` (fast_sim_mjx.py) — GPU parallel MJX, for RL training

All produce the same observation format::

    obs = {
        "joints": np.ndarray,        # (n_joints,) radians
        "gripper": np.ndarray,       # (n_grippers,) normalized 0-1
        "camera.<name>": np.ndarray, # (H, W, 3) uint8 (optional)
    }

And accept the same action format:
    joint_positions: np.ndarray  # (n_joints,) target radians
    gripper_normalized: float    # 0.0 (open) to 1.0 (closed)
"""
from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class RobotEnv(Protocol):
    """Protocol for all sim backends.

    Use ``isinstance(sim, RobotEnv)`` to verify a backend conforms.
    """

    def reset(self) -> dict[str, Any]:
        """Reset to initial state, return observation."""
        ...

    def step(
        self,
        joint_positions: np.ndarray,
        gripper_normalized: float,
    ) -> dict[str, Any]:
        """Apply action, advance physics, return observation."""
        ...

    def close(self) -> None:
        """Release resources."""
        ...
