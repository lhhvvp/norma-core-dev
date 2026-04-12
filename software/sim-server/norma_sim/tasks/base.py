"""Base Task and Trajectory types."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Protocol

import numpy as np


@dataclass
class Trajectory:
    """A sequence of (joint_targets, gripper_target) waypoints.

    Each entry: (name, joints list, gripper 0-1, n_interpolation_steps).
    """
    waypoints: list[tuple[str, list[float], float, int]]
    metadata: dict[str, Any] = field(default_factory=dict)


class Task(Protocol):
    """Protocol for task definitions.

    Implement this to add new manipulation tasks (e.g., stacking,
    drawer opening). batch_generate.py and eval scripts consume this.
    """

    @property
    def name(self) -> str:
        """Short identifier (e.g., 'pick_and_place')."""
        ...

    @property
    def description(self) -> str:
        """Natural language instruction for VLA models."""
        ...

    @property
    def domain_randomization(self) -> dict[str, Any]:
        """Randomization parameter ranges."""
        ...

    def generate_trajectory(
        self, rng: np.random.Generator
    ) -> Trajectory:
        """Generate one randomized trajectory for this task."""
        ...

    def check_success(self, obs: dict[str, Any]) -> bool | None:
        """Check if the task was completed successfully.

        Returns True/False for success/failure, or None if not
        determinable from observation alone (e.g., no object tracking).
        """
        ...
