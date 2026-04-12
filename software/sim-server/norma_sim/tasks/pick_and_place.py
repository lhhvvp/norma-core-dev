"""Pick-and-place task definition.

Generates scripted reach→grasp→lift→carry→release trajectories
with domain randomization on target position, speed, and noise.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .base import Trajectory


@dataclass
class PickAndPlace:
    """Pick up an object and place it to the side."""

    name: str = "pick_and_place"
    description: str = "pick up the red cube and place it to the side"

    # Domain randomization ranges
    domain_randomization: dict[str, Any] = field(default_factory=lambda: {
        "target_shoulder_pan": (-0.8, 0.8),
        "approach_elbow_flex": (1.2, 1.6),
        "lift_elbow_flex": (0.8, 1.2),
        "speed_factor": (0.7, 1.3),
        "home_noise_std": 0.05,
        "action_noise_std": 0.02,
    })

    def generate_trajectory(self, rng: np.random.Generator) -> Trajectory:
        """Generate one randomized pick-and-place trajectory."""
        dr = self.domain_randomization
        pan = rng.uniform(*dr["target_shoulder_pan"])
        approach_flex = rng.uniform(*dr["approach_elbow_flex"])
        lift_flex = rng.uniform(*dr["lift_elbow_flex"])
        speed = rng.uniform(*dr["speed_factor"])

        def s(base_steps: int) -> int:
            return max(10, int(base_steps * speed))

        home = [float(n) for n in rng.normal(0, dr["home_noise_std"], size=5)]

        waypoints = [
            ("home",      home,                                      0.0, s(30)),
            ("above",     [0.0, -0.6,  1.3, -0.1,  0.0],           0.0, s(40)),
            ("approach",  [0.0, -0.6,  approach_flex,  0.0,  0.0],  0.0, s(30)),
            ("grasp",     [0.0, -0.6,  approach_flex,  0.0,  0.0],  1.0, s(20)),
            ("lift",      [0.0, -0.6,  lift_flex, -0.3,  0.0],      1.0, s(40)),
            ("carry",     [pan, -0.4,  0.8, -0.2,  0.0],            1.0, s(40)),
            ("release",   [pan, -0.4,  0.8, -0.2,  0.0],            0.0, s(20)),
            ("home",      home,                                      0.0, s(40)),
        ]

        return Trajectory(
            waypoints=waypoints,
            metadata={
                "pan": pan,
                "approach_flex": approach_flex,
                "lift_flex": lift_flex,
                "speed": speed,
            },
        )

    def check_success(self, obs: dict[str, Any]) -> bool | None:
        """Cannot determine success without object pose tracking."""
        return None
