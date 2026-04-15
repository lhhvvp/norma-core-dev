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

    # Success criterion (matches scene_tabletop.xml cube initial pose)
    object_body_name: str = "cube"
    object_initial_pos: tuple[float, float, float] = (0.20, 0.0, 0.025)
    success_horizontal_displacement: float = 0.03  # metres — cube moved >3cm sideways
    success_min_height: float = -0.05               # metres — cube not fallen below floor

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
        """Loose success check: cube has been relocated horizontally.

        **This is an obs-only, stateless check.** It only inspects the
        cube's final horizontal position relative to its MJCF-declared
        initial pose. It cannot distinguish "actually picked up and
        carried" from "nudged sideways without ever being lifted" ——
        both score as success if the cube ended up >3 cm from start.

        A stricter criterion (e.g., "cube was lifted above initial z
        at some point during the episode") requires stateful tracking
        across the episode and is out of scope for this stateless
        protocol method. If you need that, track `peak_z` externally
        during rollout and combine it with this return value.

        Requires ``obs["object.<name>.pos"]`` to be present — callers
        must enable tracked-object observation via
        ``NormaSimRobotConfig.tracked_objects=[self.object_body_name]``
        (or equivalent FastSim wiring). Returns ``None`` if the pose is
        missing so the caller can distinguish "unknown" from "failed".
        """
        key = f"object.{self.object_body_name}.pos"
        pos = obs.get(key)
        if pos is None:
            return None

        pos = np.asarray(pos, dtype=np.float64)
        initial = np.asarray(self.object_initial_pos, dtype=np.float64)

        horizontal = float(np.linalg.norm(pos[:2] - initial[:2]))
        moved_sideways = horizontal > self.success_horizontal_displacement
        not_fallen = float(pos[2]) > self.success_min_height

        return moved_sideways and not_fallen
