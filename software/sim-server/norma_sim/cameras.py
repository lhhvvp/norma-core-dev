"""Shared camera configuration for all simulation backends.

Both the runtime (SteppingScheduler) and training (FastSim) paths
use these presets as fallbacks when the MJCF scene doesn't define
named cameras. Consumer scripts control resolution; this module
only defines camera poses.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class CameraConfig:
    """Fixed camera viewpoint for rendering."""

    name: str
    width: int = 320
    height: int = 240
    lookat: tuple[float, float, float] = (0.0, 0.0, 0.1)
    distance: float = 0.8
    azimuth: float = 135.0
    elevation: float = -30.0


# Default camera presets matching common LeRobot SO-101 setups.
# Resolution is the default; callers override width/height as needed.
DEFAULT_CAMERAS = {
    "top": CameraConfig(
        name="top",
        width=640, height=480,
        lookat=(0.0, 0.05, 0.1),
        distance=0.6, azimuth=90.0, elevation=-60.0,
    ),
    "wrist.top": CameraConfig(
        name="wrist.top",
        width=640, height=480,
        lookat=(0.0, 0.05, 0.15),
        distance=0.4, azimuth=180.0, elevation=-45.0,
    ),
}
