"""Canonical episode format for NormaCore imitation learning.

An `Episode` is a sequence of `EpisodeStep`s recorded from either
`NormaSimEnv` (sim) or a future `NormaHwEnv` (real hardware).  Both
paths write the same format so training code doesn't care about the
source.

Storage: `.npz` (numpy compressed archive) for simplicity.  Each
episode is one file with arrays keyed by field name.

    episode = Episode.load("episodes/ep_0001.npz")
    print(episode.joints_rad.shape)   # (T, n_joints)
    print(episode.metadata)           # dict with physics_hz, seed, etc.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class Episode:
    """One recorded episode (sequence of steps)."""

    # Per-step arrays — shape (T, ...) where T = number of steps.
    timestamp_ns: np.ndarray       # (T,) int64 — monotonic nanoseconds
    joints_rad: np.ndarray         # (T, n_joints) float64
    gripper_normalized: np.ndarray  # (T, n_grippers) float64 [0,1]
    action_joints_rad: np.ndarray  # (T, n_joints) float64 — action that produced this step
    action_gripper: np.ndarray     # (T, n_grippers) float64

    # Metadata
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def n_steps(self) -> int:
        return len(self.timestamp_ns)

    def save(self, path: str | Path) -> Path:
        """Save to .npz file."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            timestamp_ns=self.timestamp_ns,
            joints_rad=self.joints_rad,
            gripper_normalized=self.gripper_normalized,
            action_joints_rad=self.action_joints_rad,
            action_gripper=self.action_gripper,
            # metadata stored as a single-element object array
            metadata=np.array(self.metadata, dtype=object),
        )
        return path

    @classmethod
    def load(cls, path: str | Path) -> "Episode":
        """Load from .npz file."""
        data = np.load(str(path), allow_pickle=True)
        meta = data["metadata"].item() if "metadata" in data else {}
        return cls(
            timestamp_ns=data["timestamp_ns"],
            joints_rad=data["joints_rad"],
            gripper_normalized=data["gripper_normalized"],
            action_joints_rad=data["action_joints_rad"],
            action_gripper=data["action_gripper"],
            metadata=meta,
        )


class EpisodeBuilder:
    """Accumulates steps and builds an Episode."""

    def __init__(self, metadata: dict[str, Any] | None = None) -> None:
        self._timestamps: list[int] = []
        self._joints: list[np.ndarray] = []
        self._grippers: list[np.ndarray] = []
        self._act_joints: list[np.ndarray] = []
        self._act_grippers: list[np.ndarray] = []
        self.metadata = metadata or {}

    def add_step(
        self,
        obs: dict[str, Any],
        action: dict[str, Any],
        info: dict[str, Any],
    ) -> None:
        ts = info.get("sim_time_ns", time.monotonic_ns())
        self._timestamps.append(int(ts))
        self._joints.append(np.asarray(obs.get("joints", []), dtype=np.float64))
        self._grippers.append(np.asarray(obs.get("gripper", []), dtype=np.float64))
        self._act_joints.append(np.asarray(action.get("joints", []), dtype=np.float64))
        self._act_grippers.append(np.asarray(action.get("gripper", []), dtype=np.float64))

    def build(self) -> Episode:
        return Episode(
            timestamp_ns=np.array(self._timestamps, dtype=np.int64),
            joints_rad=np.stack(self._joints) if self._joints else np.empty((0, 0)),
            gripper_normalized=np.stack(self._grippers) if self._grippers else np.empty((0, 0)),
            action_joints_rad=np.stack(self._act_joints) if self._act_joints else np.empty((0, 0)),
            action_gripper=np.stack(self._act_grippers) if self._act_grippers else np.empty((0, 0)),
            metadata=self.metadata,
        )

    def clear(self) -> None:
        self._timestamps.clear()
        self._joints.clear()
        self._grippers.clear()
        self._act_joints.clear()
        self._act_grippers.clear()
