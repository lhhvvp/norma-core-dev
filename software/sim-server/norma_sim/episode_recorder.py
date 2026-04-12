"""Gymnasium wrapper that records episodes automatically.

Usage::

    env = NormaSimEnv(manifest_path="...", render_port=8012)
    env = EpisodeRecorder(env, save_dir="episodes/")

    obs, info = env.reset()
    for _ in range(100):
        action = policy(obs)
        obs, reward, terminated, truncated, info = env.step(action)
    env.close()  # saves the last episode

Each reset() starts a new episode.  The previous episode is saved
to ``save_dir/ep_NNNN.npz`` automatically.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import gymnasium as gym

from .episode import EpisodeBuilder


class EpisodeRecorder(gym.Wrapper):
    """Wraps any NormaCore env and records episodes to disk."""

    def __init__(
        self,
        env: gym.Env,
        save_dir: str | Path = "episodes",
        metadata: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(env)
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self._base_metadata = metadata or {}
        self._builder: EpisodeBuilder | None = None
        self._episode_count = 0
        self._last_action: dict[str, Any] | None = None

    def reset(self, **kwargs) -> tuple[dict[str, Any], dict[str, Any]]:
        # Save previous episode if it has steps
        self._save_current()

        obs, info = self.env.reset(**kwargs)

        # Start new episode
        meta = {
            **self._base_metadata,
            "episode_index": self._episode_count,
        }
        # Pull env metadata if available
        inner = self.env
        if hasattr(inner, "physics_hz"):
            meta["physics_hz"] = inner.physics_hz
        if hasattr(inner, "actual_action_hz"):
            meta["actual_action_hz"] = inner.actual_action_hz
        if hasattr(inner, "manifest_path"):
            meta["manifest_path"] = str(inner.manifest_path)

        self._builder = EpisodeBuilder(metadata=meta)
        self._last_action = None
        return obs, info

    def step(self, action: dict[str, Any]):
        obs, reward, terminated, truncated, info = self.env.step(action)
        if self._builder is not None:
            self._builder.add_step(obs=obs, action=action, info=info)
        self._last_action = action
        return obs, reward, terminated, truncated, info

    def close(self) -> None:
        self._save_current()
        super().close()

    def _save_current(self) -> None:
        if self._builder is not None and self._builder._timestamps:
            episode = self._builder.build()
            path = self.save_dir / f"ep_{self._episode_count:04d}.npz"
            episode.save(path)
            self._episode_count += 1
            self._builder = None
