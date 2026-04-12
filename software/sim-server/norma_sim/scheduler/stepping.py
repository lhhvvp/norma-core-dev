"""`SteppingScheduler` — step-on-demand for Gymnasium integration.

Unlike `RealTimeScheduler`, this scheduler does NOT run a continuous
loop.  Each call to `step(n)` advances physics by exactly *n* ticks
and returns a `WorldSnapshot` synchronously.  The caller (IPC session)
drives the pace — there is no wall-clock pacing or background thread.

This scheduler is selected with ``norma_sim --mode stepping``.
"""
from __future__ import annotations

from typing import Callable, Optional

from ..ipc.codec import WorldClock
from ..world.actuation import ActuationApplier
from ..world.model import MuJoCoWorld
from ..world.snapshot import SnapshotBuilder


class SteppingScheduler:
    """Synchronous step-on-demand scheduler."""

    def __init__(
        self,
        world: MuJoCoWorld,
        applier: ActuationApplier,
        builder: SnapshotBuilder,
        physics_hz: int = 500,
        on_render: Optional[Callable[[], None]] = None,
    ) -> None:
        self.world = world
        self.applier = applier
        self.builder = builder
        self.physics_hz = physics_hz
        self._on_render = on_render
        self._tick = 0

    @property
    def tick(self) -> int:
        return self._tick

    def _make_clock(self):
        return WorldClock(
            world_tick=self._tick,
            sim_time_ns=int(self._tick * (1e9 / self.physics_hz)),
            wall_time_ns=0,
        )

    def step(self, n_ticks: int = 1):
        """Advance physics by *n_ticks* steps, return snapshot."""
        with self.world.lock:
            for _ in range(n_ticks):
                self.world.step()
                self._tick += 1
        if self._on_render is not None:
            self._on_render()
        return self.builder.build(clock=self._make_clock())

    def reset(self, seed=None):
        """Reset MuJoCo state and return initial snapshot.

        *seed* is reserved for future domain randomization.  MuJoCo
        itself is deterministic, so the same model always resets to
        the same state regardless of seed.
        """
        with self.world.lock:
            self.world.reset()
        self._tick = 0
        if self._on_render is not None:
            self._on_render()
        return self.builder.build(clock=self._make_clock())

    def run_forever(self) -> None:
        """Not used — stepping scheduler is driven by IPC requests."""
        raise NotImplementedError(
            "SteppingScheduler is request-driven; use step()/reset() instead"
        )

    def stop(self) -> None:
        """No-op — nothing to stop."""
