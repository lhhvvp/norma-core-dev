"""`RealTimeScheduler` — pace MuJoCo physics to wall clock.

Spec §8.3–8.4. Each tick's deadline is `t0_wall + tick * physics_ns`
(NOT `last_deadline + physics_ns`, which would drift on catch-up).

Pacing policy:
  - Compute deadline for the next tick.
  - If slack_ns > 0, sleep that long then step.
  - If slack_ns < 0, we overran: don't catch up, just increment a
    counter and log a warning. The next deadline is still anchored
    to the original `t0_wall + tick * physics_ns`, so steady-state
    pacing recovers automatically when the overload subsides.

The scheduler drives `world.step()` and calls an injectable
`on_publish(tick)` callback every `publish_divider` physics ticks so
the IPC layer can emit a snapshot at `publish_hz < physics_hz`.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

from ..world.model import MuJoCoWorld

_log = logging.getLogger("norma_sim.scheduler")


class RealTimeScheduler:
    def __init__(
        self,
        world: MuJoCoWorld,
        physics_hz: int = 500,
        publish_hz: int = 100,
        on_publish: Optional[Callable[[int], None]] = None,
    ) -> None:
        if physics_hz <= 0:
            raise ValueError("physics_hz must be positive")
        if publish_hz <= 0 or publish_hz > physics_hz:
            raise ValueError("publish_hz must be in (0, physics_hz]")

        self.world = world
        self.physics_hz = physics_hz
        self.publish_hz = publish_hz
        self._physics_ns = 1_000_000_000 // physics_hz
        self._publish_divider = max(1, physics_hz // publish_hz)
        self._on_publish = on_publish

        self._tick = 0
        self._stop = threading.Event()
        self.overrun_count = 0

    @property
    def tick(self) -> int:
        return self._tick

    def stop(self) -> None:
        self._stop.set()

    def run_forever(self) -> None:
        t0_wall_ns = time.monotonic_ns()
        while not self._stop.is_set():
            deadline_ns = t0_wall_ns + self._tick * self._physics_ns
            now_ns = time.monotonic_ns()
            slack_ns = deadline_ns - now_ns
            if slack_ns > 0:
                time.sleep(slack_ns / 1e9)
            else:
                if slack_ns < -self._physics_ns:
                    # Overrun by more than one physics period.
                    self.overrun_count += 1
                    _log.warning(
                        "scheduler overrun",
                        extra={
                            "extra_fields": {
                                "tick": self._tick,
                                "lag_ns": -slack_ns,
                            }
                        },
                    )

            with self.world.lock:
                self.world.step()

            if self._tick % self._publish_divider == 0 and self._on_publish is not None:
                try:
                    self._on_publish(self._tick)
                except Exception:
                    _log.exception("on_publish callback raised")

            self._tick += 1

    def run_for(self, duration_s: float) -> None:
        """Run the loop for a bounded wall-clock window. Used in tests."""
        deadline = time.monotonic() + duration_s

        def watchdog():
            while time.monotonic() < deadline and not self._stop.is_set():
                time.sleep(0.001)
            self._stop.set()

        t = threading.Thread(target=watchdog, daemon=True)
        t.start()
        try:
            self.run_forever()
        finally:
            t.join(timeout=0.1)
