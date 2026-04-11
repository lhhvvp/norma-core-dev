"""Scheduler protocol.

A WorldScheduler owns the main simulation loop: it decides when to
call `world.step()`, when to build and publish snapshots, and how to
pace the whole thing relative to wall clock. Different policies
(real-time, fast-as-possible, determinism) are different
`WorldScheduler` implementations.
"""
from __future__ import annotations

from typing import Protocol


class WorldScheduler(Protocol):
    def run_forever(self) -> None:
        """Block on the simulation loop until cancelled."""

    def stop(self) -> None:
        """Signal the run loop to exit at its next iteration."""

    @property
    def tick(self) -> int:
        """Number of physics steps executed so far."""
