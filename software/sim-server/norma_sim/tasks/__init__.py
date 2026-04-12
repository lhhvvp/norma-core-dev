"""Task definitions for data generation and evaluation.

A Task encapsulates everything needed to generate training data
for a specific manipulation behavior: trajectory generation,
domain randomization, and optional reward/success criteria.

Usage::

    from norma_sim.tasks import REGISTRY, PickAndPlace

    task = REGISTRY["pick_and_place"]
    trajectory = task.generate_trajectory(rng, n_frames=260)
"""
from .base import Task, Trajectory
from .pick_and_place import PickAndPlace
from .registry import REGISTRY

__all__ = ["Task", "Trajectory", "PickAndPlace", "REGISTRY"]
