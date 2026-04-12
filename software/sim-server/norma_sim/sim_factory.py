"""Sim backend factory — CPU or GPU, same interface.

Selects the appropriate simulation backend based on availability
and requested scale. Both backends share manifest loading and
capability conversion from ``norma_sim.world``.

Usage::

    from norma_sim.sim_factory import create_sim

    # Auto-select: CPU for small runs, MJX for large
    sim = create_sim("scene.yaml", cameras={"top": (224, 224)})

    # Force backend
    sim = create_sim("scene.yaml", backend="cpu")
    sim = create_sim("scene.yaml", backend="mjx", n_envs=4096)
"""
from __future__ import annotations

from pathlib import Path


def mjx_available() -> bool:
    """Check if MJX + JAX are importable."""
    try:
        import jax  # noqa: F401
        from mujoco import mjx  # noqa: F401
        return True
    except ImportError:
        return False


def create_sim(
    manifest_path: str | Path,
    cameras: dict[str, tuple[int, int]] | None = None,
    physics_hz: int = 500,
    action_hz: int = 30,
    backend: str = "auto",
    n_envs: int = 1,
):
    """Create a simulation backend.

    Args:
        manifest_path: Path to scene.yaml
        cameras: Camera name → (height, width) mapping
        physics_hz: Physics timestep frequency
        action_hz: Action/control frequency
        backend: "cpu", "mjx", or "auto"
            - "cpu": Always use FastSim (CPU MuJoCo + OpenGL rendering)
            - "mjx": Always use FastSimMJX (GPU MuJoCo + Madrona rendering)
            - "auto": Use MJX if available AND n_envs > 1, else CPU
        n_envs: Number of parallel environments (only used by MJX backend)

    Returns:
        FastSim or FastSimMJX instance.

    When to use which:
        - Debugging / visualization / eval → CPU (instant startup, mjviser)
        - Quick data gen (5-50 eps) → CPU (no JIT overhead)
        - Large data gen (200+ eps) → CPU is fine after P1 fix (~32 min)
        - Massive data gen (10K+ eps) → MJX
        - RL training loop → MJX (training IS simulation)
        - While GPU trains neural net → CPU (no GPU contention)
    """
    if backend == "auto":
        backend = "mjx" if (n_envs > 1 and mjx_available()) else "cpu"

    if backend == "cpu":
        from .fast_sim import FastSim
        return FastSim(
            manifest_path=manifest_path,
            cameras=cameras,
            physics_hz=physics_hz,
            action_hz=action_hz,
        )

    if backend == "mjx":
        # Lazy import — JAX is heavy and optional
        from .fast_sim_mjx import FastSimMJX
        return FastSimMJX(
            manifest_path=manifest_path,
            cameras=cameras,
            physics_hz=physics_hz,
            action_hz=action_hz,
            n_envs=n_envs,
        )

    raise ValueError(f"Unknown backend: {backend!r}. Use 'cpu', 'mjx', or 'auto'.")
