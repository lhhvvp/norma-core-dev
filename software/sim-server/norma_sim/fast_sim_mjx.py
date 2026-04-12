"""GPU-parallel MuJoCo simulation via MJX (JAX/XLA).

Runs N environments in parallel on GPU using jax.vmap over mjx.step.
Same observation format as FastSim (CPU) so callers don't need to
know which backend they're using.

Requires: jax[cuda], mujoco >= 3.0 (includes mjx)

Usage::

    sim = FastSimMJX("scene.yaml", n_envs=4096, cameras={"top": (224, 224)})
    batch_obs = sim.reset()                    # {4096} observations
    batch_obs = sim.step(batch_joints, batch_gripper)  # parallel step

Not yet implemented — this is the interface contract for future work.
See WEEK1_PLAN.md architecture notes for implementation roadmap.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any


class FastSimMJX:
    """GPU-parallel MuJoCo simulation — N environments on one GPU.

    Interface mirrors FastSim (CPU) but operates on batched data.
    All arrays are JAX arrays on GPU; call jax.device_get() to
    move to CPU/numpy when needed (e.g., for dataset writing).
    """

    def __init__(
        self,
        manifest_path: str | Path,
        cameras: dict[str, tuple[int, int]] | None = None,
        physics_hz: int = 500,
        action_hz: int = 30,
        n_envs: int = 4096,
    ) -> None:
        # Fail fast with clear message if JAX not available
        try:
            import jax  # noqa: F401
            from mujoco import mjx  # noqa: F401
        except ImportError as e:
            raise ImportError(
                "FastSimMJX requires JAX and MuJoCo MJX. "
                "Install with: pip install 'jax[cuda12]' mujoco-mjx"
            ) from e

        self.n_envs = n_envs
        self.manifest_path = Path(manifest_path)
        self._cameras = cameras or {}

        # TODO: Implement MJX initialization
        # 1. Load manifest → MuJoCoWorld (CPU, for model loading)
        # 2. mjx.put_model(mj_model) → JAX model on GPU
        # 3. Pre-compute ctrl mapping (scales, offsets) as jax arrays
        # 4. Initialize batch_data via jax.vmap
        # 5. Set up Madrona batch renderer (if cameras configured)
        raise NotImplementedError(
            "FastSimMJX is a planned feature. Current status:\n"
            "  - Interface defined (this file)\n"
            "  - CPU backend (fast_sim.py) works and is tested\n"
            "  - sim_factory.py auto-selects CPU when MJX unavailable\n\n"
            "Implementation roadmap:\n"
            "  Phase 1: batch_step/batch_reset with state-only output\n"
            "  Phase 2: JAX-ified waypoint generation\n"
            "  Phase 3: Madrona batch rendering integration\n"
            "  Phase 4: RL training loop (PPO/SAC on MJX)"
        )

    def reset(self) -> dict[str, Any]:
        """Reset all N environments, return batched observations."""
        ...

    def step(self, batch_joints, batch_gripper) -> dict[str, Any]:
        """Step all N environments in parallel, return batched observations."""
        ...

    def close(self) -> None:
        ...

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
