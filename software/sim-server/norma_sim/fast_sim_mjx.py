"""GPU-parallel MuJoCo simulation via MJX (JAX/XLA).

Runs N environments in parallel on GPU using jax.vmap over mjx.step.
Same observation format as FastSim (CPU) so callers don't need to
know which backend they're using.

Requires: pip install 'jax[cuda12]' mujoco-mjx

Usage::

    sim = FastSimMJX("scene.yaml", n_envs=32)
    batch_obs = sim.reset()
    batch_obs = sim.step(batch_joints, batch_gripper)
    # batch_obs["joints"].shape = (32, 5)

Note: First call to step/reset triggers JIT compilation which may take
1-10+ minutes for complex scenes. Subsequent calls run in milliseconds.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from .world.model import MuJoCoWorld

try:
    import jax
    import jax.numpy as jnp
    from mujoco import mjx
except ImportError as e:
    raise ImportError(
        "FastSimMJX requires JAX and MuJoCo MJX. "
        "Install with: pip install 'jax[cuda12]' mujoco-mjx"
    ) from e


class FastSimMJX:
    """GPU-parallel MuJoCo simulation — N environments on one GPU."""

    def __init__(
        self,
        manifest_path: str | Path,
        cameras: dict[str, tuple[int, int]] | None = None,
        physics_hz: int = 500,
        action_hz: int = 30,
        n_envs: int = 32,
    ) -> None:
        self.n_envs = n_envs
        self.substeps = physics_hz // action_hz
        self._cameras = cameras or {}

        # ── Load model (CPU MuJoCo → GPU MJX) ──
        self.world = MuJoCoWorld.from_manifest_path(manifest_path)
        self.mj_model = self.world.model
        self.mjx_model = mjx.put_model(self.mj_model)

        # ── Actuator mapping (from MuJoCoWorld — single source of truth) ──
        self._joint_indices = [self.world.actuator_id_for(a.mjcf_actuator) for a in self.world.joint_actuators]
        self._gripper_indices = [self.world.actuator_id_for(a.mjcf_actuator) for a in self.world.gripper_actuators]

        # Pre-compute ctrl mapping as JAX arrays (no Python branching in JIT)
        n_actuators = self.mj_model.nu
        scales = np.ones(n_actuators, dtype=np.float32)
        offsets = np.zeros(n_actuators, dtype=np.float32)
        for act in self.world.gripper_actuators:
            idx = self.world.actuator_id_for(act.mjcf_actuator)
            if act.capability.kind == "GRIPPER_PARALLEL" and act.gripper:
                g = act.gripper
                norm_lo, norm_hi = g.normalized_range
                joint_lo, joint_hi = g.primary_joint_range_rad
                scales[idx] = (joint_hi - joint_lo) / (norm_hi - norm_lo)
                offsets[idx] = joint_lo - scales[idx] * norm_lo
        self._ctrl_scales = jnp.array(scales)
        self._ctrl_offsets = jnp.array(offsets)

        # Joint/gripper qpos addresses for reading state
        self._joint_qposadr = [self.world.joint_qposadr_for(a.mjcf_joint) for a in self.world.joint_actuators]
        self._gripper_qposadr = [self.world.joint_qposadr_for(a.mjcf_joint) for a in self.world.gripper_actuators]

        self._joint_qposadr_jax = jnp.array(self._joint_qposadr)
        self._gripper_qposadr_jax = jnp.array(self._gripper_qposadr)
        self._joint_ctrl_idx = jnp.array(self._joint_indices)
        self._gripper_ctrl_idx = jnp.array(self._gripper_indices)

        # ── Initial data template ──
        mj_data = mujoco.MjData(self.mj_model)
        mujoco.mj_forward(self.mj_model, mj_data)
        self._mjx_data_template = mjx.put_data(self.mj_model, mj_data)

        # ── JIT-compiled functions ──
        self._jit_step = self._build_step_fn()
        self._jit_reset = self._build_reset_fn()

        # State
        self._batch_data = None

        # Note: cameras not yet supported (requires Madrona renderer)
        if self._cameras:
            import warnings
            warnings.warn(
                "FastSimMJX does not yet support camera rendering. "
                "Observations will contain state only (no images). "
                "Use FastSim (CPU) for vision-based data generation."
            )

    def _build_step_fn(self):
        """Build JIT-compiled batched step function."""
        mjx_model = self.mjx_model
        substeps = self.substeps

        @jax.jit
        def _step(batch_data, batch_ctrl):
            # Apply control
            batch_data = batch_data.replace(ctrl=batch_ctrl)
            # Substeps via scan (single XLA op, not Python loop)
            def one_substep(bd, _):
                return jax.vmap(lambda d: mjx.step(mjx_model, d))(bd), None
            batch_data, _ = jax.lax.scan(one_substep, batch_data, None, length=substeps)
            return batch_data

        return _step

    def _build_reset_fn(self):
        """Build JIT-compiled batched reset function."""
        template = self._mjx_data_template
        n_envs = self.n_envs

        @jax.jit
        def _reset():
            return jax.vmap(lambda _: template)(jnp.arange(n_envs))

        return _reset

    def reset(self) -> dict[str, Any]:
        """Reset all N environments, return batched observations."""
        self._batch_data = self._jit_reset()
        return self._extract_obs(self._batch_data)

    def step(
        self,
        batch_joints: np.ndarray | jax.Array,
        batch_gripper: np.ndarray | jax.Array,
    ) -> dict[str, Any]:
        """Step all N environments in parallel.

        Args:
            batch_joints: (n_envs, n_joints) joint position commands
            batch_gripper: (n_envs,) or (n_envs, 1) normalized gripper commands

        Returns:
            Batched observation dict with JAX arrays.
        """
        if self._batch_data is None:
            self.reset()

        batch_joints = jnp.asarray(batch_joints, dtype=jnp.float32)
        batch_gripper = jnp.asarray(batch_gripper, dtype=jnp.float32)
        if batch_gripper.ndim == 1:
            batch_gripper = batch_gripper[:, None]

        # Build full ctrl array (n_envs, n_actuators)
        n_act = self.mj_model.nu
        ctrl = jnp.zeros((self.n_envs, n_act), dtype=jnp.float32)
        ctrl = ctrl.at[:, self._joint_ctrl_idx].set(batch_joints)
        ctrl = ctrl.at[:, self._gripper_ctrl_idx].set(batch_gripper)

        # Apply scale/offset mapping (handles GRIPPER_PARALLEL)
        ctrl = ctrl * self._ctrl_scales + self._ctrl_offsets

        self._batch_data = self._jit_step(self._batch_data, ctrl)
        return self._extract_obs(self._batch_data)

    def _extract_obs(self, batch_data) -> dict[str, Any]:
        """Extract observation dict from batched MJX data."""
        qpos = batch_data.qpos  # (n_envs, nq)
        obs: dict[str, Any] = {
            "joints": qpos[:, self._joint_qposadr_jax],     # (n_envs, n_joints)
            "gripper": qpos[:, self._gripper_qposadr_jax],   # (n_envs, n_grippers)
        }
        return obs

    def get_numpy(self, obs: dict[str, Any]) -> dict[str, np.ndarray]:
        """Move observation from GPU to CPU as numpy arrays."""
        return {k: np.asarray(v) for k, v in obs.items()}

    def close(self) -> None:
        self._batch_data = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
