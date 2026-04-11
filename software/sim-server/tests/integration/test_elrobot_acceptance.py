"""MVP-2 Phase 2 acceptance: 6 Floor criteria for ElRobot smoothness.

This is the definition of done for MVP-2. If any test fails, iterate
on elrobot_follower.xml until they all pass — 5-iteration tuning
budget, then escalate per spec §7.5 / §10 Risk B.

Criteria (spec §3.1):
  Floor 1 — no self-collision at rest
  Floor 2 — effective inertia floor (M[i,i] + armature >= 1e-4)
  Floor 3 — 10000-step stress with no NaN
  Floor 4 — per-motor step response (0.9 × ctrl_hi reached in 2s, overshoot ≤ 15%)
  Floor 5 — P0 mimic gripper regression (delegated to test_mimic_gripper.py)
  Floor 6 — MVP-1 test suite still green (delegated to make sim-test)
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

try:
    import mujoco
    from norma_sim.world.model import MuJoCoWorld
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"imports not OK: {_ERR}")


# ----------------------------------------------------------------------
# Floor 1 — no self-collision at rest
# ----------------------------------------------------------------------

def test_elrobot_no_self_collision_at_rest(elrobot_scene_yaml):
    """mj_forward at home pose should produce zero contacts.
    This catches URDF mesh overlap issues that MVP-1 had."""
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    mujoco.mj_forward(world.model, world.data)
    assert world.data.ncon == 0, (
        f"ElRobot should have clean collision at rest, got "
        f"{world.data.ncon} contacts. MVP-2 spec §3.1 Floor 1."
    )


# ----------------------------------------------------------------------
# Floor 2 — effective inertia floor
# ----------------------------------------------------------------------

def test_elrobot_effective_inertia_floor(elrobot_scene_yaml):
    """Every DOF should have M[i,i] + armature[i] >= 1e-4 kg·m².
    This ensures no joint is numerically ill-conditioned (MVP-1's
    gripper primary joint had 2.5e-7 — catastrophic)."""
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    mujoco.mj_forward(world.model, world.data)
    M = np.zeros((world.model.nv, world.model.nv))
    mujoco.mj_fullM(world.model, M, world.data.qM)
    failures = []
    for i in range(world.model.nv):
        effective = M[i, i] + world.model.dof_armature[i]
        if effective < 1e-4:
            # Resolve the joint name for a clearer error
            joint_id = None
            for j in range(world.model.njnt):
                if world.model.jnt_dofadr[j] == i:
                    joint_id = j
                    break
            joint_name = (
                mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                if joint_id is not None else f"dof{i}"
            )
            failures.append(
                f"DOF {i} ({joint_name}): M[i,i]={M[i,i]:.2e}, "
                f"armature={world.model.dof_armature[i]:.2e}, "
                f"total={effective:.2e}"
            )
    assert not failures, (
        "Spec §3.1 Floor 2 failures (effective inertia < 1e-4 kg·m²):\n"
        + "\n".join(failures)
    )


# ----------------------------------------------------------------------
# Floor 3 — stress test: 10000 random-ctrl steps, no NaN
# ----------------------------------------------------------------------

def test_elrobot_stress_10000_random_steps_no_nan(elrobot_scene_yaml):
    """10000 random-ctrl steps (resample every 100), verify qpos stays finite."""
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    rng = np.random.default_rng(42)
    lo = world.model.actuator_ctrlrange[:, 0]
    hi = world.model.actuator_ctrlrange[:, 1]
    for step in range(10000):
        if step % 100 == 0:
            world.data.ctrl[:] = rng.uniform(lo, hi)
        world.step()
        if step % 1000 == 0:
            assert np.isfinite(world.data.qpos).all(), (
                f"NaN at step {step}. Spec §3.1 Floor 3."
            )
    assert np.isfinite(world.data.qpos).all()
    assert np.isfinite(world.data.qvel).all()


# ----------------------------------------------------------------------
# Floor 4 — per-motor step response
# ----------------------------------------------------------------------

@pytest.fixture
def elrobot_world(elrobot_scene_yaml):
    """Shared fixture for all 8 motor parametrizations. A fresh
    MuJoCoWorld per test to avoid state leakage."""
    return MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)


@pytest.mark.parametrize("motor_idx", range(8))
def test_elrobot_motor_step_response(elrobot_scene_yaml, motor_idx: int):
    """Drive motor `motor_idx` to 0.9 × ctrlrange_hi. Verify:
    - qpos reaches ≥ 80% of target within 2s (1000 steps @ dt=0.002)
    - overshoot ≤ 15% of the target value

    Parametrized so failure messages specify which motor is broken.
    Spec §3.1 Floor 4."""
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    assert 0 <= motor_idx < world.model.nu, (
        f"motor_idx {motor_idx} out of range; ElRobot has {world.model.nu} actuators"
    )

    ctrl_lo = float(world.model.actuator_ctrlrange[motor_idx, 0])
    ctrl_hi = float(world.model.actuator_ctrlrange[motor_idx, 1])
    target = 0.9 * ctrl_hi
    # Skip motors where 0.9 × ctrl_hi would also be 0 (symmetric range)
    if abs(target) < 1e-9:
        target = 0.5 * ctrl_hi  # use half-range instead
        if abs(target) < 1e-9:
            pytest.skip(f"motor {motor_idx} has zero ctrl range, cannot step-response test")

    # Find the qpos address for the joint this actuator controls
    joint_id = int(world.model.actuator_trnid[motor_idx, 0])
    qadr = int(world.model.jnt_qposadr[joint_id])
    joint_name = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    actuator_name = mujoco.mj_id2name(
        world.model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_idx
    )

    world.data.ctrl[motor_idx] = target
    max_reached = 0.0
    for _ in range(1000):  # 2.0 sec at dt=0.002
        world.step()
        q = float(world.data.qpos[qadr])
        if target > 0 and q > max_reached:
            max_reached = q
        if target < 0 and q < max_reached:
            max_reached = q

    final = float(world.data.qpos[qadr])
    reached_fraction = final / target if abs(target) > 1e-9 else 0.0
    overshoot = (
        abs(max_reached - target) / abs(target)
        if abs(target) > 1e-9 and abs(max_reached) > abs(target)
        else 0.0
    )

    assert reached_fraction >= 0.8, (
        f"Motor {motor_idx} ({actuator_name}, joint {joint_name}) "
        f"only reached {final:.4f}/{target:.4f} = {reached_fraction:.1%} "
        f"within 2.0s. Spec §3.1 Floor 4 requires >= 80%. "
        f"Tune kp up or armature down for this joint in elrobot_follower.xml."
    )
    assert overshoot <= 0.15, (
        f"Motor {motor_idx} ({actuator_name}) overshot by {overshoot:.1%} "
        f"(max={max_reached:.4f}, target={target:.4f}). "
        f"Spec §3.1 Floor 4 requires <= 15%. Tune kv up or add joint damping."
    )


# ----------------------------------------------------------------------
# Floor 5 — P0 mimic gripper regression is delegated to test_mimic_gripper.py
# (Chunk 3 Task 3.5 migrated those tests; Chunk 5 Task 5.4 verified they pass)
# ----------------------------------------------------------------------

def test_floor_5_delegation_note():
    """Floor 5 (P0 gripper mimic) is covered by
    software/sim-server/tests/world/test_mimic_gripper.py. This stub
    exists only to make the Floor 5 intent visible in this file.
    Running `pytest test_mimic_gripper.py` must show 2 PASSED."""
    # No actual test logic — Floor 5 is a delegated concern.
    pass


# ----------------------------------------------------------------------
# Floor 6 — full MVP-1 test suite still green is delegated to `make sim-test`
# ----------------------------------------------------------------------

def test_floor_6_delegation_note():
    """Floor 6 (MVP-1 test suite still green) is covered by
    `make sim-test` running in CI / locally. This stub exists only to
    make the Floor 6 intent visible in this file. The Chunk 7 gate
    runs `make sim-test` and asserts 0 failures."""
    pass
