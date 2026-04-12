# MVP-3 Chunk 3 — Test Decoupling Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite the 13 physics-acceptance tests to use raw `mujoco` APIs (no `norma_sim`), move them into the engine package, and add a manifest-pipeline sentinel test to preserve the coverage the rewrite drops.

**Architecture:** Two-commit split per codex iter-1 U2: Commit 1 adds both new test files alongside the old one (pure addition, both coexist green); Commit 2 deletes the old file, cleans up dead fixtures, and bumps metadata (pure deletion + housekeeping). The split lets the reviewer run both old and new suites side-by-side at HEAD-of-commit-1 to audit assertion equivalence.

**Tech Stack:** Python, pytest, mujoco (raw C API bindings), numpy.

**Spec:** `docs/superpowers/specs/2026-04-12-mvp3-foundation-roadmap-design.md` Section 5 (lines 420-705).

**Baselines (Chunk 2 end-state, verified at session start):**
- `make sim-test`: 93 passed, 1 skipped
- Package tests (`hardware/elrobot/simulation/mujoco/elrobot_follower/tests/`): 7 passed, 1 skipped (mimic\_gripper=2, urdf\_parity=2, scene\_loadable=3, mjx\_compat=1 skip)

**Rewrite mapping** (verified against `software/sim-server/norma_sim/world/model.py:20-79`):

| Original (norma\_sim entry) | Rewrite (raw mujoco) |
|---|---|
| `from norma_sim.world.model import MuJoCoWorld` | `import mujoco` |
| `world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)` | `model, data = elrobot_sim` (fixture) |
| `world.model` / `world.data` | `model` / `data` |
| `world.step()` | `mujoco.mj_step(model, data)` |
| `mujoco.mj_forward(world.model, world.data)` | `mujoco.mj_forward(model, data)` |
| `mujoco.mj_fullM(world.model, M, world.data.qM)` | `mujoco.mj_fullM(model, M, data.qM)` |

All `world.model.*` / `world.data.*` field access (qpos, qvel, ctrl, ncon, actuator\_ctrlrange, etc.) is identical after dropping the `world.` prefix — `MuJoCoWorld` is a thin wrapper.

**Spec arithmetic correction:** Spec success criterion #6 says "18 passed + 1 skipped" for `cp -r /tmp` self-containment. Actual count is **20 passed + 1 skipped**: mimic\_gripper=2, urdf\_parity=2, scene\_loadable=**3** (not 1 as spec counted), acceptance=13, mjx=1 skip. The spec undercounted `test_scene_loadable.py` (which has 3 tests, confirmed in CHANGELOG v0.2.1 and `make sim-test` output). Plan uses the correct number.

---

## Chunk 1: Commit 1 — Add Pure-MuJoCo Acceptance + Manifest Sentinel

### Task 1: Add `elrobot_sim` fixture to package conftest

**Files:**
- Modify: `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/conftest.py`

- [ ] **Step 1: Add `import mujoco` and `elrobot_sim` fixture**

Add `import mujoco` at the top (after existing imports). Add fixture after the existing `elrobot_mjcf_path` fixture:

```python
import mujoco


@pytest.fixture
def elrobot_sim(elrobot_mjcf_path):
    """Fresh MjModel + MjData pair.  Function-scoped (default) so each
    test gets clean state — no leakage between stress test and step
    response."""
    model = mujoco.MjModel.from_xml_path(str(elrobot_mjcf_path))
    data = mujoco.MjData(model)
    return model, data
```

- [ ] **Step 2: Run existing tests — verify non-breaking**

Run: `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -q`
Expected: `7 passed, 1 skipped` (unchanged)

---

### Task 2: Write the pure-mujoco acceptance test suite

**Files:**
- Create: `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_elrobot_acceptance.py`

The original file is `software/sim-server/tests/integration/test_elrobot_acceptance.py` (205 lines, 5 functions, 13 collected items when parametrized). The rewrite is mechanical — see mapping table above.

Key differences from original:
- No `try/except _OK` import guard (mujoco is a hard dep in this package)
- No `from norma_sim.world.model import MuJoCoWorld` — zero norma\_sim imports
- Uses `elrobot_sim` fixture instead of `elrobot_scene_yaml` + `MuJoCoWorld.from_manifest_path()`
- Drops the dead `elrobot_world` fixture (defined at original:113-117, never used by any test)
- Floor 5 delegation docstring updated: path changed from `software/sim-server/tests/world/test_mimic_gripper.py` to `tests/test_mimic_gripper.py` (package-relative, since both files now live in the same package)

- [ ] **Step 1: Create the test file**

```python
"""Engine-tier physics acceptance: 6 Floor criteria for ElRobot smoothness.

Pure-mujoco rewrite of the original sim-server integration test
(software/sim-server/tests/integration/test_elrobot_acceptance.py).
Uses raw mujoco.MjModel / mujoco.MjData — zero norma_sim imports.
The manifest-layer coverage dropped by this rewrite is preserved by the
sentinel test at
software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py.

Criteria (MVP-2 spec S3.1):
  Floor 1 — no self-collision at rest
  Floor 2 — effective inertia floor (M[i,i] + armature >= 1e-4)
  Floor 3 — 10000-step stress with no NaN
  Floor 4 — per-motor step response (0.9 x ctrl_hi reached in 2s, overshoot <= 15%)
  Floor 5 — P0 mimic gripper regression (delegated to tests/test_mimic_gripper.py)
  Floor 6 — MVP-1 test suite still green (delegated to make sim-test)
"""
from __future__ import annotations

import numpy as np
import pytest

import mujoco


# ----------------------------------------------------------------------
# Floor 1 — no self-collision at rest
# ----------------------------------------------------------------------

def test_elrobot_no_self_collision_at_rest(elrobot_sim):
    """mj_forward at home pose should produce zero contacts.
    This catches URDF mesh overlap issues that MVP-1 had."""
    model, data = elrobot_sim
    mujoco.mj_forward(model, data)
    assert data.ncon == 0, (
        f"ElRobot should have clean collision at rest, got "
        f"{data.ncon} contacts. MVP-2 spec S3.1 Floor 1."
    )


# ----------------------------------------------------------------------
# Floor 2 — effective inertia floor
# ----------------------------------------------------------------------

def test_elrobot_effective_inertia_floor(elrobot_sim):
    """Every DOF should have M[i,i] + armature[i] >= 1e-4 kg*m^2.
    This ensures no joint is numerically ill-conditioned (MVP-1's
    gripper primary joint had 2.5e-7 — catastrophic)."""
    model, data = elrobot_sim
    mujoco.mj_forward(model, data)
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, M, data.qM)
    failures = []
    for i in range(model.nv):
        effective = M[i, i] + model.dof_armature[i]
        if effective < 1e-4:
            joint_id = None
            for j in range(model.njnt):
                if model.jnt_dofadr[j] == i:
                    joint_id = j
                    break
            joint_name = (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                if joint_id is not None else f"dof{i}"
            )
            failures.append(
                f"DOF {i} ({joint_name}): M[i,i]={M[i,i]:.2e}, "
                f"armature={model.dof_armature[i]:.2e}, "
                f"total={effective:.2e}"
            )
    assert not failures, (
        "Spec S3.1 Floor 2 failures (effective inertia < 1e-4 kg*m^2):\n"
        + "\n".join(failures)
    )


# ----------------------------------------------------------------------
# Floor 3 — stress test: 10000 random-ctrl steps, no NaN
# ----------------------------------------------------------------------

def test_elrobot_stress_10000_random_steps_no_nan(elrobot_sim):
    """10000 random-ctrl steps (resample every 100), verify qpos stays finite."""
    model, data = elrobot_sim
    rng = np.random.default_rng(42)
    lo = model.actuator_ctrlrange[:, 0]
    hi = model.actuator_ctrlrange[:, 1]
    for step in range(10000):
        if step % 100 == 0:
            data.ctrl[:] = rng.uniform(lo, hi)
        mujoco.mj_step(model, data)
        if step % 1000 == 0:
            assert np.isfinite(data.qpos).all(), (
                f"NaN at step {step}. Spec S3.1 Floor 3."
            )
    assert np.isfinite(data.qpos).all()
    assert np.isfinite(data.qvel).all()


# ----------------------------------------------------------------------
# Floor 4 — per-motor step response
# ----------------------------------------------------------------------

@pytest.mark.parametrize("motor_idx", range(8))
def test_elrobot_motor_step_response(elrobot_sim, motor_idx: int):
    """Drive motor motor_idx to 0.9 x ctrlrange_hi. Verify:
    - qpos reaches >= 80% of target within 2s (1000 steps @ dt=0.002)
    - overshoot <= 15% of the target value

    Parametrized so failure messages specify which motor is broken.
    Spec S3.1 Floor 4."""
    model, data = elrobot_sim
    assert 0 <= motor_idx < model.nu, (
        f"motor_idx {motor_idx} out of range; ElRobot has {model.nu} actuators"
    )

    ctrl_hi = float(model.actuator_ctrlrange[motor_idx, 1])
    target = 0.9 * ctrl_hi
    if abs(target) < 1e-9:
        target = 0.5 * ctrl_hi
        if abs(target) < 1e-9:
            pytest.skip(f"motor {motor_idx} has zero ctrl range, cannot step-response test")

    joint_id = int(model.actuator_trnid[motor_idx, 0])
    qadr = int(model.jnt_qposadr[joint_id])
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    actuator_name = mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_idx
    )

    data.ctrl[motor_idx] = target
    max_reached = 0.0
    for _ in range(1000):  # 2.0 sec at dt=0.002
        mujoco.mj_step(model, data)
        q = float(data.qpos[qadr])
        if target > 0 and q > max_reached:
            max_reached = q
        if target < 0 and q < max_reached:
            max_reached = q

    final = float(data.qpos[qadr])
    reached_fraction = final / target if abs(target) > 1e-9 else 0.0
    overshoot = (
        abs(max_reached - target) / abs(target)
        if abs(target) > 1e-9 and abs(max_reached) > abs(target)
        else 0.0
    )

    assert reached_fraction >= 0.8, (
        f"Motor {motor_idx} ({actuator_name}, joint {joint_name}) "
        f"only reached {final:.4f}/{target:.4f} = {reached_fraction:.1%} "
        f"within 2.0s. Spec S3.1 Floor 4 requires >= 80%. "
        f"Tune kp up or armature down for this joint in elrobot_follower.xml."
    )
    assert overshoot <= 0.15, (
        f"Motor {motor_idx} ({actuator_name}) overshot by {overshoot:.1%} "
        f"(max={max_reached:.4f}, target={target:.4f}). "
        f"Spec S3.1 Floor 4 requires <= 15%. Tune kv up or add joint damping."
    )


# ----------------------------------------------------------------------
# Floor 5 — P0 mimic gripper regression is delegated to test_mimic_gripper.py
# ----------------------------------------------------------------------

def test_floor_5_delegation_note():
    """Floor 5 (P0 gripper mimic) is covered by
    tests/test_mimic_gripper.py in this package. This stub exists only
    to make the Floor 5 intent visible in this file.
    Running pytest tests/test_mimic_gripper.py must show 2 PASSED."""
    pass


# ----------------------------------------------------------------------
# Floor 6 — full MVP-1 test suite still green is delegated to make sim-test
# ----------------------------------------------------------------------

def test_floor_6_delegation_note():
    """Floor 6 (MVP-1 test suite still green) is covered by
    make sim-test running in CI / locally. This stub exists only to
    make the Floor 6 intent visible in this file. The Chunk 7 gate
    runs make sim-test and asserts 0 failures."""
    pass
```

- [ ] **Step 2: Verify zero norma\_sim imports**

Run: `grep -c 'norma_sim' hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_elrobot_acceptance.py`
Expected: `0`

- [ ] **Step 3: Run new acceptance tests green**

Run: `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_elrobot_acceptance.py -v`
Expected: `13 passed` (1 Floor1 + 1 Floor2 + 1 Floor3 + 8 Floor4\[0-7\] + 1 Floor5 + 1 Floor6)

---

### Task 3: Write the manifest sentinel test

**Files:**
- Create: `software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py`

This test exercises the full `scene.yaml -> load_manifest -> MuJoCoWorld -> mj_step` pipeline. It is the coverage backstop: the pure-mujoco rewrite intentionally skips the manifest layer, and without this sentinel, regressions in scene.yaml parsing, `actuator_annotations` consistency, `mjcf_path` resolution, and the `GRIPPER_PARALLEL` capability assignment would go undetected.

Uses the existing `elrobot_scene_yaml` fixture from `software/sim-server/tests/conftest.py:49-56`.

- [ ] **Step 1: Create the sentinel test file**

```python
"""Manifest-pipeline sentinel for the ElRobot engine package.

The engine-tier acceptance tests (in mujoco/elrobot_follower/tests/) use
raw mujoco.MjModel.from_xml_path(...) and intentionally skip the manifest
layer. This sentinel exercises the full pipeline:

    scene.yaml -> load_manifest -> MuJoCoWorld -> mj_step

It catches manifest-layer regressions that the engine-tier suite cannot
see — specifically: scene.yaml parsing, world_name binding,
actuator_annotations consistency check, and the GRIPPER_PARALLEL
capability assignment for act_motor_08.
"""
from __future__ import annotations

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


def test_elrobot_manifest_pipeline_sentinel(elrobot_scene_yaml):
    """Smoke: scene.yaml -> load_manifest -> MuJoCoWorld pipeline still
    works end-to-end for the elrobot package.  Catches manifest-layer
    regressions that the engine-tier acceptance suite (now in
    mujoco/elrobot_follower/tests/) cannot see — specifically:
    scene.yaml parsing, world_name binding, actuator_annotations
    consistency check, and the GRIPPER_PARALLEL capability assignment
    for act_motor_08."""
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    mujoco.mj_step(world.model, world.data)

    # Manifest parsing actually happened (not just MJCF load)
    assert world.manifest.world_name == "elrobot_follower"

    # MJCF compile + lookup cache built correctly
    assert world.model.nu == 8

    # actuator_annotations were applied — without these explicit
    # annotations, load_manifest auto-synthesizes act_motor_08 as a
    # plain REVOLUTE_POSITION (manifest.py:187), so this assertion
    # fails if the gripper annotation is silently dropped from the
    # scene.yaml or from load_manifest's annotation merge.
    gripper = world.actuator_by_mjcf_name("act_motor_08")
    assert gripper is not None
    assert gripper.capability.kind == "GRIPPER_PARALLEL"
    assert gripper.gripper is not None
```

- [ ] **Step 2: Run sentinel test green**

Run: `pytest software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py -v`
Expected: `1 passed`

---

### Task 4: Fix Chunk 0 remainder — unicode glyph

**Files:**
- Modify: `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mimic_gripper.py:1`

- [ ] **Step 1: Replace unicode glyph**

Change line 1 from:
```
"""★ P0: pytest version of the Chunk 1 Task 1.7 MJCF demo.
```
to:
```
"""[P0] pytest version of the Chunk 1 Task 1.7 MJCF demo.
```

- [ ] **Step 2: Verify mimic gripper tests still pass**

Run: `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mimic_gripper.py -v`
Expected: `2 passed`

---

### Task 5: Verify Commit 1 gate + commit

- [ ] **Step 1: Full `make sim-test` with both suites coexisting**

Run: `make sim-test 2>&1 | tail -5`
Expected: `107 passed, 1 skipped` (baseline 93 + 13 new acceptance + 1 sentinel = +14)

- [ ] **Step 2: Side-by-side coexistence smoke check**

Run: `pytest software/sim-server/tests/integration/test_elrobot_acceptance.py hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_elrobot_acceptance.py -v 2>&1 | tail -5`
Expected: `26 passed` (13 old + 13 new — same physics, different entry points)

- [ ] **Step 3: Commit 1**

```bash
git add \
  hardware/elrobot/simulation/mujoco/elrobot_follower/tests/conftest.py \
  hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_elrobot_acceptance.py \
  hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mimic_gripper.py \
  software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py
git commit -m "$(cat <<'EOF'
mvp3-c3-add: pure-mujoco acceptance suite + manifest sentinel

Add 13-item engine-tier acceptance test (Floors 1-6) at
mujoco/elrobot_follower/tests/test_elrobot_acceptance.py using raw
mujoco APIs — zero norma_sim imports.

Add manifest-pipeline sentinel at
software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py
to preserve the manifest-layer coverage the rewrite drops.

Fix Chunk 0 remainder Item 4: unicode star -> [P0] ASCII in
test_mimic_gripper.py.

Both old and new acceptance suites coexist and pass (107 passed, 1 skipped).
Old file untouched — deletion in next commit.

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Chunk 2: Commit 2 — Delete Old + Cleanup + Metadata

### Task 6: Delete old acceptance test + audit fixtures

**Files:**
- Delete: `software/sim-server/tests/integration/test_elrobot_acceptance.py`
- Possibly modify: `software/sim-server/tests/conftest.py` (dead fixture cleanup)

- [ ] **Step 1: Grep for fixture consumers before deleting**

Run: `grep -rn 'elrobot_mjcf_path\|elrobot_scene_yaml' software/sim-server/tests/`

Expected results after the old acceptance file is deleted:
- `elrobot_scene_yaml`: used by `test_elrobot_manifest_sentinel.py` (keeping) -> **keep fixture**
- `elrobot_mjcf_path`: check if any remaining consumer exists -> if zero, **delete from conftest**

(Note: the old acceptance test used `elrobot_scene_yaml`, not `elrobot_mjcf_path`. So `elrobot_mjcf_path` may already have zero consumers. Verify with grep.)

- [ ] **Step 2: Delete the old acceptance test file**

```bash
rm software/sim-server/tests/integration/test_elrobot_acceptance.py
```

- [ ] **Step 3: Delete dead fixtures from conftest (if any)**

If grep confirms `elrobot_mjcf_path` has zero remaining consumers in `software/sim-server/tests/`:
Remove the `elrobot_mjcf_path` fixture (lines 39-45 of `software/sim-server/tests/conftest.py`):

```python
# DELETE this block if no consumers remain:
@pytest.fixture
def elrobot_mjcf_path(repo_root: Path) -> Path:
    """Path to the hand-written ElRobot MJCF (Chunk 5 artifact).
    Skipped during Chunks 2-4."""
    p = repo_root / "hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml"
    if not p.exists():
        pytest.skip(f"ElRobot MJCF not found at {p}; run Chunk 5 first")
    return p
```

If `elrobot_scene_yaml` has consumers (the sentinel), **keep it**.

- [ ] **Step 4: Verify sim-server tests pass after deletion**

Run: `pytest software/sim-server/tests/ -q 2>&1 | tail -3`
Expected: previous sim-server count minus 12 (−13 old acceptance + 1 sentinel)

---

### Task 7: Bump metadata

**Files:**
- Modify: `hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION`
- Modify: `hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md`
- Modify: `hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml`
- Modify: `hardware/elrobot/simulation/mujoco/elrobot_follower/README.md`

- [ ] **Step 1: Bump VERSION**

Change `0.2.1` to `0.2.2`.

- [ ] **Step 2: Add CHANGELOG entry**

Replace:
```markdown
## [Unreleased]

(nothing yet)
```

With:
```markdown
## [0.2.2] — 2026-04-12

### Added

- `tests/test_elrobot_acceptance.py` — 13-item physics acceptance suite
  (Floors 1-6) rewritten from the sim-server integration test to use raw
  `mujoco.MjModel` / `mujoco.MjData` APIs. Zero `norma_sim` imports.
  This is the capstone of MVP-3 Engine Package Completion: the package
  now has full physics-acceptance coverage runnable in isolation.
- `software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py`
  (in sim-server, not this package) — 1-test sentinel exercising the full
  `scene.yaml -> load_manifest -> MuJoCoWorld -> mj_step` pipeline.
  Preserves manifest-layer coverage that the pure-mujoco rewrite
  intentionally drops.

### Changed

- `tests/conftest.py`: added `elrobot_sim` fixture (function-scoped
  `MjModel + MjData` pair) shared by the 13 acceptance tests.
- `tests/test_mimic_gripper.py:1`: `★ P0` -> `[P0]` ASCII (Chunk 0
  remainder Item 4).
- `README.md`: rewrote "Relationship to NormaCore" section — engine-tier
  acceptance tests now live in this package; sim-server retains only the
  manifest-pipeline sentinel and Norma-specific integration tests.

### Removed

- `software/sim-server/tests/integration/test_elrobot_acceptance.py`
  (from sim-server, not this package) — the original norma\_sim-coupled
  version. All 13 physics assertions are preserved in the new engine-tier
  file; manifest-layer coverage is preserved by the sentinel.

### Physics gate results (at this version)

- Floor S3.1 acceptance gate: GREEN (no physics changes; same as v0.2.1).
- Engine-tier package tests: 20 passed + 1 skipped (mjx if absent).
- `cp -r /tmp` self-containment: 20 passed + 1 skipped.

### Integration context

- NormaCore main HEAD before this version: `6d4ddb4` on main
  (2026-04-12, Chunk 2 commit immediately preceding Chunk 3)
- MVP-3 Engine Package Completion Chunk 3 commits: (this commit pair)
- Roadmap spec: `docs/superpowers/specs/2026-04-12-mvp3-foundation-roadmap-design.md`
```

- [ ] **Step 3: Update robot.yaml**

Change `version.current` from `"0.2.1"` to `"0.2.2"`.

In `upstream.prerequisites`, change:
```yaml
    - Passing tests executable in isolation (no norma_sim dependency)
```
to:
```yaml
    - Passing tests executable in isolation (done at 0.2.2)
```

- [ ] **Step 4: Update README.md**

**4a.** Update the file tree in the Structure section. Replace:
```
└── tests/                   ← engine-level validation
    ├── conftest.py          ← single fixture (elrobot_mjcf_path)
    ├── test_mimic_gripper.py    ← P0 gripper mimic regression
    ├── test_urdf_parity.py      ← URDF↔MJCF consistency gate
    └── test_mjx_compat.py       ← MJX smoke test (placeholder)
```

With:
```
└── tests/                   ← engine-level validation
    ├── conftest.py              ← shared fixtures (elrobot_mjcf_path, elrobot_sim)
    ├── test_elrobot_acceptance.py ← physics acceptance (Floors 1-6, pure mujoco)
    ├── test_mimic_gripper.py    ← P0 gripper mimic regression
    ├── test_scene_loadable.py   ← scene.xml smoke gate
    ├── test_urdf_parity.py      ← URDF↔MJCF consistency gate
    └── test_mjx_compat.py       ← MJX smoke test (placeholder)
```

**4b.** Replace the "Relationship to NormaCore" section. Replace:
```markdown
## Relationship to NormaCore

The Norma-specific runtime wrapper for this robot lives at
`hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`.
That wrapper — not this directory — is what `norma_sim`'s loader reads at
runtime. This directory contains only engine-native files.

The `software/sim-server/tests/integration/test_elrobot_acceptance.py`
integration test still lives in the sim-server test tree because it
imports `norma_sim.world.MuJoCoWorld`. Pure-MuJoCo tests that do not need
`norma_sim` live here under `tests/`.
```

With:
```markdown
## Relationship to NormaCore

This package contains the complete engine-tier test suite for the ElRobot
follower arm, including physics-acceptance tests (Floors 1-6 from MVP-2
spec S3.1). All tests use raw `mujoco` APIs and run without `norma_sim`
on PYTHONPATH.

The Norma application layer still maintains:
- `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`
  — the Norma-specific runtime wrapper that maps MJCF actuator names
  (`act_motor_*`) to client-facing IDs (`rev_motor_*`) with capability
  annotations.
- `software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py`
  — a single sentinel test exercising the full
  `scene.yaml -> load_manifest -> MuJoCoWorld -> mj_step` pipeline.
- `software/sim-server/tests/integration/test_full_loop.py` and other
  Norma-specific integration tests that depend on `norma_sim`.
```

---

### Task 8: Verify all success criteria + commit

**Files:** none (verification), then commit

Spec success criteria (Section 5, lines 614-650), verified one by one:

- [ ] **Step 1: SC#1 — Physics-layer coverage equivalence**

The side-by-side audit was done at HEAD-of-commit-1 (Task 5, Step 2). 13 test functions exist in the new location. Assertion equivalence confirmed by the 26-passed run.

- [ ] **Step 2: SC#2 — Manifest-layer coverage preserved**

Run: `pytest software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py -v`
Expected: `1 passed`

- [ ] **Step 3: SC#3 — Package acceptance runs without PYTHONPATH**

Run: `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_elrobot_acceptance.py -v`
Expected: `13 passed`

- [ ] **Step 4: SC#4 — Zero norma\_sim imports in new acceptance**

Run: `grep -c 'norma_sim' hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_elrobot_acceptance.py`
Expected: `0`

- [ ] **Step 5: SC#5 — make sim-test final count**

Run: `make sim-test 2>&1 | tail -3`
Expected: `94 passed, 1 skipped` (baseline 93 + 1 sentinel net)

- [ ] **Step 6: SC#6 — Full self-containment**

Run: `cp -r hardware/elrobot/simulation/mujoco/elrobot_follower /tmp/elrobot-test && cd /tmp/elrobot-test && python3 -m pytest tests/ -v 2>&1; cd /home/yuan/proj/norma-core && rm -rf /tmp/elrobot-test`
Expected: `20 passed, 1 skipped` (mimic=2, urdf=2, scene=3, acceptance=13, mjx=1skip)

- [ ] **Step 7: SC#7 — Sim-server cleanup**

Run: `test ! -f software/sim-server/tests/integration/test_elrobot_acceptance.py && echo "old file deleted OK"`
Run: `test -f software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py && echo "sentinel exists OK"`

- [ ] **Step 8: SC#8 — Architecture invariants**

Run: `make check-arch-invariants`
Expected: `All architecture invariants hold`

- [ ] **Step 9: SC#9 — Phase G.8 grep (no stale references)**

Run: `grep -rn 'test_elrobot_acceptance' software/ hardware/ Makefile docs/ | grep -v 'docs/superpowers/.*2026-04-1[012]' | grep -v 'vendor/menagerie/VENDOR.md'`
Expected: only matches the NEW package path (`hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_elrobot_acceptance.py`), not the deleted sim-server path.

Note: the CHANGELOG.md inside the package will also match (it mentions the old path in the Removed section) — this is expected and acceptable since it's historical documentation.

- [ ] **Step 10: SC#10 — git status clean**

Run: `git status`
Expected: only the files staged for commit 2.

- [ ] **Step 11: Commit 2**

```bash
git add \
  software/sim-server/tests/integration/test_elrobot_acceptance.py \
  software/sim-server/tests/conftest.py \
  hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION \
  hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md \
  hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml \
  hardware/elrobot/simulation/mujoco/elrobot_follower/README.md
git commit -m "$(cat <<'EOF'
mvp3-c3-del: delete old acceptance test + bump metadata to 0.2.2

Delete software/sim-server/tests/integration/test_elrobot_acceptance.py
(13 physics tests now live in engine package at
mujoco/elrobot_follower/tests/test_elrobot_acceptance.py).
Manifest-layer coverage preserved by sentinel.

Clean up dead elrobot_mjcf_path fixture from sim-server conftest
(elrobot_scene_yaml kept — used by sentinel).

Bump VERSION 0.2.1 -> 0.2.2. Update CHANGELOG, robot.yaml (mark
'tests in isolation' upstream prereq done), README.

make sim-test: 94 passed, 1 skipped (baseline + 1).
Self-contained: cp -r /tmp -> 20 passed, 1 skipped.
MVP-3 Engine Package Completion: done.

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```
