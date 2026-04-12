# MVP-3 Chunk 2: `scene.xml` Menagerie-Style Wrapper — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml` — a Menagerie-style MJCF wrapper containing `<visual>`, `<statistic>`, `<asset>`, and `<worldbody>` blocks (lights + floor + groundplane) that `<include>`s the main `elrobot_follower.xml`. Add a smoke test `tests/test_scene_loadable.py` that loads the wrapper and asserts name-based invariants. Bump the package VERSION to `0.2.1`. Bundle 2 small polish backfills carried over from Chunk 1: backfill `<TBD-mvp3-chunk1>` placeholder in `README.md:38` with the actual Chunk 1 SHA `51ecccd`, and fix the stale `# No scene_extras — the MJCF has its own lighting/floor setup.` comment in `manifests/norma/elrobot_follower.scene.yaml:12` (the MJCF doesn't actually have lighting/floor — that's Chunk 2's whole point). All in **one atomic commit on `main`**.

**Architecture:** This is Chunk 2 of MVP-3 Engine Package Completion (3 chunks total). Chunk 2 is the **smallest chunk** in the milestone — pure additive content (1 new MJCF + 1 new smoke test) plus metadata bumps and 2 polish backfills. Per the roadmap spec Section 6, Chunk 1 → Chunk 2 is a **soft prerequisite** (Chunk 2's content is independent of Chunk 1, but reverse-ordering produces an intermediate state where the cp -r self-containment audit is delayed). After Chunk 2 lands, running `python3 -m mujoco.viewer hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml` opens an ElRobot visualization with floor and lights — the upstream/visualization ergonomics half of MVP-3 EPC is done. The other half (test decoupling) waits for Chunk 3.

**Tech Stack:** MuJoCo `<include>` namespace merging (textual merge, not import); MuJoCo builtin texture engine (`builtin="checker"`, `builtin="gradient"`); Python 3 + `mujoco` (compile + name-id lookup verification); pytest (smoke test); Edit + Write tools (in-place file edits and new file creation, NEVER `sed`/`awk`).

---

## Reference Documents (READ FIRST)

Before starting, the implementer MUST read:

1. **The roadmap spec, Section 4 only** — `/home/yuan/proj/norma-core/docs/superpowers/specs/2026-04-12-mvp3-foundation-roadmap-design.md` (lines ~280–410). This is the design source of truth for Chunk 2. Sections 1, 2, 3, 5, 6, 7, 8, 9 are out of scope for this implementation.
2. **The vendor Menagerie reference** — `/home/yuan/proj/norma-core/hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml` (24 lines, the canonical Menagerie convention this chunk's `scene.xml` is modeled after).
3. **The current package state** — list `/home/yuan/proj/norma-core/hardware/elrobot/simulation/mujoco/elrobot_follower/`. After Chunk 1, the package contains: `assets/` (19 STL), `elrobot_follower.urdf`, `elrobot_follower.xml`, `README.md`, `CHANGELOG.md`, `VERSION` (0.2.0), `LICENSE`, `robot.yaml`, `measurements/`, `tests/`. Chunk 2 adds `scene.xml` and `tests/test_scene_loadable.py` and edits 5 existing files.

---

## Pre-flight Grep Results (baked in by plan author 2026-04-12, re-verified at execution time)

Chunk 2 doesn't move/rename any path, so the (α) "grep first" rule from spec Section 7 doesn't apply to old-path scanning. However, two **placeholder backfills** carried over from Chunk 1 need grep to enumerate:

**Scan 1: `<TBD-mvp3-chunk1>` placeholder backfill** (Chunk 2's polish duty per Chunk 1 plan's "Note for Chunk 2 plan author" at line 1134)

```bash
grep -rn '<TBD-mvp3-chunk1>' software/ hardware/ Makefile docs/ 2>&1 | grep -v 'docs/superpowers/'
```

Expected output (at plan-write time):
```
hardware/elrobot/simulation/mujoco/elrobot_follower/README.md:38:Chunk 1, commit `<TBD-mvp3-chunk1>`). The MJCF's `meshdir="assets"`
```

(1 hit, in the package's own README.md. Chunk 2 backfills with `51ecccd` — the Chunk 1 commit SHA.)

**Scan 2: stale comment about MJCF lighting/floor** (the spec Section 4 polish item)

```bash
grep -rn 'MJCF has its own lighting\|MJCF has its own .*floor' software/ hardware/ Makefile docs/ 2>&1 | grep -v 'docs/superpowers/'
```

Expected output (at plan-write time):
```
hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml:12:# No scene_extras — the MJCF has its own lighting/floor setup.
```

(1 hit, in the Norma scene yaml. Chunk 2 fixes.)

**Critical**: if either grep returns hits OUTSIDE the expected list at execution time, STOP and escalate (NEEDS_CONTEXT). Don't silently extend the file list.

---

## Starting State Verification

Before beginning Phase A, confirm:

```bash
cd /home/yuan/proj/norma-core
git branch --show-current   # must print: main
git log --oneline -1         # HEAD must be 0ddeb60 (Chunk 1 polish commit) or 51ecccd (Chunk 1) or newer
git status --short           # must be empty (clean tree) OR only have expected untracked
```

If any of these fail, STOP and investigate. Do NOT proceed.

---

## File Structure Overview

**What exists today** (at `hardware/elrobot/simulation/mujoco/elrobot_follower/`, after Chunks 0+1):

```
hardware/elrobot/simulation/mujoco/elrobot_follower/
├── assets/                                  (19 STL, in-package since Chunk 1)
├── elrobot_follower.urdf                    (in-package since Chunk 1)
├── elrobot_follower.xml                     (meshdir="assets", since Chunk 1)
├── README.md                                ← will EDIT (Structure tree + How to view + TBD backfill)
├── CHANGELOG.md                             ← will EDIT (add [0.2.1] entry)
├── VERSION                                  ← will EDIT (0.2.0 → 0.2.1)
├── LICENSE                                  (UNCHANGED)
├── robot.yaml                               ← will EDIT (version bump + prereq mark done)
├── measurements/{README.md, menagerie_diff.md}  (UNCHANGED)
└── tests/
    ├── conftest.py                          (UNCHANGED)
    ├── test_mimic_gripper.py                (UNCHANGED)
    ├── test_urdf_parity.py                  (UNCHANGED)
    └── test_mjx_compat.py                   (UNCHANGED)
```

Plus one file in the sibling Norma manifests directory:

```
hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml  ← will EDIT (line 12 stale comment fix)
```

**What the end state looks like** (after this chunk):

```
hardware/elrobot/simulation/mujoco/elrobot_follower/
├── assets/                                  (UNCHANGED)
├── elrobot_follower.urdf                    (UNCHANGED)
├── elrobot_follower.xml                     (UNCHANGED — zero physics drift)
├── scene.xml                                ← NEW (~30 lines, Menagerie-style wrapper)
├── README.md                                (Structure section adds scene.xml; "How to view" snippet added; TBD backfilled)
├── CHANGELOG.md                             (new [0.2.1] entry added on top)
├── VERSION                                  (0.2.1)
├── LICENSE                                  (UNCHANGED)
├── robot.yaml                               (version.current: "0.2.1"; upstream.prerequisites scene.xml line marked done)
├── measurements/                            (UNCHANGED)
└── tests/
    ├── conftest.py                          (UNCHANGED)
    ├── test_mimic_gripper.py                (UNCHANGED)
    ├── test_urdf_parity.py                  (UNCHANGED)
    ├── test_mjx_compat.py                   (UNCHANGED)
    └── test_scene_loadable.py               ← NEW (~40 lines, smoke test)

hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml  (line 12 comment fixed)
```

**Total operations**: 2 new files (scene.xml + test_scene_loadable.py) + 5 in-place edits (README, CHANGELOG, VERSION, robot.yaml, manifests/norma/elrobot_follower.scene.yaml) = **7 file operations**, all committed atomically.

---

## Execution Approach

This plan has **ONE task** that performs all 7 operations and commits atomically. The task has ~22 bite-sized steps organized into 6 phases (A through F). Do **NOT** commit partway through — the entire chunk is one atomic unit.

If anything fails mid-way, run the rollback:

```bash
cd /home/yuan/proj/norma-core
git restore --staged .
git restore .
rm -f hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml \
      hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_scene_loadable.py
```

Then investigate the root cause before re-attempting Phase A.

The single-task atomic-commit structure matches Chunks 0 and 1's verified successful pattern.

---

## Chunk 2: Atomic Add

### Task 1: Add `scene.xml` + smoke test, bump VERSION, fold in 2 polish backfills, commit atomically

**Files this task touches** (high-level — exact line numbers in step content):

- **Create**:
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml` (new file, ~30 lines)
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_scene_loadable.py` (new file, ~40 lines)

- **Edit (in-place)**:
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/README.md` (3 sub-edits: Structure tree, "How to view" snippet, TBD backfill at line 38)
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md` (1 sub-edit: add `[0.2.1]` entry above `[0.2.0]`)
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION` (`0.2.0` → `0.2.1`)
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml` (2 sub-edits: `version.current` bump + `upstream.prerequisites` scene.xml line marked done)
  - `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml` (line 12 stale comment fix)

---

#### Phase A: Pre-flight verification + grep re-validation

- [ ] **Step A.1: Verify the starting state is clean**

```bash
cd /home/yuan/proj/norma-core
git branch --show-current
git status --short
git log --oneline -3
```

Expected:
- Branch: `main`
- Status: clean (no modified files; untracked files like `MUJOCO_LOG.TXT` or `station_data/` are allowed)
- HEAD: `0ddeb60` (Chunk 1 polish commit) or `51ecccd` (Chunk 1) or newer

If not clean, STOP. Investigate before proceeding.

- [ ] **Step A.2: Capture the current `make sim-test` baseline**

```bash
make sim-test 2>&1 | tail -3
```

Capture the two numbers from the output line `N passed, M skipped in Xs`. Store these locally as `BASELINE_PASSED` and `BASELINE_SKIPPED` (write them down — they will be used in Phase D success criteria).

Expected at plan-write time: `90 passed, 1 skipped` (post-Chunk-1 baseline; Chunks 0 and 1 didn't change the test count).

If `make sim-test` fails, STOP. The pre-Chunk-2 baseline is broken and must be fixed before this chunk.

- [ ] **Step A.3: Verify Chunk 1's end-state is intact**

```bash
ls hardware/elrobot/simulation/mujoco/elrobot_follower/assets/ | wc -l   # expected: 19
ls hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.urdf  # exists
grep 'meshdir' hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml  # meshdir="assets"
cat hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION  # 0.2.0
```

Expected:
- 19 STL files in `assets/`
- URDF present in package
- MJCF has `meshdir="assets"` (Chunk 1 simplification)
- VERSION is `0.2.0`

If any of these fails, Chunk 1 wasn't fully applied. STOP and investigate.

- [ ] **Step A.4: Re-run the 2 placeholder backfill greps (drift check)**

```bash
grep -rn '<TBD-mvp3-chunk1>' software/ hardware/ Makefile docs/ 2>&1 | grep -v 'docs/superpowers/'
```

Expected output (must match exactly):
```
hardware/elrobot/simulation/mujoco/elrobot_follower/README.md:38:Chunk 1, commit `<TBD-mvp3-chunk1>`). The MJCF's `meshdir="assets"`
```

```bash
grep -rn 'MJCF has its own lighting\|MJCF has its own .*floor' software/ hardware/ Makefile docs/ 2>&1 | grep -v 'docs/superpowers/'
```

Expected output (must match exactly):
```
hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml:12:# No scene_extras — the MJCF has its own lighting/floor setup.
```

**Critical**: if either grep returns ANY hit not in the expected list, STOP and escalate (NEEDS_CONTEXT). New hits would mean either someone added a new placeholder/stale-comment between plan-write time and now, OR the plan author missed a hit. Either way, the plan needs to be amended before proceeding.

- [ ] **Step A.5: Verify the vendor Menagerie reference scene.xml exists**

```bash
ls hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml
wc -l hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml
```

Expected: file exists, ~24 lines. This is the canonical reference whose `<visual>`, `<asset>`, and `<worldbody>` blocks are borrowed by the new `scene.xml`.

---

#### Phase B: Create new files (`scene.xml` + smoke test)

- [ ] **Step B.1: Create `mujoco/elrobot_follower/scene.xml`**

Use the Write tool to create this file at `/home/yuan/proj/norma-core/hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml`:

```xml
<mujoco model="elrobot_follower scene">
  <!-- Menagerie-style scene wrapper for ElRobot follower arm.
       Borrows the <visual>, <statistic>, and <asset> conventions from
       vendor/menagerie/trs_so_arm100/scene.xml so this package matches
       the upstream Menagerie convention.

       NO <compiler> block: would override the main MJCF's meshdir="assets"
       via <include> namespace merge (codex iter-1 risk #1).
       NO <default> block: would collide with the main MJCF's <default class="elrobot">.
       Only <include>, <statistic>, <visual>, <asset>, <worldbody>. -->

  <include file="elrobot_follower.xml"/>

  <statistic center="0 0 0.1" extent="0.5"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light name="floor_light" pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
```

**Notes on the design** (for the implementer's understanding):
- The `<include file="elrobot_follower.xml"/>` is a textual merge. Everything in the included MJCF is added to this scene's namespace. The included MJCF brings: `<compiler meshdir="assets">`, `<option>`, `<default class="elrobot">`, `<asset>` (mesh declarations + base material), `<worldbody>` (the robot body tree), `<contact><exclude>` pairs, `<actuator>`, `<tendon>`, `<equality>`.
- `scene.xml`'s own `<asset>` adds the skybox texture, groundplane texture, and groundplane material. None of these names (`groundplane`) collide with the main MJCF's asset names (which are mesh-derived: `Joint_01_1`, `ST3215_1_v1_1`, etc.).
- `scene.xml`'s own `<worldbody>` adds the directional light (named `floor_light` for the smoke test) and the floor geom (named `floor` for the smoke test). The main MJCF's `<worldbody>` is also merged in, bringing the robot body tree under the same world frame.
- The `<statistic>` `center="0 0 0.1" extent="0.5"` is centered on the ElRobot's roughly-at-origin base with a 0.5m visualization extent. (Vendor menagerie uses `center="0.1 -0.01 0.05" extent="0.5"` for trs_so_arm100; we adapt for ElRobot's geometry.)
- The `<visual><global>` `azimuth="120" elevation="-20"` sets the default camera angle. Vendor menagerie uses `azimuth="45"`; 120° is a slightly different default that shows ElRobot's gripper-forward orientation better. (Both are valid; if user has a strong preference, can change.)
- All other `<visual>`/`<asset>` numerical values (headlight RGB, haze color, texture rgb1/rgb2, width/height, texrepeat, reflectance) are **copied verbatim from vendor menagerie** for cross-package consistency.

Verify:

```bash
ls hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml
wc -l hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml
grep -c '<compiler' hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml || echo "OK: no <compiler> block"
grep -c '<default' hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml || echo "OK: no <default> block"
grep -n '<include\|<light\|<geom name="floor"' hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml
```

Expected:
- File exists, ~30 lines
- `OK: no <compiler> block` (codex iter-1 risk #1 mitigation)
- `OK: no <default> block` (same)
- `<include file="elrobot_follower.xml"/>`, `<light name="floor_light" ...`, `<geom name="floor" ...` all present

- [ ] **Step B.2: Smoke-compile `scene.xml` from the command line**

Before writing the smoke test, manually verify the scene.xml compiles:

```bash
python3 -c "
import mujoco
m = mujoco.MjModel.from_xml_path('hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml')
print(f'nu={m.nu} ngeom={m.ngeom} nlight={m.nlight}')
print(f'floor geom id: {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, \"floor\")}')
print(f'floor_light id: {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, \"floor_light\")}')
"
```

Expected output:
```
nu=8 ngeom=<some number ≥ 1> nlight=<some number ≥ 1>
floor geom id: <some non-negative integer>
floor_light id: <some non-negative integer>
```

If `nu` is not 8, the `<include>` namespace merge failed. Investigate.
If the floor or floor_light id is `-1`, the named element is missing. Re-check `scene.xml`.

- [ ] **Step B.3: Create `tests/test_scene_loadable.py`**

Use the Write tool to create this file at `/home/yuan/proj/norma-core/hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_scene_loadable.py`:

```python
"""Smoke test for scene.xml — Menagerie-style wrapper around elrobot_follower.xml.

This test loads the scene wrapper via mujoco.MjModel.from_xml_path and verifies
name-based invariants: actuator count must match the included main MJCF, the
floor geom must exist by name, and the directional light must exist by name.

Per spec Section 4 risk #3 / codex iter-1 reframe: this test uses
mj_name2id-based assertions, NOT count-based assertions like `m.ngeom == N`.
Count-based assertions are fragile to future geometry additions; name-based
assertions are stable across structural changes that don't affect the named
elements.
"""
from __future__ import annotations

from pathlib import Path

import mujoco
import pytest


@pytest.fixture
def scene_xml_path() -> Path:
    """Path to scene.xml inside this package."""
    p = Path(__file__).resolve().parent.parent / "scene.xml"
    assert p.exists(), (
        f"scene.xml not found at {p}. scene.xml is mandatory content of "
        f"this package after MVP-3 Chunk 2."
    )
    return p


def test_scene_xml_compiles_and_includes_main_mjcf(scene_xml_path: Path):
    """scene.xml must compile via from_xml_path and the <include> namespace
    merge must bring in the main MJCF's actuators (nu == 8)."""
    m = mujoco.MjModel.from_xml_path(str(scene_xml_path))
    assert m.nu == 8, (
        f"scene.xml's <include> should bring in the main MJCF's 8 actuators, "
        f"got nu={m.nu}. Check that <include file=\"elrobot_follower.xml\"/> "
        f"is present and the include path resolves correctly."
    )


def test_scene_xml_floor_geom_exists_by_name(scene_xml_path: Path):
    """The floor geom added by scene.xml must be discoverable by name. This
    is the name-based equivalent of asserting on geom count, which would be
    fragile to future geometry additions."""
    m = mujoco.MjModel.from_xml_path(str(scene_xml_path))
    floor_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    assert floor_id >= 0, (
        f"scene.xml should declare a <geom name=\"floor\"> in its <worldbody>, "
        f"got mj_name2id result {floor_id}. Check that the floor geom is "
        f"present and named 'floor'."
    )


def test_scene_xml_floor_light_exists_by_name(scene_xml_path: Path):
    """The directional light added by scene.xml must be discoverable by name."""
    m = mujoco.MjModel.from_xml_path(str(scene_xml_path))
    light_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, "floor_light")
    assert light_id >= 0, (
        f"scene.xml should declare a <light name=\"floor_light\"> in its "
        f"<worldbody>, got mj_name2id result {light_id}. Check that the "
        f"directional light is present and named 'floor_light'."
    )
```

Verify:

```bash
python3 -c "
import ast
tree = ast.parse(open('hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_scene_loadable.py').read())
funcs = [n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]
assert 'scene_xml_path' in funcs
assert 'test_scene_xml_compiles_and_includes_main_mjcf' in funcs
assert 'test_scene_xml_floor_geom_exists_by_name' in funcs
assert 'test_scene_xml_floor_light_exists_by_name' in funcs
print('test_scene_loadable.py syntax OK')
"
grep -c 'm.ngeom ==\|m.nlight ==' hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_scene_loadable.py || echo "OK: no count-based assertions"
grep -c 'mj_name2id' hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_scene_loadable.py
```

Expected:
- `test_scene_loadable.py syntax OK`
- `OK: no count-based assertions` (codex iter-1 mandate — only name-based)
- `mj_name2id` count is 2 (one for floor, one for floor_light)

- [ ] **Step B.4: Run the new smoke test**

```bash
PYTHONPATH= python3 -m pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_scene_loadable.py -v 2>&1 | tail -20
```

Expected: 3 passed (the 3 test functions). If any fail, the scene.xml or the smoke test has a bug. Fix before continuing.

---

#### Phase C: Edit existing files

Phase C performs the in-place content edits to (a) README.md (3 sub-edits), (b) CHANGELOG.md (1 sub-edit), (c) VERSION (1 line), (d) robot.yaml (2 sub-edits), (e) manifests/norma/elrobot_follower.scene.yaml (1 sub-edit). **Use the Edit tool, NOT `sed` / `awk` / heredoc redirection.**

- [ ] **Step C.1: Edit `mujoco/elrobot_follower/README.md` (3 sub-edits)**

**Edit C.1a (Structure section)**: Add `scene.xml` to the file tree.

In `hardware/elrobot/simulation/mujoco/elrobot_follower/README.md`:
- **Find**:
```
elrobot_follower/
├── elrobot_follower.xml     ← main MJCF (8 joints + 2 mimic slides)
├── elrobot_follower.urdf    ← URDF kinematic source of truth (used by test_urdf_parity)
├── robot.yaml               ← machine-readable identity (source of truth)
```
- **Replace with**:
```
elrobot_follower/
├── elrobot_follower.xml     ← main MJCF (8 joints + 2 mimic slides)
├── elrobot_follower.urdf    ← URDF kinematic source of truth (used by test_urdf_parity)
├── scene.xml                ← Menagerie-style wrapper with floor + lights (use with mujoco.viewer)
├── robot.yaml               ← machine-readable identity (source of truth)
```

**Edit C.1b (Backfill `<TBD-mvp3-chunk1>` placeholder — Chunk 2's polish duty)**:

In the same file:
- **Find**: `Chunk 1, commit `<TBD-mvp3-chunk1>`). The MJCF's `meshdir="assets"``
- **Replace with**: `Chunk 1, commit `51ecccd`). The MJCF's `meshdir="assets"``

(`51ecccd` is the actual Chunk 1 commit SHA, verified at plan-write time via `git log --oneline | grep 'mvp3-c1: move'`.)

**Edit C.1c (Add a "How to view" section after the package structure prose)**:

In the same file:
- **Find**:
```
The STL mesh assets live inside this package at `assets/` (moved in MVP-3
Chunk 1, commit `51ecccd`). The MJCF's `meshdir="assets"`
resolves to them. The package is now self-contained: it can be copied to
any location (e.g. `/tmp/elrobot-test`) and `pytest tests/` runs cleanly
without needing the rest of the NormaCore checkout on disk.

## How to modify
```
- **Replace with**:
```
The STL mesh assets live inside this package at `assets/` (moved in MVP-3
Chunk 1, commit `51ecccd`). The MJCF's `meshdir="assets"`
resolves to them. The package is now self-contained: it can be copied to
any location (e.g. `/tmp/elrobot-test`) and `pytest tests/` runs cleanly
without needing the rest of the NormaCore checkout on disk.

## How to view

To open an interactive 3D view of the ElRobot with floor + lights:

```bash
python3 -m mujoco.viewer hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml
```

The `scene.xml` is a Menagerie-style wrapper that `<include>`s the main
`elrobot_follower.xml` and adds a directional light and a checker
groundplane. Use `scene.xml` (not `elrobot_follower.xml`) for visual
inspection — the bare main MJCF has no lighting/floor.

For headless/CI use cases, the `tests/test_scene_loadable.py` smoke test
verifies `scene.xml` compiles correctly without requiring a display.

## How to modify
```

(Note: the heredoc inside markdown is a triple-backtick code block. Make sure the indentation and surrounding blank lines match.)

Verify all 3 sub-edits:

```bash
grep -n '├── scene.xml' hardware/elrobot/simulation/mujoco/elrobot_follower/README.md
grep -n '<TBD-mvp3-chunk1>' hardware/elrobot/simulation/mujoco/elrobot_follower/README.md || echo "OK: no TBD placeholder remaining"
grep -n '`51ecccd`' hardware/elrobot/simulation/mujoco/elrobot_follower/README.md
grep -n '## How to view' hardware/elrobot/simulation/mujoco/elrobot_follower/README.md
grep -n 'mujoco.viewer' hardware/elrobot/simulation/mujoco/elrobot_follower/README.md
```

Expected:
- First: 1 hit at the new `├── scene.xml` line
- Second: `OK: no TBD placeholder remaining`
- Third: 1 hit at the backfilled SHA
- Fourth: 1 hit at the new section header
- Fifth: 1 hit at the `python3 -m mujoco.viewer` snippet

- [ ] **Step C.2: Edit `mujoco/elrobot_follower/CHANGELOG.md` (1 sub-edit: add `[0.2.1]` entry)**

In `hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md`:
- **Find**: `## [0.2.0] — 2026-04-12`
- **Replace with**:
```
## [0.2.1] — 2026-04-12

### Added

- `scene.xml` — Menagerie-style scene wrapper that `<include>`s the main
  `elrobot_follower.xml` and adds:
  - `<visual>` (headlight, haze, default camera angle)
  - `<asset>` (skybox gradient texture, groundplane checker texture +
    material)
  - `<worldbody>` (directional `<light name="floor_light">` + `<geom
    name="floor">` plane)
  No `<compiler>` block (would override main MJCF's `meshdir="assets"`
  via `<include>` namespace merge — codex iter-1 risk #1 mitigation).
  No `<default>` block (would collide with main MJCF's
  `<default class="elrobot">`).
- `tests/test_scene_loadable.py` — 3-test smoke gate for `scene.xml`:
  asserts `nu == 8` (include namespace merge worked), asserts
  `mj_name2id(GEOM, "floor") >= 0`, asserts
  `mj_name2id(LIGHT, "floor_light") >= 0`. Per spec Section 4 risk #3 /
  codex iter-1 reframe, all assertions are name-based, NOT count-based
  (`m.ngeom == N` would be fragile to future geometry additions).

### Changed (polish backfills carried over from Chunk 1)

- `README.md`: backfilled `<TBD-mvp3-chunk1>` placeholder with the
  actual Chunk 1 commit SHA `51ecccd`. Added a `## How to view` section
  documenting `python3 -m mujoco.viewer scene.xml`.
- `manifests/norma/elrobot_follower.scene.yaml:12` (sibling Norma
  manifests directory, not in this package): fixed stale comment
  `# No scene_extras — the MJCF has its own lighting/floor setup.` —
  the MJCF does NOT have its own lighting/floor (that was an MVP-2
  Chunk 5 leftover comment from before MVP-3 EPC roadmap), now
  `scene.xml` provides them.

### Physics gate results (at this version)

- Floor §3.1 acceptance gate: GREEN (no physics changes; same as v0.2.0).
- Engine-tier package tests: 5 passed + 1 skipped (mjx if absent) — +1
  vs v0.2.0 from `test_scene_loadable.py`.
- `cp -r /tmp` self-containment: 5 passed + 1 skipped (mjx if absent)
  — `scene.xml` is in-package and `<include>` resolves relative to
  `scene.xml`'s directory.

### Integration context

- NormaCore main HEAD before this version: `0ddeb60` on main
  (2026-04-12, Chunk 1 polish + γ doc commit immediately preceding
  Chunk 2)
- MVP-3 Engine Package Completion Chunk 2 commit: (this commit)
- Roadmap spec: `docs/superpowers/specs/2026-04-12-mvp3-foundation-roadmap-design.md`

## [0.2.0] — 2026-04-12
```

Verify:

```bash
grep -c '^## \[0\.2\.1\]' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md
grep -c '^## \[0\.2\.0\]' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md
grep -n 'scene.xml' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md | head -5
```

Expected:
- First: `1` (one [0.2.1] header)
- Second: `1` (one [0.2.0] header still present)
- Third: at least 3 hits referencing scene.xml in the new entry

- [ ] **Step C.3: Bump `VERSION`**

In `hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION`:
- **Find**: `0.2.0`
- **Replace with**: `0.2.1`

Verify:

```bash
cat hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION
```

Expected: `0.2.1`.

- [ ] **Step C.4: Edit `mujoco/elrobot_follower/robot.yaml` (2 sub-edits)**

**Edit C.4a (`version.current` bump)**: in `hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml`:
- **Find**: `  current: "0.2.0"`
- **Replace with**: `  current: "0.2.1"`

**Edit C.4b (`upstream.prerequisites` mark scene.xml done)**: in the same file:
- **Find**: `    - scene.xml wrapper with lights/floor (Menagerie convention)`
- **Replace with**: `    - scene.xml wrapper with lights/floor (Menagerie convention; done at 0.2.1)`

Verify:

```bash
python3 -c "
import yaml
with open('hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml') as f:
    data = yaml.safe_load(f)
assert data['version']['current'] == '0.2.1', f'expected 0.2.1, got {data[\"version\"][\"current\"]}'
assert any('done at 0.2.1' in p for p in data['upstream']['prerequisites']), \
    'scene.xml prereq should be marked done at 0.2.1'
print('robot.yaml OK')
"
```

Expected: `robot.yaml OK`.

- [ ] **Step C.5: Fix the stale comment in `manifests/norma/elrobot_follower.scene.yaml:12`**

In `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`:
- **Find**: `# No scene_extras — the MJCF has its own lighting/floor setup.`
- **Replace with**: `# No scene_extras — the engine-tier package's scene.xml wrapper provides lighting/floor (use that for visual inspection; this Norma manifest just loads the bare MJCF).`

Verify:

```bash
grep -n 'MJCF has its own lighting' hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml || echo "OK: stale comment removed"
grep -n 'engine-tier package' hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml
```

Expected:
- First: `OK: stale comment removed`
- Second: matches the new comment line

- [ ] **Step C.6: Sanity-check the three-way version sync**

```bash
echo "VERSION file:        $(cat hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION)"
echo "robot.yaml current:  $(grep 'current:' hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml | head -1 | tr -d ' ')"
echo "CHANGELOG top entry: $(grep -m1 '^## \[0\.' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md)"
```

Expected:
```
VERSION file:        0.2.1
robot.yaml current:  current:"0.2.1"
CHANGELOG top entry: ## [0.2.1] — 2026-04-12
```

If the three don't all show `0.2.1`, STOP. One of Steps C.2, C.3, C.4 was misapplied.

---

#### Phase D: Verification gates (7 checks)

Phase D runs the 7 verification gates from spec Section 4 success criteria. Do **NOT** commit until every gate passes. If any gate fails, investigate and fix before proceeding.

- [ ] **Step D.1: scene.xml compiles + named elements present (success criterion #1)**

```bash
python3 -c "
import mujoco
import sys
m = mujoco.MjModel.from_xml_path(
    'hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml')
print(f'nu={m.nu}')
ok_nu = m.nu == 8
ok_floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor') >= 0
ok_light = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, 'floor_light') >= 0
sys.exit(0 if (ok_nu and ok_floor and ok_light) else 1)
"
echo "exit: $?"
```

Expected output: `nu=8` then `exit: 0`.

If exit is non-zero, one of the three named-element checks failed. Investigate `scene.xml`.

- [ ] **Step D.2: Smoke test passes (success criterion #2)**

```bash
PYTHONPATH= python3 -m pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_scene_loadable.py -v 2>&1 | tail -10
```

Expected output ends with `3 passed in <N>s`.

- [ ] **Step D.3: `make sim-test` delta = +3 (baseline-relative assertion, success criterion #3)**

```bash
make sim-test 2>&1 | tail -3
```

Expected output: `BASELINE_PASSED + 3 passed, BASELINE_SKIPPED skipped` — i.e., **`93 passed, 1 skipped`** if pre-Chunk-2 baseline was `90 passed, 1 skipped`.

**Why delta = +3** (note this is different from the spec Section 4 prediction which said "+1"): the spec predicted "+1 from `test_scene_loadable.py`" because it assumed the smoke test would be a single test function. This plan implements **3 test functions** in `test_scene_loadable.py` (one for compile/nu, one for floor, one for light) per the codex iter-2 strengthened smoke pattern. Therefore the actual delta is +3, not +1. The 3 functions are: `test_scene_xml_compiles_and_includes_main_mjcf`, `test_scene_xml_floor_geom_exists_by_name`, `test_scene_xml_floor_light_exists_by_name`. **The plan author updated this delta from the spec's +1 to +3 here so the implementer's assertion matches reality.** (The spec's "+1" was a rough estimate at the lean roadmap level; the +3 here is the precise plan-level number.)

If you measure a different delta, investigate.

- [ ] **Step D.4: Engine-tier tests pass without `PYTHONPATH` (success criterion #4)**

```bash
PYTHONPATH= python3 -m pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -v 2>&1 | tail -15
```

Expected output ends with one of:
- `7 passed, 1 skipped in <N>s` (pre-Chunk-2 was 4 passed + 1 mjx skip; +3 from new smoke test = 7 passed + 1 mjx skip, mjx still not installed)
- `8 passed in <N>s` (mjx installed)

- [ ] **Step D.5: cp -r /tmp self-containment (success criterion #5)**

```bash
rm -rf /tmp/elrobot-chunk2-test
cp -r hardware/elrobot/simulation/mujoco/elrobot_follower /tmp/elrobot-chunk2-test
PYTHONPATH= python3 -m pytest /tmp/elrobot-chunk2-test/tests/ -v 2>&1 | tail -10
rm -rf /tmp/elrobot-chunk2-test
```

Expected: same as D.4 (`7 passed, 1 skipped` or `8 passed`). The `<include file="elrobot_follower.xml"/>` in `scene.xml` resolves relative to `scene.xml`'s directory, which is the same directory in `/tmp/`. This verifies scene.xml didn't break self-containment.

- [ ] **Step D.6: `make check-arch-invariants` (success criterion #6)**

```bash
make check-arch-invariants 2>&1 | tail -5
```

Expected: ends with `All architecture invariants hold ✓` (or equivalent green output).

- [ ] **Step D.7: `git status` shows only the expected changes**

```bash
git status --short
```

Expected:
- 2 `??` (untracked) entries → wait, no — the new files will be under `mujoco/elrobot_follower/` which is already tracked, so git treats them as `A` (added) once you `git add`. Before staging, they show as `??`.
- After staging in Phase E, the count is:
  - 2 `A` (new): scene.xml + test_scene_loadable.py
  - 5 `M` (modified): README, CHANGELOG, VERSION, robot.yaml, manifests/norma/elrobot_follower.scene.yaml
- Total: 7 entries. Plus expected untracked: `MUJOCO_LOG.TXT`, `station_data/`.

Before `git add`:
- 2 `??` for the new files (scene.xml + test_scene_loadable.py)
- 5 ` M` for the modified files
- 2 `??` for the expected untracked

Sanity check before Phase E: `git status --short | wc -l` should be **9** (2 new + 5 modified + 2 expected untracked).

---

#### Phase E: Atomic commit

Only proceed here if **all 7 verification gates** in Phase D passed.

- [ ] **Step E.1: Final review of the change set**

```bash
git status --short
git diff --stat            # all unstaged changes
```

Expected: 5 modified files + 2 untracked new files (and 2 expected untracked log/data items). No surprises.

- [ ] **Step E.2: Stage all changes with explicit paths**

```bash
git add hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml \
        hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_scene_loadable.py \
        hardware/elrobot/simulation/mujoco/elrobot_follower/README.md \
        hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md \
        hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION \
        hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml \
        hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml
```

**Do NOT use `git add -A`** — explicit paths prevent accidental staging of `MUJOCO_LOG.TXT`, `station_data/`, or any other untracked files. **Do NOT use `git add <directory>`** — Chunk 1's lesson: `git add <directory>` doesn't pick up modified files alongside new files cleanly. Use explicit per-file paths.

Verify:

```bash
git status --short
```

Expected: every changed file is shown with a single-character indicator at column 1 (`A` for new, `M` for modified). No remaining unstaged changes for the files in the chunk. Total: 7 staged + 2 untracked = 9 lines.

- [ ] **Step E.3: Atomic commit**

Use a HEREDOC for the multi-paragraph commit message:

```bash
git commit -m "$(cat <<'EOF'
mvp3-c2: add scene.xml Menagerie-style wrapper + smoke test

Chunk 2 of MVP-3 Engine Package Completion: adds a Menagerie-style scene
wrapper to the engine-tier robot package and a 3-test smoke gate. After
this chunk, running `python3 -m mujoco.viewer hardware/elrobot/simulation/
mujoco/elrobot_follower/scene.xml` opens an ElRobot visualization with
floor and lights — the upstream/visualization ergonomics half of MVP-3
EPC is done. The other half (test decoupling) waits for Chunk 3.

What was added:

- mujoco/elrobot_follower/scene.xml — ~30-line Menagerie-style wrapper
  containing <include file="elrobot_follower.xml"/>, <statistic>,
  <visual> (headlight + haze + camera angle), <asset> (skybox + checker
  groundplane texture + material), <worldbody> (directional light named
  "floor_light" + plane geom named "floor"). NO <compiler> block (would
  override main MJCF's meshdir="assets" via <include> namespace merge —
  codex iter-1 risk #1 mitigation). NO <default> block (would collide
  with main MJCF's <default class="elrobot">). Visual/asset block
  numerical values (texture rgb, headlight RGB, haze, etc.) copied
  verbatim from vendor/menagerie/trs_so_arm100/scene.xml for cross-
  package consistency.

- mujoco/elrobot_follower/tests/test_scene_loadable.py — 3 smoke tests:
  test_scene_xml_compiles_and_includes_main_mjcf (asserts nu == 8 to
  verify <include> namespace merge works), test_scene_xml_floor_geom_
  exists_by_name (asserts mj_name2id(GEOM, "floor") >= 0),
  test_scene_xml_floor_light_exists_by_name (asserts
  mj_name2id(LIGHT, "floor_light") >= 0). Per spec Section 4 risk #3 /
  codex iter-1 reframe, all assertions are NAME-based, NOT count-based —
  m.ngeom == N would be fragile to future geometry additions; named
  element lookups are stable across structural changes.

Polish backfills folded into this chunk (carried over from Chunk 1):

- mujoco/elrobot_follower/README.md: backfilled <TBD-mvp3-chunk1>
  placeholder with actual Chunk 1 commit SHA 51ecccd. Added a
  "## How to view" section documenting
  `python3 -m mujoco.viewer scene.xml` as the canonical visual
  inspection entry point.

- manifests/norma/elrobot_follower.scene.yaml line 12: fixed stale
  comment "# No scene_extras — the MJCF has its own lighting/floor
  setup." that was an MVP-2 Chunk 5 leftover. The MJCF does NOT have
  lighting/floor (that's exactly what scene.xml is for). New comment
  references the scene.xml wrapper.

Metadata bumps (three-way version sync):

- mujoco/elrobot_follower/VERSION: 0.2.0 -> 0.2.1 (patch — purely
  additive, no API or physics change, no consumer impact). Per spec
  Section 4 VERSION bump policy.
- mujoco/elrobot_follower/robot.yaml version.current: 0.2.0 -> 0.2.1
  in sync with VERSION file.
- mujoco/elrobot_follower/robot.yaml upstream.prerequisites:
  "scene.xml wrapper with lights/floor (Menagerie convention)" line
  marked "(done at 0.2.1)". Prerequisite progress: 1/5 done at v0.1.0
  (LICENSE), 2/5 done at v0.2.1 (+ scene.xml wrapper). Remaining: 3/5
  after Chunk 3 (+ tests in isolation), then sysid_complete and full
  CITATION.cff for MVP-4+.
- mujoco/elrobot_follower/CHANGELOG.md: new [0.2.1] entry above
  [0.2.0], including Added/Changed/Physics gate/Integration context
  sections.

Verification (per spec Section 4 success criteria, baseline-relative
deltas per Section 7 alpha-prime):

- make sim-test: BASELINE_PASSED + 3 / BASELINE_SKIPPED + 0 (the +3
  comes from the 3 new smoke test functions, not +1 as the spec lean
  roadmap predicted — that was a rough estimate, the plan-level +3 is
  the precise number). At time of execution: 90 -> 93 passed, 1
  skipped.
- make check-arch-invariants: All architecture invariants hold ✓
- pytest mujoco/elrobot_follower/tests/ (no PYTHONPATH): 7 passed +
  1 mjx skip (engine-tier package owns its own absolute count — Chunk
  1 was 4 passed + 1 skip, +3 from new smoke = 7 passed + 1 skip)
- cp -r /tmp self-containment: 7 passed + 1 mjx skip (scene.xml's
  <include file="elrobot_follower.xml"/> resolves relative to scene.xml's
  directory, which works in /tmp/ since the package is self-contained
  since Chunk 1)
- robot.yaml upstream.prerequisites: 1/5 done (Chunk 0) -> 2/5 done
  (this commit)

Files unchanged in this chunk (explicit boundary — codex iter-1 zero
physics drift rule):

- elrobot_follower.xml — main MJCF body untouched (no <default>,
  <contact>, <actuator>, <tendon> changes)
- elrobot_follower.urdf — kinematic source of truth untouched
- All Rust crates (sim-runtime, station, st3215-* bridges)
- norma_sim Python library
- vendor/menagerie/
- software/station/clients/station-viewer/public/elrobot/ (frontend has
  its OWN independent URDF + STL copies, intentionally untouched)
- All other tests in software/sim-server/tests/
- All other tests in mujoco/elrobot_follower/tests/ (mimic_gripper,
  urdf_parity, mjx_compat all untouched)

Roadmap spec: docs/superpowers/specs/2026-04-12-mvp3-foundation-roadmap-design.md
Predecessor commits: 6ef605b (Chunk 0), 51ecccd (Chunk 1), 0ddeb60 (Chunk 1 polish + γ doc)
Plan: docs/superpowers/plans/2026-04-12-mvp3-chunk2-scene-wrapper.md

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

Verify:

```bash
git log --oneline -3
git status --short
git show --stat HEAD | head -30
```

Expected:
- HEAD commit message starts with `mvp3-c2: add scene.xml Menagerie-style wrapper + smoke test`
- `git status` shows clean tree (only the 2 expected untracked items)
- `git show --stat HEAD` shows **7 distinct files**: 2 new files + 5 modified

- [ ] **Step E.4: Post-commit `make sim-test` re-run sanity check**

```bash
make sim-test 2>&1 | tail -10
```

Expected: `BASELINE_PASSED + 3 passed, BASELINE_SKIPPED skipped` — same as Step D.3. This re-verifies that the commit didn't accidentally omit any staged change.

---

#### Phase F: Self-review report

- [ ] **Step F.1: Produce a short report answering**

1. How many files were in the final commit? (Expected: **7** = 2 new + 5 modified.)
2. Did all 7 Phase D verification checks pass? (Expected: yes.)
3. What was the actual `BASELINE_PASSED` and `BASELINE_SKIPPED` measured in Step A.2, and did the post-commit total = baseline + 3 passed (the new smoke test functions)?
4. Did the cp -r /tmp self-containment check produce 7 passed + 1 skipped (or 8 passed if mjx)?
5. Optional manual GUI gate (per spec Open Decision U3): did you run `python3 -m mujoco.viewer hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml` and see an ElRobot model with floor + lights? (Skipped in headless dev environments — fine to not run.)
6. Any surprises during execution? (E.g., a verification gate had unexpected output, the grep at Step A.4 had a different result from plan-write time, the manual smoke compile in Step B.2 found a namespace collision.)
7. Any steps that were ambiguous or required judgment beyond what the plan specified?
8. Final `git log --oneline -5` output showing the new commit at HEAD with `0ddeb60` directly beneath it.

The report goes into the task completion message back to the controller.

---

## Completion Criteria

Task 1 is complete when:

1. ✅ The single commit exists on `main` with the exact commit message from Step E.3.
2. ✅ `make sim-test` shows `BASELINE_PASSED + 3 passed, BASELINE_SKIPPED skipped` (delta = +3 from Step A.2 baseline).
3. ✅ `make check-arch-invariants` passes.
4. ✅ Engine-tier tests pass without `PYTHONPATH` (7 passed + 1 mjx skip OR 8 passed if mjx installed).
5. ✅ `cp -r mujoco/elrobot_follower /tmp/test && pytest tests/` passes (7 passed + 1 mjx skip OR 8 passed).
6. ✅ The Phase D.1 named-element check exits 0 (`scene.xml` compiles, `nu == 8`, `floor` geom and `floor_light` light both discoverable by name).
7. ✅ `git status` is clean (only expected untracked).
8. ✅ Three-way version sync: VERSION file, robot.yaml `version.current`, CHANGELOG `[0.2.1]` all show `0.2.1`.
9. ✅ `<TBD-mvp3-chunk1>` placeholder no longer appears anywhere in the repo (Chunk 2 polish duty discharged).
10. ✅ Stale comment `MJCF has its own lighting/floor setup` no longer appears anywhere in the repo (Chunk 2 polish duty discharged).

If all 10 criteria are met, Chunk 2 is done. Proceed to MVP-3 Chunk 3 (A1 — Test Decoupling) brainstorming as a separate session.

---

## Risks and Rollback

**Primary risk**: namespace collision between `scene.xml`'s `<asset>` / `<worldbody>` and the main MJCF's existing `<asset>` / `<worldbody>`. The MJCF has mesh-derived asset names like `Joint_01_1` and the standard worldbody robot tree. `scene.xml` adds: `groundplane` texture, `groundplane` material, `floor_light`, `floor`. None of these names exist in the main MJCF (verified at plan-write time by reading `elrobot_follower.xml`). If somehow a future edit to the main MJCF introduces a name collision, the smoke test's `nu == 8` check or `mj_name2id` lookups would catch it.

**Secondary risk**: a typo in `scene.xml` causes a compile error. Mitigation: Step B.2 manually compiles `scene.xml` immediately after creation, before writing any other files. If compile fails, fix scene.xml before continuing.

**Rollback** (pre-commit):

```bash
cd /home/yuan/proj/norma-core
git restore --staged .
git restore .
rm -f hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml \
      hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_scene_loadable.py
```

After a rollback, the repo should show the pre-Chunk-2 state (HEAD `0ddeb60` or wherever you started). Verify with:

```bash
git status --short
ls hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml 2>&1 || echo "OK: scene.xml not yet created"
ls hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_scene_loadable.py 2>&1 || echo "OK: smoke test not yet created"
cat hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION  # should still be 0.2.0
```

Then investigate the root cause of the failure and re-attempt from Step A.1.

**Post-commit rollback** (if a regression is discovered AFTER Step E.3 lands the commit): use `git revert <commit_sha>`, **NOT** `git reset --hard`. Per roadmap spec Section 6 hardening (codex iter-1).

```bash
git revert <chunk_2_commit_sha>   # creates a new commit that undoes Chunk 2
```

**Do NOT**:
- Use `git add -A` (would accidentally stage `MUJOCO_LOG.TXT`, `station_data/`, etc.)
- Use `git add <directory>` (Chunk 1's lesson: doesn't pick up modified files cleanly alongside new files; use explicit per-file)
- Amend the commit after the fact (the commit is meant to be the single atomic unit per roadmap spec)
- Commit partway through Phases B/C (atomicity is a chunk requirement)
- Skip verification checks in Phase D (they catch real issues)
- Touch `software/station/clients/station-viewer/public/elrobot/` (frontend's INDEPENDENT URDF + STL copies — chunk 2 must not affect them)
- Touch any Rust file, any norma_sim file, any vendor/menagerie file, or any MJCF body element (zero physics drift, zero Rust changes per roadmap Section 8)
- Add a `<compiler>` block to scene.xml (would override main MJCF's meshdir via include namespace merge)
- Add a `<default>` block to scene.xml (would collide with main MJCF's `<default class="elrobot">`)
- Use count-based assertions (`m.ngeom == N`) in the smoke test — only name-based per codex iter-1

---

## Execution Notes

- **`<include>` is a textual merge, not a Python-style import**. The included MJCF's namespace is fully merged into the parent. `scene.xml`'s blocks add to (not replace) the included blocks — both `<asset>` blocks coexist, both `<worldbody>` blocks coexist. The element NAMES must be unique across the merged namespace.
- **The light name `floor_light`** is a plan-author choice (vendor menagerie's light is anonymous). Naming the light makes the smoke test possible. If the user prefers a different name, change `scene.xml` and `test_scene_loadable.py` consistently.
- **The geom name `floor`** matches vendor menagerie's `<geom name="floor">`. No reason to deviate.
- **The `<statistic>` `center` value `0 0 0.1`** is centered on the ElRobot's roughly-at-origin base. If the ElRobot's actual base position differs significantly, the `mujoco.viewer` default camera will look at the wrong spot. Adjust if visual inspection reveals the issue.
- **PYTHONPATH for Phase B.4 / D.2 / D.4 / D.5**: the engine-tier tests must run WITHOUT `PYTHONPATH=software/sim-server`. If your shell env sets PYTHONPATH, use `PYTHONPATH= python3 -m pytest ...` to clear it.
- **`make sim-test` in Phase D.3 / E.4** sets PYTHONPATH automatically via the Makefile, no need to clear.
- **The CHANGELOG `[0.2.1]` entry is ~50 lines** — verbose but follows the same Keep-a-Changelog convention as `[0.2.0]`. Don't shorten — future readers (and upstream contribution) benefit from explicit Added/Changed/Physics gate/Integration context structure.
- **The `manifests/norma/elrobot_follower.scene.yaml` edit is in a SIBLING directory** (not in the engine-tier package). It's the only edit outside the package, and it's a 1-line comment fix. Don't extend the scope here — the manifest's actual data (mjcf_path, actuator_annotations, etc.) is untouched.

*End of plan.*
