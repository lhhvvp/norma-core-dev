# MVP-3 Chunk 0: Promote `hardware/elrobot/simulation/` to First-Class Directory Structure

| | |
|---|---|
| **Date** | 2026-04-12 |
| **Status** | Design locked (pending spec review) |
| **Parent plan** | MVP-3 Chunk 0 — first chunk of MVP-3 |
| **Prerequisites** | MVP-2 merged to main at `93c1597` |
| **Target branch** | `main` (direct atomic commit, no feature branch) |
| **Estimated scope** | ~30–60 minutes; single atomic commit |
| **Execution approach** | Atomic commit on main checkout (Approach 2 from brainstorming) |

---

## 1. Motivation

During MVP-2 Chunk 4's execution (2026-04-11), a parallel session produced the
**"一等公民 MJCF"** insight (captured in `~/.claude/projects/-home-yuan-proj-norma-core/memory/sim_starting_point.md`):

> MJCF 不是"MVP-2 的一个产出文件"，应该被视为**独立于代码的、有自己版本/CHANGELOG/测试/文档的一等工程 artifact**。这是 Menagerie 作为项目成功的根本原因——他们把每个机器人 MJCF 当成独立的软件包维护。

The core argument:

1. URDF describes "robot design"; MJCF describes "simulation physics parameters". They are different information layers that cannot be derived from each other.
2. MJCF contains seven classes of information that URDF does not carry: `armature`, `frictionloss`, visual–collision separation, `<exclude>` rules, PD `kp`/`kv` gains, solver parameters, and keyframes.
3. These seven classes come from **measurement** (datasheets, sysID, simulation engineer judgment) and **cannot be derived automatically**. MVP-1's `gen.py` pipeline failed precisely because it assumed URDF + a recipe was sufficient.
4. MVP-2's approach (fork Menagerie's hand-tuned physics as the baseline) is the correct pattern. **But Menagerie derives its value from treating each robot MJCF as an independently-maintained software package** — with its own VERSION, CHANGELOG, tests, documentation, and contribution process. If NormaCore wants the same long-term benefits, it must adopt the same treatment for `hardware/elrobot/simulation/`.

MVP-2's execution explicitly deferred the restructure to "Chunk 7.x or MVP-3
Chunk 0" to avoid mid-plan drift. This spec executes that deferral.

### Why now (and not later)

- Chunk 0 is low-risk (mostly `git mv` + new docs + mechanical path updates).
- Every subsequent MVP-3 chunk (whether policy training, multi-robot support,
  or the `usbvideo-compat-bridge` deferred acceptance test) will reference
  this directory. Doing the restructure first keeps later chunks clean.
- Currently the comparison table (`parameters/calibration_notes.md`'s
  eventual content) lives at
  `docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md` —
  which is awkwardly positioned as a "spec artifact" when it is actually
  a physics-model calibration record. Moving it closer to the MJCF is a
  clear organizational win even in isolation.

---

## 2. Scope

### 2.1 In scope

- Create `models/`, `parameters/`, `tests/` subdirectories under
  `hardware/elrobot/simulation/`.
- Move `elrobot_follower.xml` and `elrobot_follower.scene.yaml` into `models/`.
- Move-and-rename `docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md`
  to `hardware/elrobot/simulation/parameters/calibration_notes.md`.
- Move-and-rename `software/sim-server/tests/integration/test_elrobot_acceptance.py`
  to `hardware/elrobot/simulation/tests/test_physics_acceptance.py`.
- Move `software/sim-server/tests/world/test_mimic_gripper.py` to
  `hardware/elrobot/simulation/tests/test_mimic_gripper.py`.
- Add `README.md`, `CHANGELOG.md`, `VERSION` at the root of
  `hardware/elrobot/simulation/`.
- Add a self-contained `conftest.py` inside the new `tests/` directory.
- Update the `elrobot_follower.xml` `<compiler meshdir>` attribute to account
  for the new models/ depth.
- Update **all** path references to the moved files across the repository
  (6+ files identified via `grep -rn`).
- Update `Makefile` `sim-test` target to discover the new test directory.
- Commit everything as one atomic commit on `main`.

### 2.2 Out of scope (deferred to later chunks)

- Splitting `test_physics_acceptance.py` into `test_physics_invariants.py`
  (Floors 1–3) + `test_step_response.py` (Floor 4). The memory's "ideal
  structure" section proposes this, but the monolithic file works and
  splitting adds risk without immediate value.
- Moving `elrobot_follower.urdf` into `urdf/`. The URDF is an input to the
  MJCF construction process, but it is also referenced by other tools and
  has its own stability expectations. Defer the decision.
- Creating `docs/` subdirectory with `how-to-measure-armature.md`,
  `sysid-procedure.md`, `upstream-to-menagerie.md`. These are all valuable
  eventually, but are narrative documents with no content yet. Skeleton
  placeholders would be noise.
- Moving `menagerie_so_arm100.scene.yaml` anywhere. It is a Menagerie
  walking-skeleton fixture, not an ElRobot-specific file. It stays at
  `hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml` for now.
  (A future reorganization may relocate it into `vendor/menagerie/` or
  a dedicated `fixtures/` directory.)
- Moving `assets/*.stl` into `models/assets/`. Requires git-moving 19 binary
  blobs; the cheaper path is updating `meshdir="../assets"` so the MJCF
  continues to resolve meshes from the existing `assets/` directory.
- Integrating `hardware/elrobot/simulation/` as a publishable Python package
  with `pyproject.toml`. Premature; the tests are pytest-discoverable without
  it.
- Creating a CI job specifically for this directory. The existing
  `make sim-test` target will invoke both old and new test locations.

### 2.3 Non-goals for this chunk

- No physics parameter tuning. The MJCF content is untouched (except for
  the single `meshdir` attribute).
- No new tests. Only test relocation.
- No changes to Rust crates.
- No changes to `norma_sim` source code (only docstring updates).
- No changes to `docs/superpowers/plans/` or other spec files (only one
  `git mv` from the specs directory).

---

## 3. Target Directory Structure

### 3.1 Post-Chunk-0 tree for `hardware/elrobot/simulation/`

```
hardware/elrobot/simulation/
├── README.md                       ← NEW
├── CHANGELOG.md                    ← NEW
├── VERSION                         ← NEW (0.1.0)
├── models/                         ← NEW directory
│   ├── elrobot_follower.xml        ← MOVED (+ meshdir updated)
│   └── elrobot_follower.scene.yaml ← MOVED (no content change)
├── parameters/                     ← NEW directory
│   └── calibration_notes.md        ← MOVED+RENAMED from
│                                     docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md
├── tests/                          ← NEW directory
│   ├── conftest.py                 ← NEW (self-contained fixtures)
│   ├── test_physics_acceptance.py  ← MOVED+RENAMED from
│   │                                 software/sim-server/tests/integration/test_elrobot_acceptance.py
│   └── test_mimic_gripper.py       ← MOVED from
│                                     software/sim-server/tests/world/test_mimic_gripper.py
│
├── assets/                         ← UNCHANGED (19 STL files, too many blobs to move)
├── elrobot_follower.urdf           ← UNCHANGED (urdf/ subdir deferred)
├── menagerie_so_arm100.scene.yaml  ← UNCHANGED (Menagerie walking skeleton fixture)
└── vendor/menagerie/               ← UNCHANGED (Chunk 1 vendor, self-contained)
```

### 3.2 Rationale for subdirectory choices

- **`models/`** — matches Menagerie's per-robot MJCF directory pattern. Holds
  everything the MuJoCo compiler needs to construct the model: the MJCF
  plus its runtime scene config. Other Menagerie MJCFs also place the
  scene wrapper alongside the body XML.
- **`parameters/`** — physics-level metadata that is not itself consumed by
  MuJoCo but documents and justifies parameter choices. `calibration_notes.md`
  is the Menagerie→ElRobot comparison table; future sysID data will go here.
- **`tests/`** — MJCF-specific validation. Separating from
  `software/sim-server/tests/` makes it clear that these tests exercise the
  physics model, not the application code. Also permits upstream contribution
  of the entire directory as a standalone Menagerie package.

---

## 4. File Operations

### 4.1 Git moves (5 operations)

```bash
git mv hardware/elrobot/simulation/elrobot_follower.xml \
       hardware/elrobot/simulation/models/elrobot_follower.xml

git mv hardware/elrobot/simulation/elrobot_follower.scene.yaml \
       hardware/elrobot/simulation/models/elrobot_follower.scene.yaml

git mv docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md \
       hardware/elrobot/simulation/parameters/calibration_notes.md

git mv software/sim-server/tests/integration/test_elrobot_acceptance.py \
       hardware/elrobot/simulation/tests/test_physics_acceptance.py

git mv software/sim-server/tests/world/test_mimic_gripper.py \
       hardware/elrobot/simulation/tests/test_mimic_gripper.py
```

Git tracks renames automatically when the file content is identical or nearly
identical. All five of these should appear in `git log --follow` as renames,
preserving per-file history.

### 4.2 Content changes in moved files

Only one of the moved files needs a content edit:

**`hardware/elrobot/simulation/models/elrobot_follower.xml`:**

```diff
-  <compiler angle="radian" meshdir="assets" autolimits="true"/>
+  <compiler angle="radian" meshdir="../assets" autolimits="true"/>
```

The file moves from `simulation/` (depth 3 from repo root) to
`simulation/models/` (depth 4). MuJoCo's `meshdir` attribute is relative to
the MJCF file's parent directory. The assets stay at `simulation/assets/`,
so from `simulation/models/elrobot_follower.xml`, the path to assets is
one directory up plus `assets/`.

No other moved file needs content edits:

- `elrobot_follower.scene.yaml` has `mjcf_path: ./elrobot_follower.xml` which
  is relative to the scene yaml's own directory. Both files move together
  into `models/`, so the relative path remains valid.
- `calibration_notes.md` is pure documentation. No cross-file references to
  its previous location (we verified via grep; plan/spec files in `docs/` are
  explicitly out of scope).
- `test_physics_acceptance.py` uses the `elrobot_scene_yaml` fixture, which
  is provided by the new `conftest.py` with the updated path. No test source
  change needed.
- `test_mimic_gripper.py` uses the `elrobot_mjcf_path` fixture — same story.

### 4.3 New files

Four new files are created at the paths shown below.

#### 4.3.1 `hardware/elrobot/simulation/VERSION`

```
0.1.0
```

Single-line file (with trailing newline). Semver tracking the physics model
only, independent of the `software/` crates' VERSION. Rationale for starting
at `0.1.0`:

- Signals "calibration is iterative; nothing has been measured against real
  hardware yet".
- Leaves room for patch/minor/major bumps before hitting `1.0.0`, which will
  be reserved for the first release with real-hardware sysID data.
- Matches the starting version proposed in the "一等公民 MJCF" memory SOP.

#### 4.3.2 `hardware/elrobot/simulation/README.md`

Full content:

```markdown
# ElRobot Physics Model

This directory contains the MuJoCo physics model (MJCF) for the ElRobot
follower arm — 8 actuators (7 revolute + 1 gripper with tendon-mimic
parallel jaws), forked from MuJoCo Menagerie's `trs_so_arm100` physics
baseline and hand-tuned to ElRobot's 8-joint URDF kinematics.

## What's here

- `VERSION` — semver for the physics model, independent of `software/` VERSION
- `CHANGELOG.md` — chronological log of physics-relevant changes
- `models/elrobot_follower.xml` — main MJCF (8 joints + 2 gripper mimic slides)
- `models/elrobot_follower.scene.yaml` — runtime scene config consumed by `norma_sim`
- `parameters/calibration_notes.md` — per-joint physics rationale (armature,
  frictionloss, kp, dampratio, forcerange) with cross-references to Menagerie
- `tests/` — MJCF-specific validation (physics invariants + step response + P0 mimic)
- `vendor/menagerie/` — reference: upstream Menagerie MJCF + VENDOR.md + LICENSE
- `assets/*.stl` — visual meshes (referenced by MJCF via `meshdir="../assets"`)
- `elrobot_follower.urdf` — original URDF kinematics source (stays here for reference)

## How to modify

To change a physics parameter (armature, kp, dampratio, forcerange, etc.):

1. Edit `models/elrobot_follower.xml` directly
2. Update `parameters/calibration_notes.md` with the reason for the change
3. Bump `VERSION` (patch for tuning, minor for structural, major for breaking)
4. Add an entry to `CHANGELOG.md` under `[Unreleased]`
5. Run the tests:
   ```bash
   PYTHONPATH=software/sim-server python3 -m pytest \
       hardware/elrobot/simulation/tests/ -v
   ```
6. If Floor §3.1 acceptance tests fail, iterate (5-iteration tuning budget
   per motor per MVP-2 spec §7.5)

To add a new joint or body: same flow, plus update the URDF if the
kinematics change.

To re-vendor Menagerie: see `vendor/menagerie/VENDOR.md`.

## Relationship to software

- `software/sim-server/norma_sim/world/manifest.py` loads this MJCF via the scene yaml
- `software/station/bin/station/station-sim.yaml` points at `models/elrobot_follower.scene.yaml`
- `software/sim-bridges/st3215-compat-bridge/presets/elrobot-follower.yaml` maps
  ElRobot actuator names to fake ST3215 motor_ids for the web UI's slider viewer
- The Python test suite at `software/sim-server/tests/world/` still consumes
  `elrobot_scene_yaml` / `elrobot_mjcf_path` fixtures (defined in both
  `software/sim-server/tests/conftest.py` and
  `hardware/elrobot/simulation/tests/conftest.py` — the duplication is
  intentional so this directory can be exercised independently)

## Upstream contribution

This model may eventually be contributed to `mujoco_menagerie` as an
`elrobot_follower` sibling of `trs_so_arm100`. Prerequisites (all TODO
for post-MVP-3):

- sysID measurements on a real ElRobot (armature + frictionloss from experiment,
  not copied from Menagerie)
- Menagerie's contribution review process (tests, LICENSE, style conventions)
- A `README.md` in `mujoco_menagerie/elrobot_follower/` following their format

See `docs/upstream-to-menagerie.md` (not yet written) for the procedure once
it's defined.
```

#### 4.3.3 `hardware/elrobot/simulation/CHANGELOG.md`

Full content:

```markdown
# ElRobot Physics Model CHANGELOG

Follows a subset of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versioning is semver, independent of the `software/` crates.

## [Unreleased]

(nothing yet)

## [0.1.0] — 2026-04-12

### Added

- Initial hand-written MJCF `models/elrobot_follower.xml` (260 lines) derived
  from Menagerie `trs_so_arm100` v1.3 @ commit `c771fb04055d805f20db0eab6cb20b67555887d0`
  (2025-06-09 tuning).
- 8 `<position>` actuators (`act_motor_01` .. `act_motor_08`), mapped via
  `actuator_annotations` in the scene yaml to client-facing `actuator_id`
  `rev_motor_01` .. `rev_motor_08`.
- Menagerie-baseline physics defaults in `<default class="elrobot">`:
  `joint armature=0.1 frictionloss=0.1`, `position kp=50 dampratio=1 forcerange=±2.94`.
  forcerange uses URDF effort (±2.94) instead of Menagerie's ±3.5 (documented
  in `parameters/calibration_notes.md`).
- Tendon-based gripper mimic preserved from MVP-1 — 2 mimic slide joints
  (`rev_motor_08_1`, `rev_motor_08_2`) coupled via `<tendon><fixed>` +
  `<equality><tendon>` with multipliers ±0.0115. **P0 invariant** —
  covered by `tests/test_mimic_gripper.py`.
- Self-collision avoidance via 10 `<contact><exclude>` pairs (added after
  MVP-2 Chunk 5 Task 5.2 code review discovered motion-dependent collisions
  that the rest-pose `ncon=0` check didn't catch). Affected pairs cover
  `base_link`↔`Joint_01_1` (M1 stall root cause) + 9 kinematic fold-back
  pairs surfaced by a per-motor full-range sweep.
- Primitive collision geoms (box/cylinder/sphere) replacing MVP-1's
  mesh-based collision (which caused self-intersection at rest).
- `parameters/calibration_notes.md` — Menagerie→ElRobot joint-by-joint
  comparison table, armature/damping/frictionloss inheritance strategy,
  and 4 policy amendments (forcerange=URDF, dampratio not kv, explicit
  ctrlrange not inheritrange, Gripper_Gear_v1_1 diaginertia floor).
  **Finding**: 2 independent ElRobot joints have no Menagerie analog
  (`rev_motor_02`, `rev_motor_05`), not the 3 the plan originally guessed.
- `tests/test_physics_acceptance.py` — 13 tests encoding spec §3.1 Floor
  criteria: Floor 1 (ncon=0 at rest), Floor 2 (M[i,i]+armature≥1e-4 per DOF),
  Floor 3 (10000 random-ctrl steps no NaN), Floor 4 (per-motor step response
  parametrized × 8 motors), Floor 5+6 delegation stubs.
- `tests/test_mimic_gripper.py` — P0 regression, 2 tests.

### Physics gate results (at initial release)

- Floor §3.1 all 6 criteria: GREEN (0 tuning iterations — Menagerie baseline
  passed first-try under MuJoCo's Coulomb frictionloss + gravity bleed,
  contradicting an analytical PD overshoot prediction)
- Ceiling §3.2 item 7 (web UI slider responsiveness including M1): PASS
  (manual browser smoke test 2026-04-12). MVP-1's M1-unresponsive regression
  is resolved.
- Ceiling §3.2 item 8 (MuJoCo viewer side-by-side with Menagerie): DEFERRED
  (headless execution environment; advisory per spec §7.5)

### Known limitations

- Parameters are inherited from Menagerie's 2025-06-09 tuning (no real-hardware
  sysID yet). For the 2 ElRobot-unique joints, nearest-neighbor estimation is
  used — physics is plausible but not measured.
- Gripper_Jaw_01/02 inertial origins were reset to body origin (URDF export
  bug: the URDF had jaw COMs expressed in world-frame coordinates). The
  resulting parallel-axis error (~1.5e-6 kg·m²) is negligible for mimic-
  constrained jaws but worth flagging.
- Merged inertia for fixed joints (ST3215 motor mass collapsed into the
  parent revolute body) omits parallel-axis shift (~5.5e-6 kg·m²). Acceptable
  for Floor gates; re-evaluate for real-hardware tracking.

### Integration context

- MVP-2 merge commit: `93c1597` on `main` (2026-04-12)
- Implemented over Chunks 5-7 of the MVP-2 plan
  (`docs/superpowers/plans/2026-04-11-mvp2-menagerie-walking-skeleton.md`)
```

#### 4.3.4 `hardware/elrobot/simulation/tests/conftest.py`

Full content:

```python
"""Shared pytest fixtures for the ElRobot first-class physics model tests.

These mirror the ElRobot fixtures in software/sim-server/tests/conftest.py so
this directory can be pytest-invoked independently (e.g. for upstream
contribution or MJCF-only iteration). The duplication is intentional per
MVP-3 Chunk 0 design (2026-04-12).
"""
from pathlib import Path

import pytest


@pytest.fixture
def repo_root() -> Path:
    # tests/conftest.py → simulation/ → elrobot/ → hardware/ → repo root
    return Path(__file__).resolve().parents[4]


@pytest.fixture
def elrobot_mjcf_path(repo_root: Path) -> Path:
    """Path to the hand-written ElRobot MJCF."""
    p = repo_root / "hardware/elrobot/simulation/models/elrobot_follower.xml"
    if not p.exists():
        pytest.skip(f"ElRobot MJCF not found at {p}")
    return p


@pytest.fixture
def elrobot_scene_yaml(repo_root: Path) -> Path:
    """Path to the hand-written ElRobot scene.yaml."""
    p = repo_root / "hardware/elrobot/simulation/models/elrobot_follower.scene.yaml"
    if not p.exists():
        pytest.skip(f"ElRobot scene.yaml not found at {p}")
    return p
```

### 4.4 Path updates in unchanged files

Seven existing files retain their location but need content edits to point
at the new `models/` paths. The changes are mechanical substitutions of
`hardware/elrobot/simulation/elrobot_follower*` →
`hardware/elrobot/simulation/models/elrobot_follower*`.

| # | File | Location of reference |
|---|---|---|
| 1 | `software/station/bin/station/station-sim.yaml` | `sim-runtime.launcher` list, `--manifest` arg |
| 2 | `software/station/bin/station/station-shadow.yaml` | same field as station-sim.yaml |
| 3 | `Makefile` | `sim-standalone` target, `--manifest` arg |
| 4 | `software/sim-server/README.md` | Scenario B standalone command example |
| 5 | `software/sim-server/norma_sim/world/manifest.py` | module docstring (line 3) |
| 6 | `software/sim-server/scripts/probe_manifest.py` | usage example in module docstring |
| 7 | `software/sim-server/tests/conftest.py` | `elrobot_mjcf_path` and `elrobot_scene_yaml` fixtures (two path assignments) |

**Load-bearing note on item 7**: Three test files in `software/sim-server/tests/world/`
(`test_model.py`, `test_snapshot.py`, `test_descriptor_build.py`) still use
the `elrobot_scene_yaml` / `elrobot_mjcf_path` fixtures from the old
`conftest.py`. These three test files are NOT moving in this chunk, so the
old `conftest.py` must keep its fixture definitions with updated paths. The
same fixture is therefore defined in two places (old and new `conftest.py`),
each pointing at the new `models/` location. This duplication was approved
during the brainstorming Q4 decision.

### 4.5 Makefile `sim-test` target update

Diff:

```diff
 .PHONY: sim-test
 sim-test:
 	# ... (arch invariants + cargo test targets unchanged)
-	PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/
+	PYTHONPATH=software/sim-server python3 -m pytest \
+	    software/sim-server/tests/ \
+	    hardware/elrobot/simulation/tests/
```

One logical change: append the new test directory to the existing `pytest`
invocation. `PYTHONPATH=software/sim-server` remains required (the new tests
import `norma_sim.world.*` the same way the old tests do).

---

## 5. Verification Strategy

Execute these checks in order immediately after staging and before the commit.
Any failure means the chunk is not complete; fix and re-verify.

### 5.1 New MJCF compiles from the new location

```bash
cd /home/yuan/proj/norma-core
PYTHONPATH=software/sim-server python3 -c "
import mujoco
m = mujoco.MjModel.from_xml_path(
    'hardware/elrobot/simulation/models/elrobot_follower.xml')
print(f'nu={m.nu} neq={m.neq} ntendon={m.ntendon}')
"
```

Expected output:

```
nu=8 neq=2 ntendon=2
```

Failure modes to check if this does not produce the expected output:

- `meshdir="../assets"` not applied → mesh files not found
- Accidental content change during `git mv` (rare but worth verifying)

### 5.2 New test directory is discoverable and passing

```bash
PYTHONPATH=software/sim-server python3 -m pytest \
    hardware/elrobot/simulation/tests/ -v 2>&1 | tail -30
```

Expected: **15 passed** (13 from `test_physics_acceptance.py` + 2 from
`test_mimic_gripper.py`).

### 5.3 Old test directory still green after fixture-path updates

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/ -q \
    2>&1 | tail -5
```

Expected: **73 passed** (88 pre-Chunk-0 total minus 15 moved-out tests).

### 5.4 Full `make sim-test` pipeline green

```bash
make sim-test 2>&1 | tail -10
```

Expected:

- Architecture invariants: all ✓
- Rust: `sim-runtime 28 passed`, `st3215-wire 15 passed`,
  `st3215-compat-bridge 16 passed` (unchanged; zero Rust touched)
- Python: **88 passed, 0 failed, 0 skipped** (73 from sim-server +
  15 from hardware/elrobot/simulation/tests = 88, same total as pre-Chunk-0)

### 5.5 Menagerie walking skeleton permanent regression fixture

```bash
PYTHONPATH=software/sim-server python3 -m pytest \
    software/sim-server/tests/integration/test_menagerie_walking_skeleton.py -q
```

Expected: **6 passed**. This test is not moved and must remain green.

### 5.6 No dangling references to old paths

```bash
grep -rn 'hardware/elrobot/simulation/elrobot_follower\|hardware/elrobot/simulation/elrobot_follower\.' \
    software/ hardware/ Makefile docs/ 2>&1 \
    | grep -v 'docs/superpowers/plans/2026-04-11-mvp2' \
    | grep -v 'docs/superpowers/specs/2026-04-11-mvp2'
```

Expected: **no matches**. The `grep -v` filters exclude historical MVP-2 plan
and spec files — those are frozen documents that reference the old paths as
historical fact and are explicitly out of scope for modification.

### 5.7 Station smoke test (optional)

```bash
PYTHONPATH=software/sim-server ./target/debug/station \
    -c software/station/bin/station/station-sim.yaml --web 0.0.0.0:8889
```

Expected startup log lines (same as MVP-2 Task 7.1):

```
Starting sim-runtime (mode=Internal, startup_timeout_ms=5000)
sim-runtime started: elrobot_follower
st3215_compat_bridge bridge started: robot_id=elrobot_follower ... motors=8
WebSocket server listening on 0.0.0.0:8889
```

This step is optional in headless environments. Its purpose is a full-stack
sanity check that the path-update changes in `station-sim.yaml` resolve
correctly.

---

## 6. Risk Analysis

### 6.1 Low-risk operations

- **`git mv` of text and XML files**: git preserves content and history. Near-zero
  risk of data loss.
- **New markdown files** (`README.md`, `CHANGELOG.md`): additive, no side
  effects.
- **`VERSION` file**: a one-line text file. Trivial.
- **New `conftest.py`**: a fresh file with copied-and-adapted fixtures. The
  `parents[4]` path resolution is the only runtime computation; testable.

### 6.2 Moderate-risk operations

- **`meshdir="../assets"` edit**: a typo or wrong path would cause MJCF compile
  failure, which Section 5.1 catches.
- **`software/sim-server/tests/conftest.py` path update**: if the new path is
  wrong, the world/ tests that still use these fixtures will skip (not fail),
  which could mask a regression. Section 5.3 catches this by asserting the
  expected `73 passed` number (not `N passed, M skipped`).
- **Makefile `sim-test` target path addition**: a typo causes pytest
  collection to skip the new directory, and Section 5.4's `88 passed` total
  check catches it.

### 6.3 Low-but-notable risks

- **Path references in non-test files**: if any of the 7 files in Section 4.4
  is missed, a runtime consumer may reference a nonexistent path. The
  Section 5.6 grep catches this.
- **Accidental edit during `git mv`**: `git mv` followed by content editing
  in the same commit can make renames harder to recognize in `git log`. To
  keep `git log --follow` clean, content edits should happen in separate
  commits — but this chunk uses a single atomic commit per the execution
  approach. The `git log --follow` consequence is accepted as a minor tradeoff
  for atomicity.

### 6.4 No significant risks

- No Rust code changes.
- No physics parameter changes (only the `meshdir` attribute).
- No test logic changes (only test file locations).
- No architecture invariant changes.
- No dependency changes.

### 6.5 Rollback plan

If `make sim-test` fails post-commit and the cause is not immediately
obvious:

```bash
git revert 93c1597..HEAD  # or the specific Chunk-0 commit SHA
```

Because the commit is atomic, a single `git revert` undoes all changes.

---

## 7. Commit Message (draft)

```
mvp3: promote hardware/elrobot/simulation/ to first-class directory structure

Chunk 0 of MVP-3: reorganize the ElRobot physics model into a self-contained
subproject with its own VERSION/CHANGELOG/README/tests, following the pattern
Menagerie uses for each vendored MJCF.

New structure under hardware/elrobot/simulation/:
- README.md, CHANGELOG.md, VERSION (0.1.0) — NEW
- models/ — elrobot_follower.xml + elrobot_follower.scene.yaml (moved)
- parameters/calibration_notes.md — moved+renamed from
  docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md
- tests/ — conftest.py (NEW, self-contained fixtures) + test_physics_acceptance.py
  (moved+renamed from test_elrobot_acceptance.py) + test_mimic_gripper.py
  (moved from sim-server/tests/world/)
- assets/, vendor/, elrobot_follower.urdf, menagerie_so_arm100.scene.yaml: unchanged

Additional changes:
- models/elrobot_follower.xml: meshdir="assets" → "../assets" (models/ is one
  level deeper than assets/)
- Makefile sim-test: pytest now covers both sim-server/tests/ and
  hardware/elrobot/simulation/tests/
- sim-server/tests/conftest.py: elrobot_* fixture paths updated to models/
  (world/ tests consuming these fixtures stay in sim-server/)
- 6 files with hardcoded paths repointed: station-sim.yaml, station-shadow.yaml,
  Makefile sim-standalone, sim-server README, manifest.py docstring,
  probe_manifest.py usage

Rationale: "一等公民 MJCF" insight from MVP-2 — the physics model is an
engineering artifact independent of application code, with its own calibration
history, test suite, and (eventually) upstream contribution path. Menagerie's
success comes from treating each MJCF as its own software package; this
restructure brings hardware/elrobot/simulation/ into line.

Verification (all post-commit):
- make sim-test: 88 passed, 0 failed, 0 skipped (unchanged total)
- pytest hardware/elrobot/simulation/tests/: 15 passed (13 physics + 2 P0 mimic)
- test_menagerie_walking_skeleton: 6 passed (permanent regression fixture intact)
- make check-arch-invariants: all ✓
- Rust: sim-runtime 28/0, st3215-wire 15/0, st3215-compat-bridge 16/0 (zero Rust touched)

Deferred (not in this chunk, candidates for MVP-3 Chunk 1+):
- Splitting test_physics_acceptance.py into test_physics_invariants + test_step_response
- Moving elrobot_follower.urdf into urdf/ subdir
- docs/ subdir with sysid-procedure.md, how-to-measure-armature.md,
  upstream-to-menagerie.md
```

---

## 8. Dependencies and Follow-ups

### 8.1 Dependencies

- **MVP-2 merged to main at `93c1597`** — required because this chunk moves
  files that MVP-2 created. Must execute post-merge, which is the current
  state as of this document.

### 8.2 Immediate follow-ups (outside this chunk)

- Update `sim_starting_point.md` memory to reflect the new directory layout
  under the "How to apply" and "How to resume next session" sections. The
  memory's MVP-3 Chunk 0 SOP is already consistent with this design, so the
  update is only about reflecting execution state, not restructuring the
  memory.

### 8.3 Candidates for MVP-3 Chunk 1+

- Split `test_physics_acceptance.py` into `test_physics_invariants.py` +
  `test_step_response.py`.
- Move `elrobot_follower.urdf` into a new `urdf/` subdirectory.
- Create `docs/` subdirectory with skeleton files:
  - `docs/how-to-measure-armature.md`
  - `docs/sysid-procedure.md`
  - `docs/upstream-to-menagerie.md`
- Real-hardware sysID data collection (requires real ElRobot access).
- `pyproject.toml` at `hardware/elrobot/simulation/` for pip-installability.
- Publish as `elrobot-follower-mjcf` package on PyPI.

### 8.4 Longer-term

- Upstream contribution to `mujoco_menagerie`.
- `normvla/lerobot` dataset publication pipeline using this MJCF as the sim
  environment.

---

## 9. Appendix: Full list of modified files

To support spec review and commit planning, here is the exhaustive list of
files touched by this chunk:

### 9.1 Files moved (5)

1. `hardware/elrobot/simulation/elrobot_follower.xml` → `.../models/elrobot_follower.xml`
2. `hardware/elrobot/simulation/elrobot_follower.scene.yaml` → `.../models/elrobot_follower.scene.yaml`
3. `docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md` → `hardware/elrobot/simulation/parameters/calibration_notes.md`
4. `software/sim-server/tests/integration/test_elrobot_acceptance.py` → `hardware/elrobot/simulation/tests/test_physics_acceptance.py`
5. `software/sim-server/tests/world/test_mimic_gripper.py` → `hardware/elrobot/simulation/tests/test_mimic_gripper.py`

### 9.2 Files with content changes (8)

1. `hardware/elrobot/simulation/models/elrobot_follower.xml` — `meshdir` attribute
2. `software/station/bin/station/station-sim.yaml` — `--manifest` path
3. `software/station/bin/station/station-shadow.yaml` — `--manifest` path
4. `Makefile` — `sim-standalone` `--manifest` path + `sim-test` pytest paths
5. `software/sim-server/README.md` — Scenario B command example
6. `software/sim-server/norma_sim/world/manifest.py` — docstring line 3
7. `software/sim-server/scripts/probe_manifest.py` — docstring usage example
8. `software/sim-server/tests/conftest.py` — fixture paths (lines 43, 53)

### 9.3 New files (4)

1. `hardware/elrobot/simulation/README.md`
2. `hardware/elrobot/simulation/CHANGELOG.md`
3. `hardware/elrobot/simulation/VERSION`
4. `hardware/elrobot/simulation/tests/conftest.py`

### 9.4 Implicit new directories (3)

Created as a side effect of the `git mv` operations and new-file writes:

1. `hardware/elrobot/simulation/models/`
2. `hardware/elrobot/simulation/parameters/`
3. `hardware/elrobot/simulation/tests/`

### 9.5 Total touched files

**17 distinct file-level changes**: 5 moves + 8 edits + 4 new files. All in
a single atomic commit.

---

*End of spec.*
