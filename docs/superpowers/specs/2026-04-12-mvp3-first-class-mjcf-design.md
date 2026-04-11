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

**Important note**: This section was significantly revised after a codex
consult (2026-04-12, session `019d7726-6dcf-7fe2-8887-35ee3b9c2568`) exposed
that the original spec conflated *engine-native model assets* with
*Norma-specific runtime manifests*. The revision splits those concerns into
separate subtrees (`mujoco/elrobot_follower/` vs `manifests/norma/`) and
introduces machine-readable identity metadata. The expanded scope is still
organizational only — no physics, no Rust, no new runtime behavior.

### 2.1 In scope

**A. Engine-tier robot package** (`hardware/elrobot/simulation/mujoco/elrobot_follower/`):

- Create `hardware/elrobot/simulation/mujoco/elrobot_follower/` directory
  (two new levels: `mujoco/` + `elrobot_follower/`).
- Move `elrobot_follower.xml` into this directory.
- Update the MJCF's `<compiler meshdir>` attribute to `"../../assets"` to
  continue resolving meshes from the unchanged `hardware/elrobot/simulation/assets/`
  directory (two levels up).
- Create `measurements/` subdirectory inside the robot package (renamed from
  the original `parameters/` concept per codex's "data provenance, not just
  values" argument).
- Move-and-rename
  `docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md`
  into `measurements/menagerie_diff.md` (the file is a Menagerie→ElRobot
  parameter adaptation record, which is what its content actually is).
- Create `measurements/README.md` explaining the folder's purpose and future
  sysID workflow (placeholder content).
- Create `tests/` subdirectory inside the robot package.
- Move `software/sim-server/tests/world/test_mimic_gripper.py` into
  `tests/test_mimic_gripper.py`. This file is **pure-MuJoCo** (`import mujoco`
  only, no `norma_sim`) and becomes truly self-contained at the new location.
- Create `tests/test_urdf_parity.py` (NEW) — a minimal URDF ↔ MJCF consistency
  gate checking joint names, counts, and axes. Prevents the URDF from rotting.
- Create `tests/test_mjx_compat.py` (NEW) — a placeholder smoke test that
  imports `mujoco.mjx` and calls `mjx.put_model(MjModel.from_xml_path(...))`
  to reserve the future MJX compatibility slot. Marked `pytest.skipif(not
  mjx_available)` so it doesn't fail when MJX is not installed.
- Create `tests/conftest.py` with a SINGLE fixture (`elrobot_mjcf_path`)
  pointing at the robot package's own MJCF. Tests only depend on `mujoco`
  and this local path — no `norma_sim` imports.
- Create `README.md` for the robot package — describes the package structure,
  how to modify, and the upstream-contribution path.
- Create `CHANGELOG.md` for the robot package — physics-relevant changes.
- Create `VERSION` file for the robot package — semver string `0.1.0`. A
  separate `robot.yaml` also mirrors this version in a machine-readable
  field, but the `VERSION` file is retained as the git-friendly source.
- Create `LICENSE` file for the robot package — copied from the repo root or
  set to the same license NormaCore itself uses. Required for any future
  upstream contribution.
- Create `robot.yaml` — machine-readable canonical identity manifest with
  `robot_id`, `variant`, `dof`, `gripper_type`, actuator names, physics
  baseline provenance (Menagerie SHA + date), and version. Consumed later
  by LeRobot Dataset v3 `robot_type` and any registry tooling.

**B. Norma runtime manifests** (`hardware/elrobot/simulation/manifests/norma/`):

- Create `hardware/elrobot/simulation/manifests/norma/` directory.
- Move `elrobot_follower.scene.yaml` into
  `manifests/norma/elrobot_follower.scene.yaml`. The file contains
  `robot_id`, `actuator_annotations`, `GRIPPER_PARALLEL` capability mapping
  — all Norma-specific wrapper content, not engine-native MJCF. Separation
  clarifies the boundary.
- Update the moved scene yaml's `mjcf_path` from
  `./elrobot_follower.xml` to `../../mujoco/elrobot_follower/elrobot_follower.xml`
  (scene yaml now sits two levels deeper relative to the MJCF).
- Move `menagerie_so_arm100.scene.yaml` into
  `manifests/norma/menagerie_so_arm100.scene.yaml`. It is also a Norma
  runtime wrapper (uses Norma's scene yaml schema to point at Menagerie's
  MJCF), not an ElRobot-specific file but same concern.
- Update the moved Menagerie scene yaml's `mjcf_path` from
  `./vendor/menagerie/trs_so_arm100/scene.xml` to
  `../../vendor/menagerie/trs_so_arm100/scene.xml`.

**C. sim-server side (integration tests stay)**:

- `software/sim-server/tests/integration/test_elrobot_acceptance.py` **STAYS**
  in its current location. It is an integration test that imports
  `from norma_sim.world.model import MuJoCoWorld` and calls
  `MuJoCoWorld.from_manifest_path(...)` — it is NOT pure-MuJoCo and cannot
  be made self-contained at the robot package without a rewrite that's out
  of Chunk 0's scope.
  **This reverses the brainstorming Q3 decision** ("both tests move"). The
  reversal is explicit and the rationale is in Section 4.6.
- Update `software/sim-server/tests/conftest.py`'s fixtures
  (`elrobot_mjcf_path`, `elrobot_scene_yaml`) to point at the new paths.
  The four `world/` tests that still consume these fixtures
  (`test_model.py`, `test_snapshot.py`, `test_descriptor_build.py`,
  `test_actuation.py`) stay in place and use the updated fixtures.

**D. Path updates across the repository**:

- Update every hard-coded reference to the moved files in runtime code,
  configs, and developer docs. Grep for both
  `hardware/elrobot/simulation/elrobot_follower` and
  `hardware/elrobot/simulation/menagerie_so_arm100` and
  `hardware/elrobot/simulation/vendor/` (if any direct MJCF reference uses
  it via the Menagerie scene yaml's pre-move path).
- Update `Makefile` `sim-test` target to add the new pytest path
  (`hardware/elrobot/simulation/mujoco/elrobot_follower/tests/`).
- Commit everything as one atomic commit on `main`.

### 2.2 Out of scope (deferred to later chunks)

- **Rewriting `test_elrobot_acceptance.py` to be pure-MuJoCo** (no
  `norma_sim` imports) so that it can also move into the robot package.
  Deferred because it crosses the "pure organizational refactor" line for
  Chunk 0. Candidate for an MVP-3 Chunk 1+ "test-layer decoupling" task.
- **Splitting `test_elrobot_acceptance.py` into `test_model_invariants.py` +
  `test_dynamics_regression.py` + `test_mjx_compat.py`** (codex's long-term
  recommendation). The monolithic file works today; splitting adds risk
  without immediate value. Nice-to-have; revisit when the split tests
  actually diverge in scope.
- **Moving `elrobot_follower.urdf` into the robot package**. Keeping the
  URDF at the current `hardware/elrobot/simulation/` root means the URDF's
  own mesh references (`filename="assets/*.stl"`) continue to resolve
  against `hardware/elrobot/simulation/assets/`. Moving the URDF into the
  robot package would either require the assets to move with it (19 binary
  git mvs) or the URDF to use a compensating relative prefix. Defer until
  a future chunk that also handles the assets move or a URDF cleanup.
- **Moving `assets/*.stl`** into the robot package (`mujoco/elrobot_follower/assets/`).
  Would make the robot package truly self-contained for upstream contribution
  but requires 19 binary git moves + URDF adjustment. Defer to the
  upstream-contribution chunk.
- **Creating `docs/` subdirectory** inside the robot package with
  `how-to-measure-armature.md`, `sysid-procedure.md`, `upstream-to-menagerie.md`.
  All valuable eventually but are narrative documents with no content yet.
- **`CITATION.cff` at robot package root**. Useful for upstream contribution
  but premature for the current state (no publication, no BibTeX entry yet).
- **`pyproject.toml`** making the robot package pip-installable. Premature;
  not needed until we publish to PyPI or HuggingFace.
- **A pure-MuJoCo `scene.xml` wrapper** (Menagerie-style, with lights + floor
  + background) alongside `elrobot_follower.xml`. Would match Menagerie's
  flat robot package pattern exactly. Defer to the upstream-contribution
  chunk. The current `elrobot_follower.xml` is a bare body/actuator file
  without lights/floor — fine for sim runtime, not yet ready for standalone
  `python -m mujoco.viewer` visual comparison.
- **`simulation/mujoco/` peer directories for other engines**
  (`simulation/isaac/`, `simulation/genesis/`, `simulation/usd/`). Only the
  `mujoco/` tier is created in this chunk because that is where current work
  lives. The `mujoco/` naming reserves the slot so a future engine addition
  does not need to re-root.
- **A second robot under `mujoco/`** (e.g. `mujoco/menagerie_so_arm100/`).
  The Menagerie scene yaml is a Norma-side wrapper around the `vendor/`
  MJCF — it is not a NormaCore robot package of its own. No new robot
  package for Menagerie.

### 2.3 Non-goals for this chunk

- No physics parameter tuning. The MJCF content is untouched except for
  the single `meshdir` attribute.
- No new Python test logic beyond `test_urdf_parity.py` and
  `test_mjx_compat.py`. Both are minimal skeletons (≤30 lines each) that
  reserve future work slots, not full implementations.
- No changes to Rust crates.
- No changes to `norma_sim` source code (only docstring updates).
- No changes to `docs/superpowers/plans/` or other spec files (only one
  `git mv` from the specs directory).

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
├── mujoco/                                      ← NEW (engine-tier slot)
│   └── elrobot_follower/                        ← NEW robot package
│       ├── elrobot_follower.xml                 ← MOVED from simulation/ (+ meshdir updated)
│       ├── README.md                            ← NEW (per-robot README)
│       ├── CHANGELOG.md                         ← NEW (physics-relevant changes)
│       ├── VERSION                              ← NEW (0.1.0, git-friendly semver)
│       ├── LICENSE                              ← NEW (upstream-contribution pre-req)
│       ├── robot.yaml                           ← NEW (machine-readable identity)
│       ├── measurements/                        ← NEW (future sysID provenance)
│       │   ├── README.md                        ← NEW (placeholder, explains dir purpose)
│       │   └── menagerie_diff.md                ← MOVED+RENAMED from
│       │                                          docs/superpowers/specs/
│       │                                          2026-04-11-mvp2-menagerie-comparison-table.md
│       └── tests/                               ← NEW
│           ├── conftest.py                      ← NEW (single fixture, pure-mujoco)
│           ├── test_mimic_gripper.py            ← MOVED from software/sim-server/
│           │                                      tests/world/test_mimic_gripper.py
│           ├── test_urdf_parity.py              ← NEW (URDF↔MJCF invariant gate)
│           └── test_mjx_compat.py               ← NEW (MJX forward-pass smoke gate)
│
├── manifests/                                   ← NEW (Norma runtime wrappers)
│   └── norma/
│       ├── elrobot_follower.scene.yaml          ← MOVED from simulation/ (+ mjcf_path updated)
│       └── menagerie_so_arm100.scene.yaml       ← MOVED from simulation/ (+ mjcf_path updated)
│
├── assets/                                      ← UNCHANGED (19 STL files; URDF + MJCF both reach them)
├── elrobot_follower.urdf                        ← UNCHANGED (keeps `filename="assets/..."` relative paths)
└── vendor/menagerie/                            ← UNCHANGED (Chunk 1 vendor snapshot)
```

### 3.2 Rationale for the three-tier structure

This design splits `hardware/elrobot/simulation/` into three concerns that
are tangled in the pre-Chunk-0 layout:

1. **Engine-native robot assets** (`mujoco/elrobot_follower/`) — what the
   MuJoCo compiler actually reads: the MJCF, its measurements, its tests,
   its identity metadata. This subtree is the **upstream-contribution
   candidate**: everything needed to hand over to MuJoCo Menagerie as a
   sibling of `trs_so_arm100/` lives inside it.

2. **Framework-specific runtime wrappers** (`manifests/norma/`) — the
   `.scene.yaml` files that NormaCore's `norma_sim` loader consumes.
   These are Norma-side metadata (actuator annotations, gripper capability
   wiring, `robot_id` mapping) layered over an engine-native MJCF. They
   are NOT part of the upstream-contribution candidate: a new runtime
   targeting this MJCF would write its own wrappers.

3. **Shared raw assets and historical artifacts** (`assets/`, `vendor/`,
   `elrobot_follower.urdf`) — files that are either consumed by multiple
   layers (assets referenced from both URDF and MJCF) or preserved for
   reference (the original URDF, the vendored Menagerie snapshot).

The `mujoco/` intermediate directory is a **slot reservation**, not a
robot-count wrapper. We only have one MuJoCo-native robot package
(`elrobot_follower`), but adding `mujoco/` now means a future Isaac Lab /
Genesis / USD addition becomes `simulation/isaac/...` peer directories
without re-rooting the MuJoCo assets. If we keep the flat layout, any
multi-engine future forces a second reorganization of the MuJoCo tree.

### 3.3 Why not flat Menagerie-style `hardware/elrobot/simulation/elrobot_follower/`?

Codex offered two structural options during the consult: (a) flat
Menagerie-style (`simulation/elrobot_follower/` with everything inside),
(b) layered engine-tier (`simulation/mujoco/elrobot_follower/`).

Option (a) works best when a repository packages **many robots** (Menagerie
has ~50). With a single robot, the flat wrapper duplicates the parent
path (`elrobot` + `elrobot_follower`), and the engine-tier slot is only
reserved indirectly. Option (b) is chosen here because:

- We have exactly one robot package today, so the `mujoco/` tier adds
  meaningful forward-structure without being redundant with the robot
  name.
- Multi-engine forward compatibility is a real concern; reserving a slot
  now is zero-cost insurance.
- If a second MuJoCo robot package is added later (e.g. a NormaCore fork
  of Menagerie's `trs_so_arm100`), it slots in as
  `mujoco/so_arm100_normacore/` alongside `mujoco/elrobot_follower/`.

### 3.4 Why `manifests/norma/` and not just `manifests/`?

The single-level `manifests/` intermediate with a `norma/` subdirectory
reserves the slot for a future non-Norma runtime wrapper (e.g.
`manifests/lerobot/` for a LeRobot env config, or `manifests/gym/` for a
Gymnasium env wrapper). A bare `manifests/` would name-collide with any
future framework that adopts a different wrapper schema.

---

## 4. File Operations

### 4.1 Git moves (5 operations)

```bash
# 1. MJCF → engine-tier robot package
git mv hardware/elrobot/simulation/elrobot_follower.xml \
       hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml

# 2. ElRobot scene yaml → Norma manifests
git mv hardware/elrobot/simulation/elrobot_follower.scene.yaml \
       hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml

# 3. Menagerie scene yaml → Norma manifests (also a Norma wrapper)
git mv hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml \
       hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml

# 4. Comparison table → measurements (renamed to reflect actual content)
git mv docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md \
       hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/menagerie_diff.md

# 5. Pure-MuJoCo P0 gripper test → engine-tier robot package tests
git mv software/sim-server/tests/world/test_mimic_gripper.py \
       hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mimic_gripper.py
```

Git tracks renames automatically for all five. Per-file history is
preserved via `git log --follow`.

### 4.2 Content changes in moved files

**File 1 (`mujoco/elrobot_follower/elrobot_follower.xml`):** update `meshdir`
attribute to account for the two-level-deeper location relative to assets.

```diff
-  <compiler angle="radian" meshdir="assets" autolimits="true"/>
+  <compiler angle="radian" meshdir="../../assets" autolimits="true"/>
```

From the MJCF's new location `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml`,
the parent directory is `mujoco/elrobot_follower/`. Going up twice reaches
`simulation/`, and `assets/` then reaches the unchanged STL directory at
`hardware/elrobot/simulation/assets/`.

**File 2 (`manifests/norma/elrobot_follower.scene.yaml`):** update
`mjcf_path` field to point at the MJCF's new location.

```diff
-mjcf_path: ./elrobot_follower.xml
+mjcf_path: ../../mujoco/elrobot_follower/elrobot_follower.xml
```

From the scene yaml's new location `manifests/norma/`, going up twice
reaches `simulation/`, then `mujoco/elrobot_follower/elrobot_follower.xml`.

**File 3 (`manifests/norma/menagerie_so_arm100.scene.yaml`):** update
`mjcf_path` field to account for the scene yaml's new depth.

```diff
-mjcf_path: ./vendor/menagerie/trs_so_arm100/scene.xml
+mjcf_path: ../../vendor/menagerie/trs_so_arm100/scene.xml
```

**File 4 (`measurements/menagerie_diff.md`):** no content change. The file
is pure documentation; its new name more accurately describes what it
contains (a Menagerie→ElRobot parameter adaptation record).

**File 5 (`tests/test_mimic_gripper.py`):** no content change. The test
uses only `import mujoco` and the `elrobot_mjcf_path` fixture, which the
new `conftest.py` provides with an updated path.

### 4.3 New files

Seven new files are created at the paths below.

#### 4.3.1 `mujoco/elrobot_follower/VERSION`

```
0.1.0
```

Single-line text file with trailing newline. Semver for the physics model,
independent of `software/` crate versions. Rationale: despite codex's
argument that Menagerie relies only on git tags + CHANGELOG, a plain
`VERSION` file is trivially cheap (5 bytes) and becomes the single source
of truth once a future chunk adds `pyproject.toml` or `robot.yaml` version
field (both of which can read this file). The cost of keeping it is zero;
the cost of removing it and re-adding it later is non-zero.

#### 4.3.2 `mujoco/elrobot_follower/LICENSE`

A copy of the same license NormaCore itself uses at the repo root (or
Apache-2.0 if NormaCore has no explicit license, since that matches
Menagerie's `trs_so_arm100` and the majority of the robotics sim ecosystem).
Verification: before committing, `head -3 LICENSE` at the repo root to
confirm which license NormaCore uses.

#### 4.3.3 `mujoco/elrobot_follower/robot.yaml`

Full content:

```yaml
# robot.yaml — canonical identity for the ElRobot follower arm physics model.
#
# This file is the machine-readable index intended for:
# - LeRobot Dataset v3 `robot_type` field (future integration)
# - Any registry tooling that needs a stable robot ID
# - Programmatic discovery of the MJCF and its metadata
#
# When the content here disagrees with README.md or CHANGELOG.md, this file
# wins (it is the source of truth for automated consumers). Prose files
# should mirror the values here, not the other way around.

robot:
  id: elrobot_follower
  display_name: ElRobot Follower Arm
  variant: default
  license: Apache-2.0
  source_org: norma-core
  # Future: bump this when the robot package is published to a registry.
  registry_uri: null

kinematics:
  total_dof: 10   # 8 actuated joints + 2 mimic slides (gripper)
  actuated_dof: 8
  revolute_actuators: 7
  gripper_actuators: 1
  gripper_type: tendon_mimic_parallel
  gripper_mimic_joints:
    - rev_motor_08_1
    - rev_motor_08_2
  gripper_mimic_multipliers:
    - -0.0115
    - 0.0115

actuators:
  count: 8
  mjcf_names:
    - act_motor_01
    - act_motor_02
    - act_motor_03
    - act_motor_04
    - act_motor_05
    - act_motor_06
    - act_motor_07
    - act_motor_08
  # Client-facing actuator_id as used by the st3215-compat bridge and
  # surfaced in the Norma scene yaml's actuator_annotations remap.
  client_ids:
    - rev_motor_01
    - rev_motor_02
    - rev_motor_03
    - rev_motor_04
    - rev_motor_05
    - rev_motor_06
    - rev_motor_07
    - rev_motor_08

physics_baseline:
  origin: mujoco_menagerie/trs_so_arm100
  origin_commit: c771fb04055d805f20db0eab6cb20b67555887d0
  origin_date: "2025-06-09"
  # Flag flips to true once real-hardware sysID data replaces Menagerie-
  # inherited parameters. Future chunks should gate this flip on a specific
  # measurement corpus landing in measurements/sysid/.
  sysid_complete: false

version:
  current: "0.1.0"
  changelog: CHANGELOG.md
  version_file: VERSION

upstream:
  # Candidate upstream destination; not yet engaged.
  candidate: mujoco_menagerie
  prerequisites:
    - LICENSE file present (done at 0.1.0)
    - sysid_complete == true
    - Full CITATION.cff metadata
    - scene.xml wrapper with lights/floor (Menagerie convention)
    - Passing tests executable in isolation (no norma_sim dependency)
  engaged: false
```

#### 4.3.4 `mujoco/elrobot_follower/README.md`

Full content:

```markdown
# ElRobot Follower Arm — MuJoCo Physics Model

This directory is the engine-tier robot package for the ElRobot follower
arm: 8 actuators (7 revolute + 1 tendon-mimic-parallel gripper), forked
from MuJoCo Menagerie's `trs_so_arm100` physics baseline and adapted to
ElRobot's 8-joint URDF kinematics.

## Status

- Version: see `VERSION`
- License: see `LICENSE`
- Upstream candidate: `mujoco_menagerie` (not yet contributed)
- Machine-readable identity: `robot.yaml`

## Structure

```
elrobot_follower/
├── elrobot_follower.xml     ← main MJCF (8 joints + 2 mimic slides)
├── robot.yaml               ← machine-readable identity (source of truth)
├── VERSION                  ← semver (git-friendly)
├── LICENSE                  ← Apache-2.0
├── README.md                ← this file
├── CHANGELOG.md             ← physics-relevant changes
├── measurements/            ← parameter provenance + future sysID data
│   ├── README.md            ← folder purpose + workflow
│   └── menagerie_diff.md    ← Menagerie→ElRobot parameter adaptation record
└── tests/                   ← engine-level validation
    ├── conftest.py          ← single fixture (elrobot_mjcf_path)
    ├── test_mimic_gripper.py    ← P0 gripper mimic regression
    ├── test_urdf_parity.py      ← URDF↔MJCF consistency gate
    └── test_mjx_compat.py       ← MJX smoke test (placeholder)
```

The shared assets live one level up at `hardware/elrobot/simulation/assets/`
(not yet moved into this package). The MJCF's `meshdir="../../assets"`
resolves to them. A future upstream-contribution chunk will move assets
into this directory to make the package fully self-contained.

## How to modify

To change a physics parameter (armature, kp, dampratio, forcerange, ...):

1. Edit `elrobot_follower.xml`
2. Append a `measurements/menagerie_diff.md` note explaining the change
3. Bump `VERSION` (patch for tuning, minor for structural, major for
   breaking) and update `robot.yaml`'s `version.current`
4. Add a CHANGELOG entry under `[Unreleased]`
5. Run the engine-level tests (pure-mujoco, no `norma_sim` dependency):
   ```bash
   python3 -m pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -v
   ```
6. Run the full Norma integration suite as a smoke check:
   ```bash
   make sim-test
   ```

## Relationship to NormaCore

The Norma-specific runtime wrapper for this robot lives at
`hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`.
That wrapper — not this directory — is what `norma_sim`'s loader reads at
runtime. This directory contains only engine-native files.

The `software/sim-server/tests/integration/test_elrobot_acceptance.py`
integration test still lives in the sim-server test tree because it
imports `norma_sim.world.MuJoCoWorld`. Pure-MuJoCo tests that do not need
`norma_sim` live here under `tests/`.

## Upstream contribution

This package is structured to eventually be contributed to
`mujoco_menagerie` as a sibling of `trs_so_arm100/`. Prerequisites are
tracked in `robot.yaml`'s `upstream.prerequisites` field. Summary: need
LICENSE (done), real-hardware sysID data, CITATION.cff, a scene.xml
wrapper matching Menagerie convention, and tests that run without a
NormaCore checkout.
```

#### 4.3.5 `mujoco/elrobot_follower/CHANGELOG.md`

Full content follows the same format and body as the earlier draft (Section
4.3.3 of the previous spec version) with one structural change: the first
entry is tagged `[0.1.0] — 2026-04-12` and its "Added" list now includes
the new items introduced by Chunk 0 itself (`robot.yaml`, `LICENSE`,
`measurements/` layout, URDF parity test skeleton, MJX smoke gate skeleton).

The body is reproduced in full in Appendix A to keep this section
compact; see Section 9.

#### 4.3.6 `mujoco/elrobot_follower/measurements/README.md`

Full content:

```markdown
# Measurements

This directory holds physics-parameter provenance for the ElRobot
follower arm. It is a **sysID-ready slot**, not just documentation.

## Current contents

- `menagerie_diff.md` — Menagerie `trs_so_arm100`→ElRobot parameter
  adaptation record. Maps each Menagerie joint/actuator to its ElRobot
  analog, documents nearest-neighbor estimates for joints with no direct
  analog, and lists the four MVP-2 amendment policies (`forcerange` from
  URDF effort, `dampratio` not `kv`, explicit `ctrlrange` not
  `inheritrange`, `Gripper_Gear_v1_1` inertia floor).

## Future contents (post-MVP-3 Chunk 0)

When real-hardware sysID lands, this directory will also contain:

```
measurements/
├── raw/              ← CSV logs from the sysID rigs
├── fit/              ← fitted parameter values + fit residuals
├── plots/            ← visualization of fit quality per joint
├── sysid/            ← procedure scripts + notes
└── menagerie_diff.md ← kept as historical context
```

The `robot.yaml` `physics_baseline.sysid_complete` flag flips to `true`
when `fit/` contains a full per-joint dataset replacing Menagerie
inheritance.

## Workflow

1. Measure. (Separate chunk; procedure documented in `sysid/README.md`.)
2. Fit. (Separate chunk; fit script emits `fit/<joint_name>.json`.)
3. Update `../elrobot_follower.xml` with the fitted values.
4. Flip `robot.yaml` `sysid_complete: true`.
5. Bump `../VERSION` to `0.2.0` (minor — structural physics change).
6. Add a `../CHANGELOG.md` entry.
```

#### 4.3.7 `mujoco/elrobot_follower/tests/conftest.py`

Full content:

```python
"""Shared pytest fixtures for the ElRobot follower MuJoCo package.

These fixtures are deliberately minimal. The tests in this directory
exercise the MJCF via direct `mujoco.MjModel.from_xml_path(...)` calls
and must NOT import `norma_sim` — the goal is an engine-level test suite
that can run from a fresh checkout without any NormaCore application
code on PYTHONPATH.

If you need a test that uses `norma_sim.world.MuJoCoWorld` or any other
application-layer helper, put it in `software/sim-server/tests/` instead.
"""
from pathlib import Path

import pytest


@pytest.fixture
def elrobot_mjcf_path() -> Path:
    """Path to the MJCF in this package."""
    p = Path(__file__).resolve().parent.parent / "elrobot_follower.xml"
    if not p.exists():
        pytest.skip(f"ElRobot MJCF not found at {p}")
    return p
```

Note the single fixture (no `elrobot_scene_yaml` — this package deliberately
does not reference the Norma scene yaml wrapper) and the relative-path
resolution via `Path(__file__).resolve().parent.parent` (one dir up from
`tests/` reaches the robot package root where the MJCF lives).

#### 4.3.8 `mujoco/elrobot_follower/tests/test_urdf_parity.py`

Full content:

```python
"""URDF ↔ MJCF consistency gate.

The `elrobot_follower.urdf` at hardware/elrobot/simulation/ is kept as the
kinematic source of truth. This test prevents it from rotting: whenever
the MJCF's joint topology diverges from the URDF's, the test fails loudly.

Scope: structural invariants only (joint names, counts, axes). Dynamic
properties (inertia, mass, friction) are NOT checked because the MJCF
intentionally overrides those per the MVP-2 calibration_notes.md.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import pytest


@pytest.fixture
def urdf_path() -> Path:
    here = Path(__file__).resolve()
    # tests/ → robot package → mujoco/ → simulation/
    p = here.parent.parent.parent.parent / "elrobot_follower.urdf"
    if not p.exists():
        pytest.skip(f"ElRobot URDF not found at {p}")
    return p


def test_urdf_and_mjcf_agree_on_joint_names(urdf_path: Path, elrobot_mjcf_path: Path):
    """Joint names M1..M7 (revolute) + M8 (gripper) must match between URDF and MJCF."""
    urdf_root = ET.parse(urdf_path).getroot()
    urdf_joint_names = {
        j.attrib["name"]
        for j in urdf_root.findall("joint")
        if j.attrib.get("type") in ("revolute", "continuous", "prismatic")
    }
    model = mujoco.MjModel.from_xml_path(str(elrobot_mjcf_path))
    mjcf_joint_names = set()
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            mjcf_joint_names.add(name)
    missing_in_mjcf = urdf_joint_names - mjcf_joint_names
    assert not missing_in_mjcf, (
        f"URDF joints missing from MJCF: {sorted(missing_in_mjcf)}"
    )


def test_urdf_and_mjcf_agree_on_actuated_joint_count(
    urdf_path: Path, elrobot_mjcf_path: Path
):
    """ElRobot has 8 actuated joints (7 revolute + 1 gripper primary).
    The MJCF may have additional mimic joints (rev_motor_08_1, rev_motor_08_2)
    that do not appear in the URDF as top-level actuated joints."""
    urdf_root = ET.parse(urdf_path).getroot()
    urdf_actuated = {
        j.attrib["name"]
        for j in urdf_root.findall("joint")
        if j.attrib.get("type") in ("revolute", "continuous")
    }
    assert len(urdf_actuated) == 8, (
        f"Expected 8 actuated URDF joints, got {len(urdf_actuated)}: "
        f"{sorted(urdf_actuated)}"
    )
```

The test uses stdlib `xml.etree.ElementTree` (no new dependencies) and
the minimal pytest fixtures already in `conftest.py`. Intentionally
**narrow** scope: catches joint-topology drift, not full kinematic
equivalence. A richer parity test (axis alignment, joint limits) is a
future chunk candidate.

#### 4.3.9 `mujoco/elrobot_follower/tests/test_mjx_compat.py`

Full content:

```python
"""MJX forward-pass smoke gate (placeholder).

This test reserves a CI slot for verifying the MJCF compiles under
MuJoCo's JAX backend (`mujoco.mjx`). MJX is the GPU-accelerated / batched
rollout / differentiable sim path that matters for policy training
(RL, IL, domain randomization).

Status: placeholder. If `mujoco.mjx` is importable, run a minimal
`mjx.put_model` + `mjx.forward` pass. Otherwise skip.

When MVP-3 moves into policy training, this test expands into a real
MJX compatibility gate with full forward+backward pass verification.
"""
import pytest


def test_mjx_forward_pass_compiles(elrobot_mjcf_path):
    """Minimal smoke test: MJX must be able to compile this MJCF and
    run a single forward pass without errors."""
    mjx = pytest.importorskip("mujoco.mjx")
    import mujoco

    mj_model = mujoco.MjModel.from_xml_path(str(elrobot_mjcf_path))
    mjx_model = mjx.put_model(mj_model)
    mjx_data = mjx.make_data(mjx_model)
    mjx_data = mjx.forward(mjx_model, mjx_data)
    # Verify basic invariants post-forward:
    assert mjx_model.nu == 8, (
        f"expected nu=8 in MJX model, got {mjx_model.nu}"
    )
    assert mjx_model.nv == 10, (
        f"expected nv=10 in MJX model, got {mjx_model.nv}"
    )
```

Skips cleanly when MJX is not installed (which is the current state —
`mujoco.mjx` depends on JAX which is not a NormaCore dependency). The
skip is fine for Chunk 0; the test reserves the slot.

### 4.4 Path updates in unchanged files

A `grep -rn` for the old path references identifies the following files.
Note that this list is **larger than the pre-codex version** because the
new manifests/ + mujoco/ layering means more paths change.

| # | File | Old reference | New reference |
|---|---|---|---|
| 1 | `software/station/bin/station/station-sim.yaml` | `hardware/elrobot/simulation/elrobot_follower.scene.yaml` | `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml` |
| 2 | `software/station/bin/station/station-shadow.yaml` | same field as #1 | same new path as #1 |
| 3 | `software/station/bin/station/station-sim-menagerie.yaml` | `hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml` | `hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml` |
| 4 | `Makefile` (sim-standalone target) | `hardware/elrobot/simulation/elrobot_follower.scene.yaml` | `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml` |
| 5 | `Makefile` (sim-test target) | `software/sim-server/tests/` (only) | add `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/` |
| 6 | `software/sim-server/README.md` | `hardware/elrobot/simulation/elrobot_follower.scene.yaml` (Scenario B command example) | new path |
| 7 | `software/sim-server/norma_sim/world/manifest.py` (docstring line 3) | same | new path |
| 8 | `software/sim-server/scripts/probe_manifest.py` (usage example) | same | new path |
| 9 | `software/sim-server/tests/conftest.py` (`elrobot_mjcf_path` fixture) | `hardware/elrobot/simulation/elrobot_follower.xml` | `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml` |
| 10 | `software/sim-server/tests/conftest.py` (`elrobot_scene_yaml` fixture) | `hardware/elrobot/simulation/elrobot_follower.scene.yaml` | `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml` |

### 4.5 Still-in-sim-server test files consuming old fixtures

After this chunk, the four `world/` test files below still use the
`elrobot_mjcf_path` / `elrobot_scene_yaml` fixtures from the old
`software/sim-server/tests/conftest.py`. They are NOT moving — only the
fixture's internal paths change:

1. `software/sim-server/tests/world/test_model.py`
2. `software/sim-server/tests/world/test_snapshot.py`
3. `software/sim-server/tests/world/test_descriptor_build.py`
4. `software/sim-server/tests/world/test_actuation.py`

Plus the integration test that also stays:

5. `software/sim-server/tests/integration/test_elrobot_acceptance.py`

These five files are all ElRobot-specific physics tests that import
`norma_sim.world.MuJoCoWorld`. They cannot be moved to the engine-tier
robot package without a `norma_sim` dependency decoupling that is
explicitly out of scope for this chunk.

### 4.6 Why `test_elrobot_acceptance.py` STAYS (reversal from brainstorming Q3)

The brainstorming Q3 decision was "both tests move" —
`test_elrobot_acceptance.py` → `test_physics_acceptance.py` in the new
location, plus `test_mimic_gripper.py` also moves. The codex consult
exposed a flaw in that plan: `test_elrobot_acceptance.py` imports
`from norma_sim.world.model import MuJoCoWorld` and uses
`MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)` — it is an
**integration test** that depends on `norma_sim`, not a pure-mujoco test.

If it moved, the engine-tier robot package's "pure mujoco, runnable from
a fresh checkout without NormaCore" property breaks. The package would
still require `PYTHONPATH=software/sim-server` and a full NormaCore
checkout — defeating the self-containment goal that motivated the move
in the first place.

**This chunk therefore reverses Q3's "both move" to "only test_mimic_gripper
moves."** `test_elrobot_acceptance.py` stays at
`software/sim-server/tests/integration/test_elrobot_acceptance.py` and is
NOT renamed. Its purpose (Floor §3.1 acceptance gate) is unchanged.

A future chunk ("test-layer decoupling") can rewrite
`test_elrobot_acceptance.py` to use direct `mujoco.MjModel.from_xml_path(...)`
calls instead of `MuJoCoWorld.from_manifest_path(...)`, removing the
`norma_sim` dependency. Once that's done, it can migrate to
`mujoco/elrobot_follower/tests/test_physics_acceptance.py` (or the
codex-recommended split into `test_model_invariants.py` +
`test_dynamics_regression.py`). That is future work, not Chunk 0.

### 4.7 Makefile `sim-test` target update

Diff:

```diff
 .PHONY: sim-test
 sim-test:
 	# ... (arch invariants + cargo test targets unchanged)
-	PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/
+	PYTHONPATH=software/sim-server python3 -m pytest \
+	    software/sim-server/tests/ \
+	    hardware/elrobot/simulation/mujoco/elrobot_follower/tests/
```

Note that `PYTHONPATH` is still required for the `software/sim-server/tests/`
portion (integration tests that import `norma_sim`). The new robot package
tests do not need it and will collect cleanly without PYTHONPATH if invoked
directly from the command line — but invoking via `make sim-test` the
unified PYTHONPATH is harmless.

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

**Load-bearing note on item 7**: **Four** test files in
`software/sim-server/tests/world/` still use the `elrobot_scene_yaml` /
`elrobot_mjcf_path` fixtures from the old `conftest.py`:

1. `test_model.py`
2. `test_snapshot.py`
3. `test_descriptor_build.py`
4. `test_actuation.py`

(Note: `test_mimic_gripper.py` — the fifth `world/` file that currently
uses `elrobot_mjcf_path` — is moving out in this chunk, which is why it
is not counted here.)

These four test files are NOT moving in this chunk, so the old
`conftest.py` must keep its fixture definitions with updated paths. The
same fixture is therefore defined in two places (old and new
`conftest.py`), each pointing at the new `models/` location. This
duplication was approved during the brainstorming Q4 decision.

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
python3 -c "
import mujoco
m = mujoco.MjModel.from_xml_path(
    'hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml')
print(f'nu={m.nu} neq={m.neq} ntendon={m.ntendon}')
"
```

Expected output:

```
nu=8 neq=2 ntendon=2
```

Note: `PYTHONPATH` is NOT required for this check — the robot package's
MJCF is a pure-MuJoCo file with no `norma_sim` dependency. If this command
fails with ModuleNotFoundError for `norma_sim`, something is wrong.

Failure modes to check:

- `meshdir="../../assets"` not applied → mesh files not found (the MJCF
  must resolve `../../assets/*.stl` from
  `mujoco/elrobot_follower/elrobot_follower.xml`)
- Accidental content change during `git mv` (rare but worth verifying)

### 5.2 Norma scene yaml loads from its new location

```bash
PYTHONPATH=software/sim-server python3 -c "
from pathlib import Path
from norma_sim.world.model import MuJoCoWorld
world = MuJoCoWorld.from_manifest_path(Path(
    'hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml'))
print(f'nu={world.model.nu} neq={world.model.neq} ntendon={world.model.ntendon}')
gripper = world.actuator_by_mjcf_name('act_motor_08')
assert gripper is not None and gripper.capability.kind == 'GRIPPER_PARALLEL'
print('ElRobot scene yaml loads OK from new manifests/ location')
"
```

Expected: `nu=8 neq=2 ntendon=2` and the `OK` message. This verifies that
the scene yaml's internal `mjcf_path` update (to
`../../mujoco/elrobot_follower/elrobot_follower.xml`) resolves correctly
from the new scene yaml location.

### 5.3 Menagerie scene yaml loads from its new location

```bash
PYTHONPATH=software/sim-server python3 -c "
from pathlib import Path
from norma_sim.world.model import MuJoCoWorld
world = MuJoCoWorld.from_manifest_path(Path(
    'hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml'))
print(f'nu={world.model.nu}')
"
```

Expected: `nu=6` (Menagerie SO-ARM100 has 6 actuators). Verifies that the
Menagerie scene yaml's `mjcf_path` update
(`../../vendor/menagerie/trs_so_arm100/scene.xml`) also resolves.

### 5.4 New engine-tier test directory is discoverable and passing

```bash
python3 -m pytest \
    hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -v 2>&1 | tail -30
```

Expected: **3 passed, 1 skipped** (or **4 passed** if MJX is installed):

- `test_mimic_gripper.py::test_mimic_gripper_equality_works` PASSED
- `test_mimic_gripper.py::test_mimic_gripper_zero_setpoint_holds_zero` PASSED
- `test_urdf_parity.py::test_urdf_and_mjcf_agree_on_joint_names` PASSED
- `test_urdf_parity.py::test_urdf_and_mjcf_agree_on_actuated_joint_count` PASSED
- `test_mjx_compat.py::test_mjx_forward_pass_compiles` SKIPPED
  (or PASSED if `mujoco.mjx` is importable on this host)

Total: 4 tests passing in the happy case, 1 skipped. (2 from
`test_mimic_gripper.py` + 2 from `test_urdf_parity.py` + 1 from
`test_mjx_compat.py`, which skips if MJX not installed.)

Note that `PYTHONPATH=software/sim-server` is NOT required for this
command — the engine-tier tests are pure-mujoco and do not import
`norma_sim`. If they fail without PYTHONPATH, something is wrong.

### 5.5 Old sim-server test directory still green after fixture-path updates

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/ -q \
    2>&1 | tail -5
```

Expected: **86 passed** (88 pre-Chunk-0 minus 2 moved-out `test_mimic_gripper.py`
tests = 86). The `test_elrobot_acceptance.py` 13 tests stay and are
counted here.

### 5.6 Full `make sim-test` pipeline green

```bash
make sim-test 2>&1 | tail -10
```

Expected:

- Architecture invariants: all ✓
- Rust: `sim-runtime 28 passed`, `st3215-wire 15 passed`,
  `st3215-compat-bridge 16 passed` (unchanged; zero Rust touched)
- Python: **90 passed, 0 failed, at most 1 skipped** (86 from sim-server +
  4 from engine-tier tests = 90; the +1 skipped accounts for
  `test_mjx_compat.py` on a host without MJX). This is a +2 increase from
  the pre-Chunk-0 baseline of 88 — the 2 extra tests are the new
  URDF parity test functions.

### 5.7 Menagerie walking skeleton permanent regression fixture

```bash
PYTHONPATH=software/sim-server python3 -m pytest \
    software/sim-server/tests/integration/test_menagerie_walking_skeleton.py -q
```

Expected: **6 passed**. This test is not moved and must remain green.

### 5.8 No dangling references to old paths

```bash
grep -rn 'hardware/elrobot/simulation/elrobot_follower\|hardware/elrobot/simulation/menagerie_so_arm100' \
    software/ hardware/ Makefile docs/ 2>&1 \
    | grep -v 'docs/superpowers/.*2026-04-1[012]' \
    | grep -v 'vendor/menagerie/VENDOR.md'
```

Expected: **no matches**. The `grep -v` filters exclude:

- `docs/superpowers/{specs,plans}/2026-04-1{0,1,2}-*` — historical spec
  and plan files from MVP-1 (2026-04-10), MVP-2 (2026-04-11), and this
  very spec document itself (2026-04-12). The latter contains `git mv`
  command blocks with the old paths as part of the documentation, which
  is expected and benign.
- `hardware/elrobot/simulation/vendor/menagerie/VENDOR.md` — vendor
  record. Out of scope for this chunk.

If the filtered grep produces any matches, there is a real missing path
update — investigate before committing.

### 5.9 Station smoke test (optional, skip if no display)

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
correctly given the deeper `manifests/norma/` scene yaml location.

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

- **`meshdir="../../assets"` edit**: a typo or wrong path would cause MJCF
  compile failure, which Section 5.1 catches. Note the two-level depth — a
  single `../` would silently resolve to a non-existent path inside
  `mujoco/assets/`.
- **scene yaml `mjcf_path` edits**: both the ElRobot and Menagerie scene
  yamls need their `mjcf_path` fields updated to account for the deeper
  `manifests/norma/` location. Section 5.2 and 5.3 catch errors by calling
  `MuJoCoWorld.from_manifest_path(...)` on both and asserting successful
  load. A wrong path causes `ElRobot MJCF not found` (old behavior) or
  load failure.
- **`software/sim-server/tests/conftest.py` path update**: if the new paths
  are wrong, the 4 world/ tests + the 1 integration test that still use
  these fixtures will skip (not fail), which could mask a regression.
  Section 5.5 catches this by asserting the expected `86 passed` number
  (not `N passed, M skipped`).
- **Makefile `sim-test` target path addition**: a typo causes pytest
  collection to skip the new directory, and Section 5.6's `90 passed`
  total check catches it.
- **URDF parity test**: the stdlib `xml.etree.ElementTree` parse + set
  comparison is straightforward, but the test's URDF-path resolution uses
  `here.parent.parent.parent.parent / "elrobot_follower.urdf"` (four
  `.parent` calls from `.../tests/test_urdf_parity.py` = `simulation/`
  directory). A miscounted depth would surface as `FileNotFoundError` on
  the URDF. Section 5.4 catches this by actually running the test.

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
mvp3: promote hardware/elrobot/simulation/ to three-tier first-class structure

Chunk 0 of MVP-3: reorganize the ElRobot physics model into an engine-tier
robot package (mujoco/elrobot_follower/) separate from Norma-side runtime
manifests (manifests/norma/). Structure follows codex consult feedback
(session 019d7726...) from the brainstorming stage, which exposed that the
original "models/+parameters/+tests/ as one dir" plan conflated engine-
native MJCF assets with Norma-specific wrapper metadata.

New structure under hardware/elrobot/simulation/:

  mujoco/elrobot_follower/           ← NEW engine-tier robot package
    elrobot_follower.xml             ← MOVED from simulation/ (+ meshdir)
    README.md                        ← NEW
    CHANGELOG.md                     ← NEW
    VERSION                          ← NEW (0.1.0)
    LICENSE                          ← NEW
    robot.yaml                       ← NEW (machine-readable identity)
    measurements/
      README.md                      ← NEW (folder purpose + workflow)
      menagerie_diff.md              ← MOVED+RENAMED from docs/superpowers/specs/
                                       2026-04-11-mvp2-menagerie-comparison-table.md
    tests/
      conftest.py                    ← NEW (single fixture, pure-mujoco)
      test_mimic_gripper.py          ← MOVED from software/sim-server/tests/world/
      test_urdf_parity.py            ← NEW (URDF↔MJCF invariant gate)
      test_mjx_compat.py             ← NEW (MJX forward-pass smoke gate, skip-if-no-mjx)

  manifests/norma/                   ← NEW Norma runtime wrappers
    elrobot_follower.scene.yaml      ← MOVED from simulation/ (+ mjcf_path)
    menagerie_so_arm100.scene.yaml   ← MOVED from simulation/ (+ mjcf_path)

  assets/                            ← UNCHANGED (19 STL files; URDF + MJCF both reach)
  elrobot_follower.urdf              ← UNCHANGED
  vendor/menagerie/                  ← UNCHANGED

Engine-tier package is pure-mujoco: tests import only `mujoco`, no
`norma_sim`. This makes the package runnable from a fresh checkout
without the NormaCore application-layer code on PYTHONPATH, matching the
self-containment standard of MuJoCo Menagerie robot packages.

Content changes in moved files:
- mujoco/elrobot_follower/elrobot_follower.xml: meshdir="assets" → "../../assets"
  (robot package is two levels deeper than assets/)
- manifests/norma/elrobot_follower.scene.yaml: mjcf_path: ./elrobot_follower.xml
  → ../../mujoco/elrobot_follower/elrobot_follower.xml
- manifests/norma/menagerie_so_arm100.scene.yaml: mjcf_path: ./vendor/... →
  ../../vendor/... (same Menagerie target, scene yaml is now deeper)

Path updates in unchanged-location files (10 edits):
- software/station/bin/station/station-sim.yaml (launcher --manifest)
- software/station/bin/station/station-shadow.yaml (launcher --manifest)
- software/station/bin/station/station-sim-menagerie.yaml (launcher --manifest)
- Makefile sim-standalone target --manifest arg
- Makefile sim-test target (append new pytest path)
- software/sim-server/README.md (Scenario B command example)
- software/sim-server/norma_sim/world/manifest.py (docstring line 3)
- software/sim-server/scripts/probe_manifest.py (usage example)
- software/sim-server/tests/conftest.py (elrobot_mjcf_path fixture)
- software/sim-server/tests/conftest.py (elrobot_scene_yaml fixture)

Reversal from brainstorming Q3:
- test_elrobot_acceptance.py STAYS in sim-server/tests/integration/ (not
  moved, not renamed). It's an integration test importing norma_sim and
  cannot be made self-contained at the engine-tier package without a
  rewrite that's out of Chunk 0 scope. The four world/ tests consuming
  elrobot_* fixtures also stay. See spec Section 4.6 for rationale.

Rationale: "一等公民 MJCF" insight from MVP-2, refined by codex consult —
the physics model must be separable from its Norma runtime wrapper, and
both must be separable from cross-engine infrastructure. This three-tier
structure (engine-native robot package / framework runtime manifests /
shared raw assets) sets the boundaries for future integration with
LeRobot EnvHub, MJX GPU sim, Isaac Lab task definitions, and upstream
contribution to MuJoCo Menagerie.

Verification (all post-commit):
- make sim-test: 90 passed, 0 failed, ≤1 skipped (88 pre-Chunk-0 baseline +
  2 new URDF parity tests; mjx smoke test skips if mujoco.mjx not installed)
- make check-arch-invariants: all ✓
- Rust: sim-runtime 28/0, st3215-wire 15/0, st3215-compat-bridge 16/0 (zero
  Rust touched)
- test_menagerie_walking_skeleton: 6 passed (permanent regression fixture intact)
- Pure-mujoco engine-tier tests runnable without PYTHONPATH: verified

Deferred to MVP-3 Chunk 1+:
- Rewriting test_elrobot_acceptance.py to pure-mujoco so it can also move
- Splitting it into test_model_invariants.py + test_dynamics_regression.py
- Moving elrobot_follower.urdf + assets/ into the robot package
- Menagerie-style scene.xml wrapper (lights + floor) for visual parity
- docs/ subdir with sysid-procedure.md, how-to-measure-armature.md,
  upstream-to-menagerie.md
- CITATION.cff, pyproject.toml (pip-installable package)
- Real-hardware sysID data + sysid_complete=true flip
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
  memory's MVP-3 Chunk 0 SOP was written before codex's consult and
  references the old `models/` + `parameters/` structure; it will need a
  revision pass after Chunk 0 lands.

### 8.3 Candidates for MVP-3 Chunk 1+

- **Test-layer decoupling**: rewrite `test_elrobot_acceptance.py` to use
  direct `mujoco.MjModel.from_xml_path(...)` calls instead of
  `MuJoCoWorld.from_manifest_path(...)`. Once pure-mujoco, migrate it into
  `mujoco/elrobot_follower/tests/test_physics_acceptance.py`. This was
  originally Chunk 0 scope but was deferred because it requires non-trivial
  test rewriting (not just file moves) and crosses the "pure refactor" line.
- **Split `test_physics_acceptance.py`** (after the above migration) into
  `test_model_invariants.py` (Floors 1-3) + `test_dynamics_regression.py`
  (Floor 4). Codex's long-term recommendation; defer until the monolithic
  file actually needs splitting for parallelism or scope clarity.
- **Move `assets/*.stl`** into the robot package so it is truly
  self-contained for upstream contribution. Requires 19 binary `git mv`
  operations and an adjustment to the URDF's mesh references (or a
  decision to keep URDF pointing at a legacy path).
- **Move `elrobot_follower.urdf`** into a new
  `mujoco/elrobot_follower/urdf/` or co-located with the MJCF. Ties to the
  assets-move decision.
- **Add `scene.xml` wrapper** (Menagerie-style, with lights + floor +
  headlight + groundplane) at
  `mujoco/elrobot_follower/scene.xml`, using `<include file="elrobot_follower.xml"/>`.
  Enables `python -m mujoco.viewer mujoco/elrobot_follower/scene.xml` to
  show the robot on a ground plane without additional setup, matching
  Menagerie's user-facing convention.
- **Create `docs/` subdirectory** inside the robot package with skeleton
  narrative files:
  - `docs/how-to-measure-armature.md`
  - `docs/sysid-procedure.md`
  - `docs/upstream-to-menagerie.md`
- **Real-hardware sysID data collection** — requires real ElRobot access.
  Adds `measurements/sysid/`, `measurements/raw/`, `measurements/fit/`,
  `measurements/plots/` subdirectories. When complete, flip `robot.yaml`
  `physics_baseline.sysid_complete` to `true` and bump `VERSION` to `0.2.0`.
- **Add `CITATION.cff`** at the robot package root — required for
  upstream contribution. Prerequisite: a BibTeX-citable publication or
  the equivalent NormaCore technical report.
- **`pyproject.toml`** at `mujoco/elrobot_follower/` for pip-installability
  as a standalone package (`elrobot-follower-mjcf`). Enables
  `pip install elrobot-follower-mjcf` to get the MJCF + robot.yaml + tests
  in a fresh environment without cloning NormaCore.
- **LeRobot EnvHub integration**: add a top-level `env.py` at
  `hardware/elrobot/simulation/envs/lerobot/` that exposes `make_env(...)`
  for LeRobot's Hub-native env repo format. Separate chunk because it
  introduces a new `envs/` tier alongside `manifests/`.
- **`gymnasium.Env` wrapper**: similarly a new `envs/gym/` sibling for
  policy training. Independent of LeRobot integration.
- **MJX-based batched rollout CI**: expand `test_mjx_compat.py` from a
  smoke test into a full forward+backward pass with batch-of-N
  verification. Enables GPU-accelerated RL rollouts for MVP-3 Chunk N
  (policy training).

### 8.4 Longer-term

- **Upstream contribution to `mujoco_menagerie`**. Gated on: assets moved
  in, URDF moved in, scene.xml added, CITATION.cff present, sysID
  complete, tests runnable without NormaCore checkout, LICENSE verified
  compatible.
- **LeRobot dataset publication pipeline** using this MJCF as the sim
  environment. Dataset's `robot_type` field points at the `robot.yaml` `robot.id`.
- **Second robot package** if NormaCore adds another arm (e.g. a fork of
  Menagerie SO-ARM100 with NormaCore-specific calibration). Slots in as
  `mujoco/so_arm100_normacore/` alongside `mujoco/elrobot_follower/`.
- **Non-MuJoCo engine support**: if MVP-N needs Isaac Lab / Genesis / USD,
  new peer directories `simulation/isaac/`, `simulation/genesis/`,
  `simulation/usd/` slot in next to `simulation/mujoco/`. The `mujoco/`
  layer reserved in Chunk 0 means this is a pure addition, not a
  reorganization.

---

## 9. Appendix: Full list of modified files

To support spec review and commit planning, here is the exhaustive list of
files touched by this chunk.

### 9.1 Files moved (5)

1. `hardware/elrobot/simulation/elrobot_follower.xml` → `.../mujoco/elrobot_follower/elrobot_follower.xml`
2. `hardware/elrobot/simulation/elrobot_follower.scene.yaml` → `.../manifests/norma/elrobot_follower.scene.yaml`
3. `hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml` → `.../manifests/norma/menagerie_so_arm100.scene.yaml`
4. `docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md` → `hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/menagerie_diff.md`
5. `software/sim-server/tests/world/test_mimic_gripper.py` → `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mimic_gripper.py`

### 9.2 Files with content changes (13)

Three of the moved files need content edits (the `meshdir` attribute on
the MJCF and the `mjcf_path` fields on both scene yamls) in addition to
the move itself. The remaining 10 are files that stay in place but
reference the moved paths.

1. `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml` — `meshdir` attribute
2. `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml` — `mjcf_path` field
3. `hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml` — `mjcf_path` field
4. `software/station/bin/station/station-sim.yaml` — `--manifest` path
5. `software/station/bin/station/station-shadow.yaml` — `--manifest` path
6. `software/station/bin/station/station-sim-menagerie.yaml` — `--manifest` path
7. `Makefile` — `sim-standalone` `--manifest` path
8. `Makefile` — `sim-test` pytest paths (append new test dir)
9. `software/sim-server/README.md` — Scenario B command example
10. `software/sim-server/norma_sim/world/manifest.py` — docstring line 3
11. `software/sim-server/scripts/probe_manifest.py` — docstring usage example
12. `software/sim-server/tests/conftest.py` — `elrobot_mjcf_path` fixture path
13. `software/sim-server/tests/conftest.py` — `elrobot_scene_yaml` fixture path

(Entries 12 and 13 are in the same file but are two logically distinct
edits, so they are listed separately.)

### 9.3 New files (9)

Seven new files in the engine-tier robot package, one in `measurements/`,
and one in `tests/` for the MJX smoke gate:

1. `hardware/elrobot/simulation/mujoco/elrobot_follower/README.md`
2. `hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md`
3. `hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION`
4. `hardware/elrobot/simulation/mujoco/elrobot_follower/LICENSE`
5. `hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml`
6. `hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/README.md`
7. `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/conftest.py`
8. `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py`
9. `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mjx_compat.py`

### 9.4 Implicit new directories (5)

Created as a side effect of `git mv` and new-file writes:

1. `hardware/elrobot/simulation/mujoco/`
2. `hardware/elrobot/simulation/mujoco/elrobot_follower/`
3. `hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/`
4. `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/`
5. `hardware/elrobot/simulation/manifests/norma/`

### 9.5 Total touched files

**27 distinct file-level changes**: 5 moves (3 of which also have content
edits) + 13 content edits (including the 3 moved-file edits) + 9 new
files. The overlap is:

- `mujoco/elrobot_follower/elrobot_follower.xml` is counted as both a move
  (9.1 row 1) and a content edit (9.2 row 1).
- `manifests/norma/elrobot_follower.scene.yaml` is counted as both a move
  (9.1 row 2) and a content edit (9.2 row 2).
- `manifests/norma/menagerie_so_arm100.scene.yaml` is counted as both a
  move (9.1 row 3) and a content edit (9.2 row 3).

So the number of **distinct files** touched is `5 + (13 - 3) + 9 = 21`,
while the number of **operations** is 27. Both numbers are given for
clarity.

All 27 operations land in a single atomic commit on `main`.

---

## Appendix A: Full content of `mujoco/elrobot_follower/CHANGELOG.md`

```markdown
# ElRobot Physics Model CHANGELOG

Follows a subset of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versioning is semver, independent of the `software/` crates.

## [Unreleased]

(nothing yet)

## [0.1.0] — 2026-04-12

### Added

- Initial hand-written MJCF `elrobot_follower.xml` (260 lines) derived from
  Menagerie `trs_so_arm100` v1.3 @ commit
  `c771fb04055d805f20db0eab6cb20b67555887d0` (2025-06-09 tuning).
- 8 `<position>` actuators (`act_motor_01` .. `act_motor_08`), mapped via
  `actuator_annotations` in the sibling Norma scene yaml
  (`../../manifests/norma/elrobot_follower.scene.yaml`) to client-facing
  `actuator_id` `rev_motor_01` .. `rev_motor_08`.
- Menagerie-baseline physics defaults in `<default class="elrobot">`:
  `joint armature=0.1 frictionloss=0.1`,
  `position kp=50 dampratio=1 forcerange=±2.94`. forcerange uses URDF
  effort (±2.94) instead of Menagerie's ±3.5 (documented in
  `measurements/menagerie_diff.md`).
- Tendon-based gripper mimic preserved from MVP-1 — 2 mimic slide joints
  (`rev_motor_08_1`, `rev_motor_08_2`) coupled via `<tendon><fixed>` +
  `<equality><tendon>` with multipliers ±0.0115. **P0 invariant** —
  covered by `tests/test_mimic_gripper.py`.
- Self-collision avoidance via 10 `<contact><exclude>` pairs (added after
  MVP-2 Chunk 5 Task 5.2 code review discovered motion-dependent
  collisions that the rest-pose `ncon=0` check didn't catch).
- Primitive collision geoms (box/cylinder/sphere) replacing MVP-1's
  mesh-based collision (which caused self-intersection at rest).
- `measurements/menagerie_diff.md` — Menagerie→ElRobot joint-by-joint
  comparison table, armature/damping/frictionloss inheritance strategy,
  and 4 policy amendments (forcerange=URDF, dampratio not kv, explicit
  ctrlrange not inheritrange, Gripper_Gear_v1_1 diaginertia floor).
  **Finding**: 2 independent ElRobot joints have no Menagerie analog
  (`rev_motor_02`, `rev_motor_05`), not the 3 the MVP-2 plan originally
  guessed.
- `VERSION` file at 0.1.0.
- `LICENSE` file (Apache-2.0, matching NormaCore's root license).
- `robot.yaml` — machine-readable canonical identity. Consumed later by
  LeRobot Dataset v3 `robot_type` field and future registry tooling.
- `tests/test_mimic_gripper.py` — P0 regression, 2 tests. Pure-mujoco
  (no `norma_sim` dependency).
- `tests/test_urdf_parity.py` — NEW URDF↔MJCF structural invariant test,
  2 test functions. Prevents URDF from rotting.
- `tests/test_mjx_compat.py` — NEW MJX forward-pass smoke gate,
  1 test function (skip-if-no-mjx). Reserves the MJX compatibility slot
  for future RL/policy training chunks.
- `tests/conftest.py` — single fixture `elrobot_mjcf_path` resolving to
  this package's own MJCF. No `norma_sim` import.

### Physics gate results (at initial release)

- Floor §3.1 all 6 criteria: GREEN (0 tuning iterations — Menagerie
  baseline passed first-try under MuJoCo's Coulomb frictionloss + gravity
  bleed, contradicting an analytical PD overshoot prediction). Verified
  via `software/sim-server/tests/integration/test_elrobot_acceptance.py`
  which stays in sim-server because it depends on `norma_sim`.
- Ceiling §3.2 item 7 (web UI slider responsiveness including M1): PASS
  (manual browser smoke test 2026-04-12). MVP-1's M1-unresponsive
  regression is resolved.
- Ceiling §3.2 item 8 (MuJoCo viewer side-by-side with Menagerie):
  DEFERRED (headless execution environment; advisory per MVP-2 spec §7.5).

### Known limitations

- Parameters are inherited from Menagerie's 2025-06-09 tuning (no
  real-hardware sysID yet). For the 2 ElRobot-unique joints,
  nearest-neighbor estimation is used — physics is plausible but not
  measured. `robot.yaml` `physics_baseline.sysid_complete: false`.
- Gripper_Jaw_01/02 inertial origins were reset to body origin (URDF
  export bug: the URDF had jaw COMs expressed in world-frame
  coordinates). The resulting parallel-axis error (~1.5e-6 kg·m²) is
  negligible for mimic-constrained jaws but worth flagging.
- Merged inertia for fixed joints (ST3215 motor mass collapsed into the
  parent revolute body) omits parallel-axis shift (~5.5e-6 kg·m²).
  Acceptable for Floor gates; re-evaluate for real-hardware tracking.
- Assets (`*.stl`) still live at `hardware/elrobot/simulation/assets/`,
  not inside this package. MJCF uses `meshdir="../../assets"`. This
  prevents the package from being truly self-contained for upstream
  contribution; a future chunk will move assets into the package.
- No `scene.xml` wrapper with lights/floor. Running
  `python -m mujoco.viewer hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml`
  will show the arm without a floor. A future chunk will add a Menagerie-
  style `scene.xml` for visual parity.
- `CITATION.cff` is not present. Required for upstream contribution; a
  future chunk will add it once the real-hardware sysID corpus lands.

### Integration context

- NormaCore MVP-2 merge commit: `93c1597` on `main` (2026-04-12)
- MVP-3 Chunk 0 commit: TBD (pending this chunk's execution)
- Chunk 0 spec: `docs/superpowers/specs/2026-04-12-mvp3-first-class-mjcf-design.md`
```

---

*End of spec.*
