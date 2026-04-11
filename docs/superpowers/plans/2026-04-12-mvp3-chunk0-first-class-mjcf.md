# MVP-3 Chunk 0: First-Class MJCF Directory Structure Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Promote `hardware/elrobot/simulation/` to a three-tier first-class directory structure — engine-tier robot package under `mujoco/elrobot_follower/`, Norma runtime wrappers under `manifests/norma/`, shared raw assets unchanged — in a single atomic commit on `main`.

**Architecture:** The reorganization separates three tangled concerns in the pre-Chunk-0 layout: (1) engine-native MuJoCo assets that are candidate for upstream contribution to `mujoco_menagerie`, (2) Norma-specific runtime manifests (`.scene.yaml` wrappers), and (3) shared raw assets. The `mujoco/` layer reserves a slot for future Isaac Lab / Genesis / USD peer directories. This is a pure organizational refactor — no physics changes, no Rust changes, no new behavior. A single machine-readable `robot.yaml` becomes the canonical identity source for future LeRobot Dataset v3 / registry tooling integration.

**Tech Stack:** Git (for moves + atomic commit), Bash (for directory creation + verification), Python 3 + MuJoCo (for compile + test verification), `mujoco.mjx` (optional, for MJX smoke gate), `xml.etree.ElementTree` (stdlib, for URDF parity test).

---

## Reference Documents (READ THESE FIRST)

Before starting, the implementer must read:

1. **The spec**: `docs/superpowers/specs/2026-04-12-mvp3-first-class-mjcf-design.md` (1503 lines)
   - Section 3: target directory structure (the authoritative tree)
   - Section 4: file operations (the authoritative list of moves, edits, new files, path updates)
   - Section 5: verification strategy (the authoritative post-execution check list)
   - Section 9: appendix with full content of all new files
2. **Memory insight**: `~/.claude/projects/-home-yuan-proj-norma-core/memory/sim_starting_point.md`, specifically the "🌟 一等公民 MJCF 洞察" section — the original motivation.
3. **MVP-2 context**: the completed MVP-2 work merged at commit `93c1597` on `main` is the starting state. `hardware/elrobot/simulation/` currently contains the pre-Chunk-0 flat layout.

---

## Starting State Verification

Before beginning, confirm:

```bash
cd /home/yuan/proj/norma-core
git branch --show-current   # must print: main
git log --oneline -1         # must start with: b80b6c6 or newer (spec commits)
git status --short           # must be empty (clean tree) OR only have expected untracked
make sim-test 2>&1 | tail -3 # must show: 88 passed (the MVP-2 post-merge baseline)
```

If any of these fail, stop and investigate. Do NOT proceed with the restructure on an unclean baseline.

---

## File Structure Overview

**What exists today** (at `hardware/elrobot/simulation/`):

```
hardware/elrobot/simulation/
├── assets/                         (19 STL files)
├── elrobot_follower.scene.yaml     ← will move + edit mjcf_path
├── elrobot_follower.urdf           (unchanged)
├── elrobot_follower.xml            ← will move + edit meshdir
├── menagerie_so_arm100.scene.yaml  ← will move + edit mjcf_path
└── vendor/menagerie/               (unchanged)
```

**What exists elsewhere that this plan touches**:

```
docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md    ← will move + rename
software/sim-server/tests/world/test_mimic_gripper.py                   ← will move
software/sim-server/tests/conftest.py                                   ← elrobot_* fixtures get path update
software/sim-server/norma_sim/world/manifest.py                         ← docstring path update
software/sim-server/scripts/probe_manifest.py                           ← docstring path update
software/sim-server/README.md                                           ← Scenario B example path update
software/station/bin/station/station-sim.yaml                           ← launcher --manifest path update
software/station/bin/station/station-sim-menagerie.yaml                 ← launcher --manifest path update
software/station/bin/station/station-shadow.yaml                        ← launcher --manifest path update
Makefile                                                                ← sim-standalone --manifest + sim-test pytest paths
```

**What the end state looks like**:

```
hardware/elrobot/simulation/
├── mujoco/                                      ← NEW
│   └── elrobot_follower/                        ← NEW robot package
│       ├── elrobot_follower.xml                 (MOVED; meshdir="../../assets")
│       ├── README.md                            (NEW)
│       ├── CHANGELOG.md                         (NEW)
│       ├── VERSION                              (NEW; "0.1.0")
│       ├── LICENSE                              (NEW; copied from repo root)
│       ├── robot.yaml                           (NEW; machine-readable identity)
│       ├── measurements/
│       │   ├── README.md                        (NEW)
│       │   └── menagerie_diff.md                (MOVED+RENAMED from docs/superpowers/specs/)
│       └── tests/
│           ├── conftest.py                      (NEW)
│           ├── test_mimic_gripper.py            (MOVED from software/sim-server/tests/world/)
│           ├── test_urdf_parity.py              (NEW)
│           └── test_mjx_compat.py               (NEW)
├── manifests/                                   ← NEW
│   └── norma/
│       ├── elrobot_follower.scene.yaml          (MOVED; mjcf_path updated)
│       └── menagerie_so_arm100.scene.yaml       (MOVED; mjcf_path updated)
├── assets/                                      (UNCHANGED)
├── elrobot_follower.urdf                        (UNCHANGED)
└── vendor/menagerie/                            (UNCHANGED)
```

**Total operations**: 5 moves + 13 content edits + 9 new files = 27 operations, all committed atomically.

---

## Execution Approach

This plan has **ONE task** that performs all 27 operations and commits atomically. The task has ~40 bite-sized steps. Do NOT commit partway through — the entire restructure is one atomic unit. If anything fails mid-way, `git restore .` + `git clean -fd hardware/elrobot/simulation/mujoco hardware/elrobot/simulation/manifests` to reset, then investigate.

The single-task structure matches the brainstorming-approved "Approach 2: atomic commit on main" that was chosen over worktree+subagent-per-file ceremony because the scope is tight and the risk is low (all operations are mechanical).

---

## Chunk 1: Atomic Restructure

### Task 1: Restructure `hardware/elrobot/simulation/` to three-tier first-class layout

**Files:**

- Create (directories):
  - `hardware/elrobot/simulation/mujoco/`
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/`
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/`
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/`
  - `hardware/elrobot/simulation/manifests/`
  - `hardware/elrobot/simulation/manifests/norma/`

- Create (files):
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/README.md`
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md`
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION`
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/LICENSE`
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml`
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/README.md`
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/conftest.py`
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py`
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mjx_compat.py`

- Move (from → to):
  - `hardware/elrobot/simulation/elrobot_follower.xml` → `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml`
  - `hardware/elrobot/simulation/elrobot_follower.scene.yaml` → `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`
  - `hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml` → `hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml`
  - `docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md` → `hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/menagerie_diff.md`
  - `software/sim-server/tests/world/test_mimic_gripper.py` → `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mimic_gripper.py`

- Modify (in-place content edits):
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml` (meshdir, after the move)
  - `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml` (mjcf_path, after the move)
  - `hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml` (mjcf_path, after the move)
  - `software/station/bin/station/station-sim.yaml` (`--manifest` path)
  - `software/station/bin/station/station-shadow.yaml` (`--manifest` path)
  - `software/station/bin/station/station-sim-menagerie.yaml` (`--manifest` path)
  - `Makefile` (`sim-standalone` target `--manifest` path)
  - `Makefile` (`sim-test` target pytest paths — append new test dir)
  - `software/sim-server/README.md` (Scenario B command example)
  - `software/sim-server/norma_sim/world/manifest.py` (docstring line 3)
  - `software/sim-server/scripts/probe_manifest.py` (docstring line 10)
  - `software/sim-server/tests/conftest.py` (`elrobot_mjcf_path` fixture path)
  - `software/sim-server/tests/conftest.py` (`elrobot_scene_yaml` fixture path)

---

#### Phase A: Pre-flight verification

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
- HEAD: `b80b6c6` or newer (the last spec commit)

If not clean, STOP. Investigate before proceeding.

- [ ] **Step A.2: Verify the pre-Chunk-0 baseline**

```bash
make sim-test 2>&1 | tail -3
```

Expected: `88 passed, 0 failed, 0 skipped in <N>s`.

If not 88 passed, STOP. MVP-2's merge is broken and must be fixed before Chunk 0.

- [ ] **Step A.3: Record the repo root LICENSE for later copy**

```bash
ls -la /home/yuan/proj/norma-core/LICENSE* /home/yuan/proj/norma-core/COPYING* 2>&1
head -5 /home/yuan/proj/norma-core/LICENSE 2>&1 || echo "no LICENSE at repo root"
```

Expected outcomes:
- If a `LICENSE` file exists at repo root: note its path; it will be copied in Step C.2.
- If no `LICENSE` at repo root: the robot package will get an Apache-2.0 LICENSE (the same license MuJoCo Menagerie uses, which is the most common choice in robotics sim). This falls back to `https://www.apache.org/licenses/LICENSE-2.0.txt` copied into the file.

Record the decision before proceeding (write it into your task notes or the `robot.yaml` license field).

---

#### Phase B: Create directory skeleton + move files

- [ ] **Step B.1: Create the new directory skeleton**

```bash
cd /home/yuan/proj/norma-core
mkdir -p hardware/elrobot/simulation/mujoco/elrobot_follower/measurements
mkdir -p hardware/elrobot/simulation/mujoco/elrobot_follower/tests
mkdir -p hardware/elrobot/simulation/manifests/norma
```

Expected: no output, exit 0. The `-p` flag creates intermediate directories and silently succeeds if they already exist.

Verify:

```bash
ls -d hardware/elrobot/simulation/mujoco/elrobot_follower/{measurements,tests}
ls -d hardware/elrobot/simulation/manifests/norma
```

Expected: all three paths print without errors.

- [ ] **Step B.2: Move the MJCF into the engine-tier package**

```bash
git mv hardware/elrobot/simulation/elrobot_follower.xml \
       hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml
```

Expected: no output, exit 0.

Verify:

```bash
git status --short | head -5
ls hardware/elrobot/simulation/elrobot_follower.xml 2>&1 || echo "OK: old path removed"
ls hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml
```

Expected: status shows an `R` (rename) entry; old path does not exist; new path exists.

- [ ] **Step B.3: Move the ElRobot scene yaml into Norma manifests**

```bash
git mv hardware/elrobot/simulation/elrobot_follower.scene.yaml \
       hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml
```

Verify:

```bash
ls hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml
```

Expected: file exists at the new location.

- [ ] **Step B.4: Move the Menagerie scene yaml into Norma manifests**

```bash
git mv hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml \
       hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml
```

Verify:

```bash
ls hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml
```

Expected: file exists at the new location.

- [ ] **Step B.5: Move the comparison table into measurements, renaming**

```bash
git mv docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md \
       hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/menagerie_diff.md
```

Verify:

```bash
ls docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md 2>&1 || echo "OK: old path gone"
ls hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/menagerie_diff.md
head -3 hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/menagerie_diff.md
```

Expected: old path gone, new path exists, file contains the comparison table header (`# MVP-2 Menagerie → ElRobot Parameter Comparison Table` or similar — the existing content is preserved).

- [ ] **Step B.6: Move `test_mimic_gripper.py` into the engine-tier tests**

```bash
git mv software/sim-server/tests/world/test_mimic_gripper.py \
       hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mimic_gripper.py
```

Verify:

```bash
ls software/sim-server/tests/world/test_mimic_gripper.py 2>&1 || echo "OK: old path gone"
ls hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mimic_gripper.py
grep -c 'import norma_sim\|from norma_sim' hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mimic_gripper.py
```

Expected: old path gone, new path exists, grep returns `0` (no `norma_sim` imports — the file is pure mujoco, which is why it can live in the engine-tier package).

---

#### Phase C: Edit moved files (content changes)

- [ ] **Step C.1: Update `meshdir` in the moved MJCF**

The MJCF at the new location has `meshdir="assets"` which would try to resolve mesh files at `mujoco/elrobot_follower/assets/` — that directory doesn't exist. Change to `meshdir="../../assets"` which resolves two levels up to `hardware/elrobot/simulation/assets/`.

Use the Edit tool (not sed) to apply exactly this change:

In `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml`:
- Find: `<compiler angle="radian" meshdir="assets" autolimits="true"/>`
- Replace with: `<compiler angle="radian" meshdir="../../assets" autolimits="true"/>`

Verify:

```bash
grep 'meshdir' hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml
```

Expected output: `  <compiler angle="radian" meshdir="../../assets" autolimits="true"/>`

- [ ] **Step C.2: Update `mjcf_path` in the moved ElRobot scene yaml**

In `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`:
- Find: `mjcf_path: ./elrobot_follower.xml`
- Replace with: `mjcf_path: ../../mujoco/elrobot_follower/elrobot_follower.xml`

Verify:

```bash
grep 'mjcf_path' hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml
```

Expected output: `mjcf_path: ../../mujoco/elrobot_follower/elrobot_follower.xml`

- [ ] **Step C.3: Update `mjcf_path` in the moved Menagerie scene yaml**

In `hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml`:
- Find: `mjcf_path: ./vendor/menagerie/trs_so_arm100/scene.xml`
- Replace with: `mjcf_path: ../../vendor/menagerie/trs_so_arm100/scene.xml`

Verify:

```bash
grep 'mjcf_path' hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml
```

Expected output: `mjcf_path: ../../vendor/menagerie/trs_so_arm100/scene.xml`

---

#### Phase D: Create new metadata files

- [ ] **Step D.1: Create `VERSION`**

Write to `hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION`:

```
0.1.0
```

(Single line; file ends with a trailing newline.)

Verify:

```bash
cat hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION
wc -c hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION
```

Expected: content is `0.1.0`, byte count is `6` (5 chars + newline).

- [ ] **Step D.2: Create `LICENSE`**

Copy from repo root LICENSE if it exists, otherwise use Apache-2.0 text.

If repo root has a LICENSE:

```bash
cp /home/yuan/proj/norma-core/LICENSE \
   hardware/elrobot/simulation/mujoco/elrobot_follower/LICENSE
```

If no repo root LICENSE, download or paste Apache-2.0 full text into the file. The canonical text is at <https://www.apache.org/licenses/LICENSE-2.0.txt>. Since this plan cannot assume network access, the implementer should:
- First check `git show HEAD:LICENSE 2>&1` for a repo-root LICENSE
- If that fails, check for `COPYING` at repo root
- If both fail, write Apache-2.0 text from memory or skip LICENSE creation temporarily and flag it as a concern in the self-review report

Verify:

```bash
ls -la hardware/elrobot/simulation/mujoco/elrobot_follower/LICENSE
head -3 hardware/elrobot/simulation/mujoco/elrobot_follower/LICENSE
```

Expected: file exists and contains recognizable license header text.

- [ ] **Step D.3: Create `robot.yaml`**

Write to `hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml`:

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

Verify:

```bash
python3 -c "
import yaml
with open('hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml') as f:
    data = yaml.safe_load(f)
assert data['robot']['id'] == 'elrobot_follower'
assert data['kinematics']['actuated_dof'] == 8
assert data['actuators']['count'] == 8
assert len(data['actuators']['mjcf_names']) == 8
assert len(data['actuators']['client_ids']) == 8
assert data['version']['current'] == '0.1.0'
print('robot.yaml OK')
"
```

Expected: `robot.yaml OK`.

- [ ] **Step D.4: Create `README.md`**

Write to `hardware/elrobot/simulation/mujoco/elrobot_follower/README.md`:

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

Verify:

```bash
wc -l hardware/elrobot/simulation/mujoco/elrobot_follower/README.md
head -3 hardware/elrobot/simulation/mujoco/elrobot_follower/README.md
```

Expected: ~70 lines, starts with `# ElRobot Follower Arm — MuJoCo Physics Model`.

- [ ] **Step D.5: Create `CHANGELOG.md`**

Write to `hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md`. Use the full content from the spec's Appendix A (Section 9 of `docs/superpowers/specs/2026-04-12-mvp3-first-class-mjcf-design.md`). The content is too long to reproduce inline in this plan — read it from the spec and paste it into the file.

Alternatively, the essential skeleton is:

```markdown
# ElRobot Physics Model CHANGELOG

Follows a subset of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versioning is semver, independent of the `software/` crates.

## [Unreleased]

(nothing yet)

## [0.1.0] — 2026-04-12

### Added

- Initial hand-written MJCF `elrobot_follower.xml` derived from Menagerie
  `trs_so_arm100` v1.3 @ commit `c771fb04055d805f20db0eab6cb20b67555887d0`
  (2025-06-09 tuning). Full change list is in the spec Appendix A.

### Physics gate results (at initial release)

- Floor §3.1 all 6 criteria: GREEN (0 tuning iterations; Menagerie
  baseline passed first-try).
- Ceiling §3.2 item 7 (web UI slider responsiveness including M1): PASS
  via manual browser smoke test 2026-04-12. MVP-1's M1-unresponsive
  regression is resolved.

### Integration context

- NormaCore MVP-2 merge commit: `93c1597` on `main` (2026-04-12)
- MVP-3 Chunk 0 commit: (this commit)
- Chunk 0 spec: `docs/superpowers/specs/2026-04-12-mvp3-first-class-mjcf-design.md`
```

**Important**: the implementer should use the FULL Appendix A content from the spec, not this skeleton. The skeleton is here only to show the required structure. Read `docs/superpowers/specs/2026-04-12-mvp3-first-class-mjcf-design.md` Section 9's "Appendix A: Full content of `mujoco/elrobot_follower/CHANGELOG.md`" block and copy it verbatim.

Verify:

```bash
wc -l hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md
grep -c '## \[0.1.0\]' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md
grep -c '`c771fb04055d805f20db0eab6cb20b67555887d0`' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md
```

Expected: ~80-100 lines, one `## [0.1.0]` header, one mention of the Menagerie commit SHA.

- [ ] **Step D.6: Create `measurements/README.md`**

Write to `hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/README.md`:

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

Verify:

```bash
wc -l hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/README.md
head -3 hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/README.md
```

Expected: ~35 lines, starts with `# Measurements`.

---

#### Phase E: Create new test files

- [ ] **Step E.1: Create `tests/conftest.py`**

Write to `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/conftest.py`:

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

Verify:

```bash
python3 -c "
import ast
tree = ast.parse(open('hardware/elrobot/simulation/mujoco/elrobot_follower/tests/conftest.py').read())
funcs = [n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]
assert funcs == ['elrobot_mjcf_path'], f'expected single fixture, got {funcs}'
print('conftest.py OK')
"
```

Expected: `conftest.py OK`.

- [ ] **Step E.2: Create `tests/test_urdf_parity.py`**

Write to `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py`:

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

Verify:

```bash
python3 -c "
import ast
tree = ast.parse(open('hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py').read())
funcs = [n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]
assert 'test_urdf_and_mjcf_agree_on_joint_names' in funcs
assert 'test_urdf_and_mjcf_agree_on_actuated_joint_count' in funcs
print('test_urdf_parity.py OK')
"
```

Expected: `test_urdf_parity.py OK`.

- [ ] **Step E.3: Create `tests/test_mjx_compat.py`**

Write to `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mjx_compat.py`:

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

Verify:

```bash
python3 -c "
import ast
tree = ast.parse(open('hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_mjx_compat.py').read())
funcs = [n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]
assert 'test_mjx_forward_pass_compiles' in funcs
print('test_mjx_compat.py OK')
"
```

Expected: `test_mjx_compat.py OK`.

---

#### Phase F: Update path references in existing files

- [ ] **Step F.1: Update `station-sim.yaml` `--manifest` path**

In `software/station/bin/station/station-sim.yaml`:
- Find: `    - hardware/elrobot/simulation/elrobot_follower.scene.yaml`
- Replace with: `    - hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`

Note: preserve the 4-space indentation.

Verify:

```bash
grep '\-\-manifest\|elrobot_follower.scene.yaml' software/station/bin/station/station-sim.yaml
```

Expected: shows two lines, `- --manifest` and `- hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`.

- [ ] **Step F.2: Update `station-shadow.yaml` `--manifest` path**

In `software/station/bin/station/station-shadow.yaml`:
- Find: `    - hardware/elrobot/simulation/elrobot_follower.scene.yaml`
- Replace with: `    - hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`

Verify:

```bash
grep 'elrobot_follower.scene.yaml' software/station/bin/station/station-shadow.yaml
```

Expected: `    - hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`

- [ ] **Step F.3: Update `station-sim-menagerie.yaml` `--manifest` path**

In `software/station/bin/station/station-sim-menagerie.yaml`:
- Find: `    - hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml`
- Replace with: `    - hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml`

Verify:

```bash
grep 'menagerie_so_arm100.scene.yaml' software/station/bin/station/station-sim-menagerie.yaml
```

Expected: `    - hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml`

- [ ] **Step F.4: Update `Makefile` `sim-standalone` target**

In `Makefile`:
- Find: `	  --manifest hardware/elrobot/simulation/elrobot_follower.scene.yaml \`
- Replace with: `	  --manifest hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml \`

Note: preserve the tab character at the start (Makefile syntax).

Verify:

```bash
grep -n 'manifest hardware/elrobot' Makefile
```

Expected: one line showing the new path (no tab-vs-space issues — use `cat -A` if uncertain).

- [ ] **Step F.5: Update `Makefile` `sim-test` target**

In `Makefile`, find the `sim-test` recipe line that runs pytest:

Find: `	PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/`

Replace with:
```
	PYTHONPATH=software/sim-server python3 -m pytest \
	    software/sim-server/tests/ \
	    hardware/elrobot/simulation/mujoco/elrobot_follower/tests/
```

Preserve tab indentation. Note the line continuations via backslash and the two tab-indented continuation lines.

Verify:

```bash
grep -A 3 'sim-test:' Makefile | tail -5
cat -A Makefile | grep -A 3 'sim-test:' | tail -5
```

Expected: the `pytest` invocation spans multiple lines, includes both the old `software/sim-server/tests/` path and the new `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/` path. Tab characters (shown as `^I` with `cat -A`) at the start of continuation lines.

- [ ] **Step F.6: Update `software/sim-server/README.md` Scenario B command example**

In `software/sim-server/README.md`:
- Find: `    --manifest hardware/elrobot/simulation/elrobot_follower.scene.yaml \`
- Replace with: `    --manifest hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml \`

Verify:

```bash
grep -n 'manifest hardware' software/sim-server/README.md
```

Expected: one match with the new path.

- [ ] **Step F.7: Update `software/sim-server/norma_sim/world/manifest.py` docstring**

In `software/sim-server/norma_sim/world/manifest.py`, line 3 (inside the module docstring):
- Find: `Parses \`hardware/elrobot/simulation/elrobot_follower.scene.yaml\``
- Replace with: `Parses \`hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml\``

Verify:

```bash
head -10 software/sim-server/norma_sim/world/manifest.py
```

Expected: docstring shows the new path.

- [ ] **Step F.8: Update `software/sim-server/scripts/probe_manifest.py` usage example**

In `software/sim-server/scripts/probe_manifest.py`, around line 10 (inside the usage docstring):
- Find: `    --manifest hardware/elrobot/simulation/elrobot_follower.scene.yaml`
- Replace with: `    --manifest hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`

Verify:

```bash
grep -n 'manifest hardware' software/sim-server/scripts/probe_manifest.py
```

Expected: one match with the new path.

- [ ] **Step F.9: Update `software/sim-server/tests/conftest.py` `elrobot_mjcf_path` fixture path**

In `software/sim-server/tests/conftest.py`, inside the `elrobot_mjcf_path` fixture:
- Find: `    p = repo_root / "hardware/elrobot/simulation/elrobot_follower.xml"`
- Replace with: `    p = repo_root / "hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml"`

Verify:

```bash
grep -A 1 'def elrobot_mjcf_path' software/sim-server/tests/conftest.py | tail -5
```

Expected: shows the new path with `mujoco/elrobot_follower/`.

- [ ] **Step F.10: Update `software/sim-server/tests/conftest.py` `elrobot_scene_yaml` fixture path**

In `software/sim-server/tests/conftest.py`, inside the `elrobot_scene_yaml` fixture:
- Find: `    p = repo_root / "hardware/elrobot/simulation/elrobot_follower.scene.yaml"`
- Replace with: `    p = repo_root / "hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml"`

Verify:

```bash
grep -A 1 'def elrobot_scene_yaml' software/sim-server/tests/conftest.py | tail -5
```

Expected: shows the new path with `manifests/norma/`.

---

#### Phase G: Verification

Run all 7 verification checks from the spec's Section 5. Do NOT commit until every check passes. If any check fails, investigate and fix before proceeding.

- [ ] **Step G.1: MJCF compiles from the new location (Section 5.1)**

```bash
cd /home/yuan/proj/norma-core
python3 -c "
import mujoco
m = mujoco.MjModel.from_xml_path(
    'hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml')
print(f'nu={m.nu} neq={m.neq} ntendon={m.ntendon}')
"
```

Expected output: `nu=8 neq=2 ntendon=2`

If this fails, likely causes:
- `meshdir="../../assets"` not applied — revisit Step C.1
- Accidental content change — re-read the moved MJCF and confirm only the `<compiler>` line changed

- [ ] **Step G.2: Norma scene yaml loads from new location (Section 5.2)**

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

Expected: `nu=8 neq=2 ntendon=2` then `ElRobot scene yaml loads OK from new manifests/ location`.

- [ ] **Step G.3: Menagerie scene yaml loads from new location (Section 5.3)**

```bash
PYTHONPATH=software/sim-server python3 -c "
from pathlib import Path
from norma_sim.world.model import MuJoCoWorld
world = MuJoCoWorld.from_manifest_path(Path(
    'hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml'))
print(f'nu={world.model.nu}')
"
```

Expected: `nu=6` (Menagerie has 6 actuators).

- [ ] **Step G.4: New engine-tier tests pass (Section 5.4)**

```bash
python3 -m pytest \
    hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -v 2>&1 | tail -30
```

Expected:
- `test_mimic_gripper.py::test_mimic_gripper_equality_works PASSED`
- `test_mimic_gripper.py::test_mimic_gripper_zero_setpoint_holds_zero PASSED`
- `test_urdf_parity.py::test_urdf_and_mjcf_agree_on_joint_names PASSED`
- `test_urdf_parity.py::test_urdf_and_mjcf_agree_on_actuated_joint_count PASSED`
- `test_mjx_compat.py::test_mjx_forward_pass_compiles` PASSED or SKIPPED

Total: 4 passed + 1 skipped (if MJX not installed) OR 5 passed (if MJX installed).

Note: no `PYTHONPATH` is needed — these are pure-mujoco tests.

- [ ] **Step G.5: Old sim-server tests still green (Section 5.5)**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/ -q \
    2>&1 | tail -5
```

Expected: `86 passed` (88 pre-Chunk-0 baseline minus 2 moved-out `test_mimic_gripper.py` tests).

If fewer than 86, check:
- Did Step F.9 or F.10 miss a fixture path update? Run the new fixture smoke tests: `pytest software/sim-server/tests/world/test_model.py::test_mujoco_world_loads_elrobot_mjcf -v`

- [ ] **Step G.6: Full `make sim-test` pipeline (Section 5.6)**

```bash
make sim-test 2>&1 | tail -15
```

Expected:
- Architecture invariants: `All architecture invariants hold ✓`
- Rust: three `test result: ok. N passed` lines (28 + 15 + 16 = 59 total)
- Python summary: `90 passed, 0 failed, 0 or 1 skipped`

The +2 over the pre-Chunk-0 baseline of 88 is explained by the 2 new URDF parity test functions.

- [ ] **Step G.7: Menagerie walking skeleton regression (Section 5.7)**

```bash
PYTHONPATH=software/sim-server python3 -m pytest \
    software/sim-server/tests/integration/test_menagerie_walking_skeleton.py -q
```

Expected: `6 passed`.

- [ ] **Step G.8: No dangling path references (Section 5.8)**

```bash
grep -rn 'hardware/elrobot/simulation/elrobot_follower\|hardware/elrobot/simulation/menagerie_so_arm100' \
    software/ hardware/ Makefile docs/ 2>&1 \
    | grep -v 'docs/superpowers/.*2026-04-1[012]' \
    | grep -v 'vendor/menagerie/VENDOR.md'
```

Expected: **no output** (no matches).

If any matches appear, go back and fix them. Possible leftover locations not in Phase F's list:
- `docs/superpowers/plans/*` — historical plan files, expected to match but filtered out by the grep
- `docs/superpowers/specs/*` — historical spec files, ditto

If the filtered output still shows operational files (in `software/` or `hardware/` outside `vendor/`), those are real leaks that must be fixed.

---

#### Phase H: Atomic commit

Only proceed here if ALL verification steps in Phase G passed.

- [ ] **Step H.1: Review the full change set**

```bash
git status --short
git diff --stat
```

Expected:
- 5 rename entries (from `git mv` operations)
- ~13 modified entries (edits to moved files + external path updates)
- ~9 new files (the new docs + metadata + tests)
- Total: ~27 operations as predicted in the spec

Sanity check: no unexpected files. If anything surprising shows up (e.g. accidental edits to `vendor/`, `target/`, etc.), investigate before committing.

- [ ] **Step H.2: Stage all changes**

```bash
git add hardware/elrobot/simulation/ \
        docs/superpowers/specs/ \
        software/sim-server/ \
        software/station/bin/station/ \
        Makefile
```

Do NOT use `git add -A` — explicit paths prevent accidental staging of untracked files like `MUJOCO_LOG.TXT` or `station_data/`.

Verify:

```bash
git status --short
```

Expected: every changed file is shown with a staged-only indicator (first character), no remaining unstaged changes for the files in the refactor.

- [ ] **Step H.3: Commit atomically**

Use a HEREDOC for the multi-paragraph commit message. The message is drafted in the spec's Section 7; copy it verbatim (it is the authoritative wording).

```bash
git commit -m "$(cat <<'EOF'
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

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```

Verify:

```bash
git log --oneline -3
git status --short
git show --stat HEAD | head -40
```

Expected:
- HEAD commit message starts with `mvp3: promote hardware/elrobot/simulation/ to three-tier first-class structure`
- `git status` shows clean tree
- `git show --stat HEAD` shows 21 distinct files changed (5 renames + 8 new + 8 edits to unchanged-location files + the 3 moved-files that also had content edits but are shown as renames)

- [ ] **Step H.4: Post-commit sanity re-run of `make sim-test`**

```bash
make sim-test 2>&1 | tail -10
```

Expected: same 90 passed / 0 failed / 0-1 skipped as in Step G.6. This re-verifies that the commit did not accidentally omit any staged change.

- [ ] **Step H.5: Self-review report**

Produce a short report answering:

1. How many files were in the final commit? (Expected: 21 distinct files.)
2. Did all 8 verification checks in Phase G pass? (Expected: yes.)
3. Any surprises during execution? (E.g., LICENSE file fallback, unexpected path match, missing directory.)
4. Any steps that were ambiguous or required judgment beyond what the plan specified? (These should be noted so the plan can be updated for future use.)
5. Final `git log --oneline -3` output showing the new commit at HEAD, with `b80b6c6` or newer directly beneath it.

The report goes into the task completion message back to the controller.

---

## Completion Criteria

Task 1 is complete when:

1. ✅ The single commit exists on `main` with the exact commit message from Step H.3.
2. ✅ `make sim-test` shows 90 passed, 0 failed, ≤1 skipped.
3. ✅ `make check-arch-invariants` passes.
4. ✅ `test_menagerie_walking_skeleton.py` still shows 6 passed.
5. ✅ Engine-tier tests (`pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/`) pass without `PYTHONPATH`.
6. ✅ The Section 5.8 grep for dangling path references produces no output.
7. ✅ `git status` is clean.

If all 7 criteria are met, the chunk is done. Proceed to the post-execution documentation update (memory refresh, etc.) as a separate follow-up — NOT part of this task.

---

## Risks and Rollback

**Primary risk**: a single step in Phases B/C/F introduces a typo that breaks Phase G verification. Rollback is trivial because no intermediate commits exist:

```bash
cd /home/yuan/proj/norma-core
git restore --staged .
git restore .
git clean -fd hardware/elrobot/simulation/mujoco hardware/elrobot/simulation/manifests
```

After a rollback, the repo should show the pre-Chunk-0 state (as of HEAD `b80b6c6` or newer). Verify with:

```bash
git status --short
ls hardware/elrobot/simulation/elrobot_follower.xml        # should exist (unmoved)
ls hardware/elrobot/simulation/mujoco 2>&1                  # should not exist
ls hardware/elrobot/simulation/manifests 2>&1               # should not exist
```

Then investigate the root cause of the failure and re-attempt from Step A.1.

**Secondary risk**: a partial commit lands if Step H.2 stages too much or too little. Mitigation: Step H.1's `git status --short` + `git diff --stat` review before staging. If the commit is wrong, `git reset --soft HEAD~1` undoes the commit but keeps changes staged for re-attempt.

**Do NOT**:
- Use `git add -A` (would accidentally stage `MUJOCO_LOG.TXT`, `station_data/`, etc.)
- Amend the commit after the fact (the commit is meant to be the single atomic unit)
- Commit partway through Phase B/C/F (atomicity is a chunk requirement)
- Skip verification checks in Phase G (they catch real issues)

---

## Execution Notes

- **Every `git mv` preserves history** — `git log --follow` on the new path will walk back through the old path's commits. No file history is lost.
- **Every content edit is small** — typically 1-3 lines per file. Use the Edit tool, not bulk sed, so the change surface is auditable.
- **PYTHONPATH matters**:
  - Section 5.1 / 5.4 / 5.7 (engine-tier tests) do NOT need PYTHONPATH (pure mujoco)
  - Section 5.2 / 5.3 / 5.5 / 5.6 (anything touching `norma_sim`) DO need `PYTHONPATH=software/sim-server`
  - `make sim-test` sets PYTHONPATH automatically, so running via `make` is the simplest path
- **The Menagerie walking skeleton is the canary** — if Section 5.7's 6 tests break, something in the Menagerie scene yaml's `mjcf_path` update is wrong, or the vendor path is unreachable. Investigate before committing.
