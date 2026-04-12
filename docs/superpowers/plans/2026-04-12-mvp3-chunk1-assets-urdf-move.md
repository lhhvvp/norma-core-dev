# MVP-3 Chunk 1: Assets + URDF Move into `elrobot_follower` Package — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move `hardware/elrobot/simulation/assets/` (19 STL meshes) and `hardware/elrobot/simulation/elrobot_follower.urdf` **into** the engine-tier robot package at `hardware/elrobot/simulation/mujoco/elrobot_follower/`. Simplify the MJCF's `meshdir="../../assets"` to `meshdir="assets"`. Update the 2 internal package documentation references that name the old asset path. Upgrade `tests/test_urdf_parity.py`'s URDF fixture from silent skip to hard fail (URDF is now mandatory content). Bump the robot package VERSION to 0.2.0. Fold in 3 minor Chunk 0 余债 items. All in **one atomic commit on `main`**.

**Architecture:** This is Chunk 1 of MVP-3 Engine Package Completion (3 chunks total). After this chunk, `mujoco/elrobot_follower/` is fully self-contained — `cp -r mujoco/elrobot_follower /tmp/test && cd /tmp/test && pytest tests/ -v` passes for the first time. Chunk 1 is the only true hard prerequisite for Chunk 3 (`test_elrobot_acceptance.py` rewrite needs the package to be self-contained for its capstone cp -r success criterion). Chunk 1 is purely organizational at the filesystem level: zero physics changes, zero Rust changes, zero new test logic. The single `make sim-test` total stays the same (delta = +0).

**Tech Stack:** Git (per-file `git mv` enumerated explicitly, NOT a bash loop — codex iter-1 recommended pattern matching Chunk 0); Python 3 + `mujoco` (MJCF compile verification); pytest (test verification); Bash (grep-first scan + verification scripts); Edit tool (in-place content edits, NEVER `sed`/`awk`).

---

## Reference Documents (READ FIRST)

Before starting, the implementer MUST read the following:

1. **The roadmap spec, Section 3 only** — `/home/yuan/proj/norma-core/docs/superpowers/specs/2026-04-12-mvp3-foundation-roadmap-design.md` (1200 lines total). You only need lines ~96–280 (Section 3: Chunk 1 — A2: Assets + URDF Move). The rest of the spec is for Chunks 2 and 3 and is out of scope for this implementation.
2. **The current package state** — list `/home/yuan/proj/norma-core/hardware/elrobot/simulation/mujoco/elrobot_follower/` (the engine-tier package created by Chunk 0). It contains: `elrobot_follower.xml`, `README.md`, `CHANGELOG.md`, `VERSION`, `LICENSE`, `robot.yaml`, `measurements/`, `tests/`. After this chunk it will also contain: `assets/` (19 STL files) and `elrobot_follower.urdf`.
3. **The Chunk 0 spec amendment chain** for context on how the three-tier structure was designed — `/home/yuan/proj/norma-core/docs/superpowers/specs/2026-04-12-mvp3-first-class-mjcf-design.md` (only Sections 3 and 4 if you need full context; otherwise skip).

---

## Pre-flight Grep Results (baked in by plan author 2026-04-12, re-verified at execution time)

The plan author ran the Section 7 (α) "grep first" mandatory scan when writing this plan. Results are baked into Phase A and Phase E below. The implementer **must** re-run the same grep at Step A.3 to detect any drift between plan-write time and execution time. If new matches appear, escalate (NEEDS_CONTEXT) — do NOT silently extend the file list.

**Scan command** (plan-write time):
```bash
grep -rn 'simulation/assets\|simulation/elrobot_follower\.urdf' software/ hardware/ Makefile docs/ 2>&1 \
    | grep -v 'docs/superpowers/.*2026-04-1[012]' \
    | grep -v 'vendor/menagerie/VENDOR.md'
```

**Results at plan-write time** (only 2 hits, both inside the engine-tier package):
1. `hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md:82` — `- Assets (\`*.stl\`) still live at \`hardware/elrobot/simulation/assets/\`,`
2. `hardware/elrobot/simulation/mujoco/elrobot_follower/README.md:35` — `The shared assets live one level up at \`hardware/elrobot/simulation/assets/\``

**Notably absent** (verified at plan-write time, must remain absent at execution time):
- Zero references in `software/sim-server/` (no fixtures, no integration tests, no scripts reference the URDF path or assets path)
- Zero references in `Makefile`
- The frontend at `software/station/clients/station-viewer/public/elrobot/` has its OWN independent copy of `elrobot_follower.urdf` + 19 STL files (verified by `diff` at plan-write time — the frontend URDF has different mass values, it's a separate asset). Chunk 1 must NOT touch the frontend copies.

---

## Starting State Verification

Before beginning Phase A, confirm:

```bash
cd /home/yuan/proj/norma-core
git branch --show-current   # must print: main
git log --oneline -1         # HEAD must be aa65fd3 (final spec commit) or newer
git status --short           # must be empty (clean tree) OR only have expected untracked
```

If any of these fail, STOP and investigate. Do NOT proceed.

---

## File Structure Overview

**What exists today** (at `hardware/elrobot/simulation/`):

```
hardware/elrobot/simulation/
├── assets/                                  ← 19 STL files; will MOVE
│   ├── Gripper_Base_v1_1.stl
│   ├── Gripper_Gear_v1_1.stl
│   ├── Gripper_Jaw_01_v1_1.stl
│   ├── Gripper_Jaw_02_v1_1.stl
│   ├── Joint_01_1.stl
│   ├── Joint_02_1.stl
│   ├── Joint_03_v1_1.stl
│   ├── Joint_04_v1_1.stl
│   ├── Joint_05_v1_1.stl
│   ├── Joint_06_v1_1.stl
│   ├── ST3215_1_v1_1.stl
│   ├── ST3215_2_v1_1.stl
│   ├── ST3215_3_v1_1.stl
│   ├── ST3215_4_v1_1.stl
│   ├── ST3215_5_v1_1.stl
│   ├── ST3215_6_v1_1.stl
│   ├── ST3215_7_v1_1.stl
│   ├── ST3215_8_v1_1.stl
│   └── base_link.stl
├── elrobot_follower.urdf                    ← 521 lines; will MOVE
├── manifests/norma/                         (UNCHANGED)
├── mujoco/elrobot_follower/                 (current location of the package; receives the moves)
│   ├── elrobot_follower.xml                 ← will EDIT (meshdir simplification)
│   ├── README.md                            ← will EDIT (Structure + line 35 + new "How to" maybe)
│   ├── CHANGELOG.md                         ← will EDIT (line 82 + new [0.2.0] entry + Item 3 backfill)
│   ├── VERSION                              ← will EDIT (0.1.0 → 0.2.0)
│   ├── LICENSE                              (UNCHANGED)
│   ├── robot.yaml                           ← will EDIT (version bump + prereq update)
│   ├── measurements/
│   │   ├── README.md                        (UNCHANGED)
│   │   └── menagerie_diff.md                (UNCHANGED)
│   └── tests/
│       ├── conftest.py                      (UNCHANGED)
│       ├── test_mimic_gripper.py            (UNCHANGED)
│       ├── test_urdf_parity.py              ← will EDIT (fixture path layer + skip→fail + unused param fix + Item 2 docstring fix)
│       └── test_mjx_compat.py               (UNCHANGED)
└── vendor/menagerie/                        (UNCHANGED)
```

**What the end state looks like** (after this chunk):

```
hardware/elrobot/simulation/
├── manifests/norma/                                       (UNCHANGED)
├── mujoco/elrobot_follower/
│   ├── assets/                                            ← NEW location for the 19 STLs
│   │   ├── (all 19 STL files, byte-identical to before)
│   ├── elrobot_follower.urdf                              ← NEW location
│   ├── elrobot_follower.xml                               (meshdir="assets" — simplified)
│   ├── README.md                                          (Structure section + line 35 updated)
│   ├── CHANGELOG.md                                       ([0.1.0] TBD → 6ef605b backfilled, [0.2.0] entry added)
│   ├── VERSION                                            (0.2.0)
│   ├── LICENSE                                            (UNCHANGED)
│   ├── robot.yaml                                         (version.current: "0.2.0")
│   ├── measurements/                                      (UNCHANGED)
│   └── tests/                                             (test_urdf_parity.py edited; others unchanged)
└── vendor/menagerie/                                      (UNCHANGED)
```

The `assets/` and `elrobot_follower.urdf` at `hardware/elrobot/simulation/` no longer exist after this chunk.

**Total operations**: 19 STL `git mv` + 1 URDF `git mv` + 1 MJCF edit + 1 test fixture edit (covering 3 things: path layer / skip→fail / unused param fix / Item 2 docstring fix) + 1 README edit + 1 CHANGELOG edit (covering 2 things: line 82 + new entry + Item 3 TBD backfill) + 1 VERSION edit + 1 robot.yaml edit = **26 file operations**, all committed atomically.

---

## Execution Approach

This plan has **ONE task** that performs all 26 operations and commits atomically. The task has ~35 bite-sized steps organized into 7 phases (A through G). Do **NOT** commit partway through — the entire restructure is one atomic unit.

If anything fails mid-way, run the Phase A-equivalent rollback:

```bash
cd /home/yuan/proj/norma-core
git restore --staged .
git restore .
git clean -fd hardware/elrobot/simulation/mujoco/elrobot_follower/assets hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.urdf
```

Then investigate the root cause before re-attempting Phase A.

The single-task atomic-commit structure matches Chunk 0's verified successful pattern.

---

## Chunk 1: Atomic Move

### Task 1: Move assets + URDF into `mujoco/elrobot_follower/` and atomically commit

**Files this task touches** (high-level — exact line numbers in step content):

- **Move (`git mv`)**:
  - 19 STL files: `hardware/elrobot/simulation/assets/<name>.stl` → `hardware/elrobot/simulation/mujoco/elrobot_follower/assets/<name>.stl`
  - `hardware/elrobot/simulation/elrobot_follower.urdf` → `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.urdf`

- **Edit (in-place)**:
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml` (line 21: `meshdir="../../assets"` → `meshdir="assets"`)
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py` (4 changes: fixture path layer, skip→fail upgrade, unused parameter fix via assertion, docstring drift fix)
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/README.md` (Structure section + line 35 reference)
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md` (line 82 + add `[0.2.0]` entry + Item 3 backfill of `[0.1.0]` TBD → `6ef605b`)
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION` (`0.1.0` → `0.2.0`)
  - `hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml` (`version.current` field)

---

#### Phase A: Pre-flight verification + grep-first re-validation

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
- HEAD: `aa65fd3` (final spec commit) or newer

If not clean, STOP. Investigate before proceeding.

- [ ] **Step A.2: Capture the current `make sim-test` baseline**

```bash
make sim-test 2>&1 | tail -3
```

Capture the two numbers from the output line `N passed, M skipped in Xs`. Store these locally as `BASELINE_PASSED` and `BASELINE_SKIPPED` (write them down — they will be used in Phase E success criteria).

Expected at plan-write time: `90 passed, 1 skipped` (mjx skipped because dev env doesn't have mujoco.mjx installed). If your environment differs, that's fine — the plan asserts deltas, not absolutes.

If `make sim-test` fails, STOP. The pre-Chunk-1 baseline is broken and must be fixed before this chunk.

- [ ] **Step A.3: Re-run grep-first scan (Section 7 (α) mandate)**

```bash
grep -rn 'simulation/assets\|simulation/elrobot_follower\.urdf' software/ hardware/ Makefile docs/ 2>&1 \
    | grep -v 'docs/superpowers/.*2026-04-1[012]' \
    | grep -v 'vendor/menagerie/VENDOR.md'
```

Expected output (at plan-write time, must match at execution time):

```
hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md:82:- Assets (`*.stl`) still live at `hardware/elrobot/simulation/assets/`,
hardware/elrobot/simulation/mujoco/elrobot_follower/README.md:35:The shared assets live one level up at `hardware/elrobot/simulation/assets/`
```

**Critical**: if grep returns ANY hit not in the expected list, STOP and escalate (NEEDS_CONTEXT). Do not silently add new hits to the Phase C edit list. New hits would mean either (a) someone added a new operational reference between plan-write time and now, OR (b) the plan author missed a hit. Either way, the plan needs to be amended before proceeding.

- [ ] **Step A.4: Verify the package's current state**

```bash
ls hardware/elrobot/simulation/mujoco/elrobot_follower/
```

Expected directory contents:
```
CHANGELOG.md
LICENSE
README.md
VERSION
elrobot_follower.xml
measurements
robot.yaml
tests
```

(No `assets/`, no `elrobot_follower.urdf` yet — those arrive in Phase B.)

- [ ] **Step A.5: Verify the 19 STL files exist at the source location**

```bash
ls hardware/elrobot/simulation/assets/ | wc -l
ls hardware/elrobot/simulation/assets/
```

Expected: count is **19**, and the files are:

```
Gripper_Base_v1_1.stl
Gripper_Gear_v1_1.stl
Gripper_Jaw_01_v1_1.stl
Gripper_Jaw_02_v1_1.stl
Joint_01_1.stl
Joint_02_1.stl
Joint_03_v1_1.stl
Joint_04_v1_1.stl
Joint_05_v1_1.stl
Joint_06_v1_1.stl
ST3215_1_v1_1.stl
ST3215_2_v1_1.stl
ST3215_3_v1_1.stl
ST3215_4_v1_1.stl
ST3215_5_v1_1.stl
ST3215_6_v1_1.stl
ST3215_7_v1_1.stl
ST3215_8_v1_1.stl
base_link.stl
```

If the count is not 19 or any name differs, STOP. Investigate. (This catches the case where someone renamed/added/deleted an STL since plan-write time.)

- [ ] **Step A.6: Verify the URDF exists at the source location**

```bash
ls hardware/elrobot/simulation/elrobot_follower.urdf
wc -l hardware/elrobot/simulation/elrobot_follower.urdf
```

Expected: file exists, ~521 lines (line count may vary slightly across formatting changes; we only assert file presence).

- [ ] **Step A.7: Verify the current MJCF `meshdir` setting**

```bash
grep -n 'meshdir' hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml
```

Expected output: `21:  <compiler angle="radian" meshdir="../../assets" autolimits="true"/>`

(The line number must be 21. If it's different, the MJCF has been edited since plan-write time — escalate.)

---

#### Phase B: Move operations (`git mv` per-file, NO bash loop)

Phase B uses **explicit per-file `git mv`** following the Chunk 0 pattern. No `for` loops. Every command is auditable in the plan and easy to verify individually.

- [ ] **Step B.1: Create the destination assets directory inside the package**

```bash
mkdir -p hardware/elrobot/simulation/mujoco/elrobot_follower/assets
```

Expected: no output, exit 0. The `-p` flag silently succeeds if the directory already exists (it doesn't, but defensive).

Verify:

```bash
ls -d hardware/elrobot/simulation/mujoco/elrobot_follower/assets
```

Expected: prints the path without error.

- [ ] **Step B.2: `git mv` all 19 STL files from `simulation/assets/` to `mujoco/elrobot_follower/assets/`**

Run all 19 commands as a single bash invocation. **All commands enumerated explicitly per-file** (NOT a `for` loop — codex iter-1 recommended pattern from Chunk 0):

```bash
git mv hardware/elrobot/simulation/assets/Gripper_Base_v1_1.stl     hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Gripper_Base_v1_1.stl
git mv hardware/elrobot/simulation/assets/Gripper_Gear_v1_1.stl     hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Gripper_Gear_v1_1.stl
git mv hardware/elrobot/simulation/assets/Gripper_Jaw_01_v1_1.stl   hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Gripper_Jaw_01_v1_1.stl
git mv hardware/elrobot/simulation/assets/Gripper_Jaw_02_v1_1.stl   hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Gripper_Jaw_02_v1_1.stl
git mv hardware/elrobot/simulation/assets/Joint_01_1.stl            hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Joint_01_1.stl
git mv hardware/elrobot/simulation/assets/Joint_02_1.stl            hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Joint_02_1.stl
git mv hardware/elrobot/simulation/assets/Joint_03_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Joint_03_v1_1.stl
git mv hardware/elrobot/simulation/assets/Joint_04_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Joint_04_v1_1.stl
git mv hardware/elrobot/simulation/assets/Joint_05_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Joint_05_v1_1.stl
git mv hardware/elrobot/simulation/assets/Joint_06_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Joint_06_v1_1.stl
git mv hardware/elrobot/simulation/assets/ST3215_1_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/ST3215_1_v1_1.stl
git mv hardware/elrobot/simulation/assets/ST3215_2_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/ST3215_2_v1_1.stl
git mv hardware/elrobot/simulation/assets/ST3215_3_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/ST3215_3_v1_1.stl
git mv hardware/elrobot/simulation/assets/ST3215_4_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/ST3215_4_v1_1.stl
git mv hardware/elrobot/simulation/assets/ST3215_5_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/ST3215_5_v1_1.stl
git mv hardware/elrobot/simulation/assets/ST3215_6_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/ST3215_6_v1_1.stl
git mv hardware/elrobot/simulation/assets/ST3215_7_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/ST3215_7_v1_1.stl
git mv hardware/elrobot/simulation/assets/ST3215_8_v1_1.stl         hardware/elrobot/simulation/mujoco/elrobot_follower/assets/ST3215_8_v1_1.stl
git mv hardware/elrobot/simulation/assets/base_link.stl             hardware/elrobot/simulation/mujoco/elrobot_follower/assets/base_link.stl
```

Expected: 19 commands run, no output, all exit 0.

Verify the moves succeeded and all 19 are detected as renames:

```bash
git status --short | grep '^R' | wc -l
git status --short | grep '\.stl' | head -25
```

Expected:
- First command: `19` (all 19 files showed up as `R` rename entries)
- Second command: 19 lines of `R  hardware/elrobot/simulation/assets/<name>.stl -> hardware/elrobot/simulation/mujoco/elrobot_follower/assets/<name>.stl`

If the rename count is < 19, STOP and investigate. Some `git mv` may have failed silently.

- [ ] **Step B.3: Verify the source `assets/` directory is now empty (and remove it)**

```bash
ls hardware/elrobot/simulation/assets/ 2>&1
```

Expected: either empty output, OR error "No such file or directory" (git mv may have removed the now-empty directory automatically).

If the directory still exists and is empty, git's tracking will show it as removed, but the empty dir on disk is harmless. Move on.

If the directory still has files, STOP — some `git mv` failed. Investigate.

- [ ] **Step B.4: `git mv` the URDF**

```bash
git mv hardware/elrobot/simulation/elrobot_follower.urdf hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.urdf
```

Expected: no output, exit 0.

Verify:

```bash
ls hardware/elrobot/simulation/elrobot_follower.urdf 2>&1 || echo "OK: old path removed"
ls hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.urdf
git status --short | grep urdf
```

Expected:
- First command: prints `OK: old path removed` (file no longer at old location)
- Second command: prints the new path (file present)
- Third command: shows `R  hardware/elrobot/simulation/elrobot_follower.urdf -> hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.urdf` (rename detected)

- [ ] **Step B.5: Sanity-check the overall git status**

```bash
git status --short
```

Expected output looks like (order may vary):
```
R  hardware/elrobot/simulation/assets/Gripper_Base_v1_1.stl -> hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Gripper_Base_v1_1.stl
R  hardware/elrobot/simulation/assets/Gripper_Gear_v1_1.stl -> hardware/elrobot/simulation/mujoco/elrobot_follower/assets/Gripper_Gear_v1_1.stl
... (17 more STL renames) ...
R  hardware/elrobot/simulation/elrobot_follower.urdf -> hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.urdf
?? MUJOCO_LOG.TXT
?? station_data/
```

Total: **20 R entries** (19 STL + 1 URDF) plus 2 untracked. No `M` (modified) entries yet — those come in Phase C.

If you see `M` entries, something edited a file outside the planned scope. Investigate.

If you see fewer than 20 `R` entries, some moves failed. Investigate.

---

#### Phase C: Edit moved + dependent files

Phase C performs the in-place content edits to (a) the moved MJCF, (b) the test fixture, (c) the README, (d) the CHANGELOG. **Use the Edit tool, NOT `sed` / `awk` / heredoc redirection.** Each edit is small (1-3 lines) and the Edit tool's exact-string-match prevents accidental scope creep.

- [ ] **Step C.1: Simplify `meshdir` in the moved MJCF**

The MJCF currently has `meshdir="../../assets"`. After Phase B, the assets are now at `mujoco/elrobot_follower/assets/`, which is the same directory as the MJCF, so the meshdir simplifies to `meshdir="assets"`.

In `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml`:
- **Find**: `<compiler angle="radian" meshdir="../../assets" autolimits="true"/>`
- **Replace with**: `<compiler angle="radian" meshdir="assets" autolimits="true"/>`

Use the Edit tool to apply this change. The exact-match form is critical — both strings include the surrounding `<compiler ...>` tag attributes to disambiguate.

Verify:

```bash
grep -n 'meshdir' hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml
```

Expected output: `21:  <compiler angle="radian" meshdir="assets" autolimits="true"/>`

(Same line number as before — line 21 — only the value changed.)

**Why this isn't a meshdir compiler edge case** (codex iter-1 reframe): MuJoCo resolves `meshdir` relative to the *containing MJCF file*. `meshdir="assets"` and `meshdir="../../assets"` use identical resolution logic — there is no compiler difference. The actual risk is in Chunk 2 (when `scene.xml` arrives), not in Chunk 1.

- [ ] **Step C.2: Edit `tests/test_urdf_parity.py` (4 changes in 1 file)**

This single file gets 4 edits, all from spec Section 3 + Chunk 0 余债 fold-ins:

**Edit 2a (path layer correction)**: the URDF fixture currently resolves `parent.parent.parent.parent / "elrobot_follower.urdf"` (4 levels up — `tests/` → `elrobot_follower/` → `mujoco/` → `simulation/`). After Phase B, the URDF lives in `mujoco/elrobot_follower/`, which is `tests/`'s parent's parent (2 levels up).

**Edit 2b (skip → fail upgrade)**: the fixture currently uses `pytest.skip("URDF not found")`. After this chunk, the URDF is **mandatory** content of the package — silent skip is wrong because the off-by-one path bug becomes invisible. Upgrade to a hard error.

**Edit 2c (Item 1 — fix unused parameter)**: `test_urdf_and_mjcf_agree_on_actuated_joint_count` declares `elrobot_mjcf_path` but doesn't use it. Codex iter-1 recommended upgrading the lint smell into a meaningful cross-check assertion (`assert model.nu == 8`).

**Edit 2d (Item 2 — fix docstring drift)**: same test function's docstring says "ElRobot has 8 actuated joints (7 revolute + 1 gripper primary)" but the URDF treats all 8 as `<joint type="revolute">`. Reword for internal consistency.

The safest way to apply 4 edits in 1 file is **4 separate Edit tool calls**, each with a unique find string. Apply them in this order:

**Edit 2a**: in `hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py`:
- **Find**:
```python
@pytest.fixture
def urdf_path() -> Path:
    here = Path(__file__).resolve()
    # tests/ → robot package → mujoco/ → simulation/
    p = here.parent.parent.parent.parent / "elrobot_follower.urdf"
    if not p.exists():
        pytest.skip(f"ElRobot URDF not found at {p}")
    return p
```
- **Replace with**:
```python
@pytest.fixture
def urdf_path() -> Path:
    here = Path(__file__).resolve()
    # tests/ → robot package (the URDF lives at the root of mujoco/elrobot_follower/)
    p = here.parent.parent / "elrobot_follower.urdf"
    # URDF is now mandatory content of the package (since MVP-3 Chunk 1).
    # If it's missing, that's a hard structural error, NOT a skip — silent
    # skip would hide an off-by-one path bug. Codex iter-1 explicitly
    # recommended this upgrade.
    assert p.exists(), (
        f"ElRobot URDF not found at {p}. URDF is mandatory content of "
        f"this package after MVP-3 Chunk 1; check that the assets+urdf "
        f"move was applied correctly."
    )
    return p
```

This single Edit handles both Edit 2a (path layer: `parent.parent.parent.parent` → `parent.parent`) and Edit 2b (skip → assert).

**Edit 2c**: in the same file, find the second test function and replace its body:
- **Find**:
```python
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
- **Replace with**:
```python
def test_urdf_and_mjcf_agree_on_actuated_joint_count(
    urdf_path: Path, elrobot_mjcf_path: Path
):
    """ElRobot has 8 revolute joints (7 arm DoF + 1 gripper primary).
    The MJCF may have additional mimic prismatic joints (rev_motor_08_1,
    rev_motor_08_2) that do not appear in the URDF as top-level
    revolute joints."""
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
    # Cross-check: the MJCF should also have nu==8 actuators (codex
    # iter-1 recommended turning the previously-unused elrobot_mjcf_path
    # parameter into a belt-and-suspenders check).
    model = mujoco.MjModel.from_xml_path(str(elrobot_mjcf_path))
    assert model.nu == 8, (
        f"Expected MJCF nu==8 actuators (matching URDF), got {model.nu}"
    )
```

This single Edit handles both Edit 2c (use the previously-unused `elrobot_mjcf_path` parameter via `assert model.nu == 8`) and Edit 2d (docstring drift fix: "8 revolute (7 arm DoF + 1 gripper primary)" instead of "8 actuated joints (7 revolute + 1 gripper primary)").

Verify:

```bash
python3 -c "
import ast
tree = ast.parse(open('hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py').read())
funcs = [n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]
assert 'test_urdf_and_mjcf_agree_on_joint_names' in funcs
assert 'test_urdf_and_mjcf_agree_on_actuated_joint_count' in funcs
assert 'urdf_path' in funcs
print('test_urdf_parity.py syntax OK')
"
grep -n 'parent\.parent' hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py
grep -n 'pytest\.skip' hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py || echo "OK: no pytest.skip remaining"
grep -n 'assert model\.nu == 8' hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py
grep -n '8 revolute joints' hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py
```

Expected:
- First command: `test_urdf_parity.py syntax OK`
- Second command: shows `parent.parent` (NOT `parent.parent.parent.parent`)
- Third command: `OK: no pytest.skip remaining` (the skip was removed in Edit 2b)
- Fourth command: matches the new `assert model.nu == 8` line
- Fifth command: matches the new docstring "8 revolute joints (7 arm DoF + 1 gripper primary)"

If any of these fails, you missed an edit. Re-apply the missing one before proceeding.

- [ ] **Step C.3: Edit `mujoco/elrobot_follower/README.md`**

Two changes needed:

**Edit 3a (Structure section)**: the README's "Structure" tree currently shows the package without `assets/` or `elrobot_follower.urdf`. Add them.

**Edit 3b (line 35 — assets reference)**: the README currently says "The shared assets live one level up at `hardware/elrobot/simulation/assets/`" — that's no longer true after Chunk 1. Update.

Both edits are in the same file. Use 2 separate Edit tool calls.

**Edit 3a (Structure section)**: in `hardware/elrobot/simulation/mujoco/elrobot_follower/README.md`:
- **Find**:
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
- **Replace with**:
```
elrobot_follower/
├── elrobot_follower.xml     ← main MJCF (8 joints + 2 mimic slides)
├── elrobot_follower.urdf    ← URDF kinematic source of truth (used by test_urdf_parity)
├── robot.yaml               ← machine-readable identity (source of truth)
├── VERSION                  ← semver (git-friendly)
├── LICENSE                  ← Apache-2.0
├── README.md                ← this file
├── CHANGELOG.md             ← physics-relevant changes
├── assets/                  ← STL meshes referenced by elrobot_follower.xml (meshdir="assets")
├── measurements/            ← parameter provenance + future sysID data
│   ├── README.md            ← folder purpose + workflow
│   └── menagerie_diff.md    ← Menagerie→ElRobot parameter adaptation record
└── tests/                   ← engine-level validation
    ├── conftest.py          ← single fixture (elrobot_mjcf_path)
    ├── test_mimic_gripper.py    ← P0 gripper mimic regression
    ├── test_urdf_parity.py      ← URDF↔MJCF consistency gate
    └── test_mjx_compat.py       ← MJX smoke test (placeholder)
```

**Edit 3b (line 35 — assets reference paragraph)**: same file, replace the next paragraph:
- **Find**:
```
The shared assets live one level up at `hardware/elrobot/simulation/assets/`
(not yet moved into this package). The MJCF's `meshdir="../../assets"`
resolves to them. A future upstream-contribution chunk will move assets
into this directory to make the package fully self-contained.
```
- **Replace with**:
```
The STL mesh assets live inside this package at `assets/` (moved in MVP-3
Chunk 1, commit `<TBD-mvp3-chunk1>`). The MJCF's `meshdir="assets"`
resolves to them. The package is now self-contained: it can be copied to
any location (e.g. `/tmp/elrobot-test`) and `pytest tests/` runs cleanly
without needing the rest of the NormaCore checkout on disk.
```

(`<TBD-mvp3-chunk1>` is intentionally a literal placeholder — Chunk 2 will retroactively backfill the actual SHA when it touches CHANGELOG. Same chicken-and-egg pattern as the Chunk 0 [0.1.0] TBD that we're fixing in this chunk.)

Verify:

```bash
grep -n '^├── assets/' hardware/elrobot/simulation/mujoco/elrobot_follower/README.md
grep -n 'meshdir="assets"' hardware/elrobot/simulation/mujoco/elrobot_follower/README.md
grep -n 'self-contained' hardware/elrobot/simulation/mujoco/elrobot_follower/README.md
grep -n 'one level up' hardware/elrobot/simulation/mujoco/elrobot_follower/README.md || echo "OK: no stale 'one level up' reference"
```

Expected:
- First: matches the new `├── assets/` tree entry
- Second: matches `meshdir="assets"` (1 hit, in the prose paragraph)
- Third: matches `self-contained` (≥1 hit)
- Fourth: `OK: no stale 'one level up' reference`

- [ ] **Step C.4: Edit `mujoco/elrobot_follower/CHANGELOG.md`**

Three changes needed:

**Edit 4a (line 82 — Known limitations)**: the CHANGELOG entry for `[0.1.0]` lists as a "Known limitation" that "Assets (`*.stl`) still live at `hardware/elrobot/simulation/assets/`...". This is no longer a known limitation after Chunk 1 (assets are now in the package). Edit the bullet.

**Edit 4b (Item 3 fold-in — TBD backfill)**: the same `[0.1.0]` entry has `MVP-3 Chunk 0 commit: TBD (pending this chunk's execution)` near line 96. The Chunk 0 commit is `6ef605b`. Backfill it.

**Edit 4c (new `[0.2.0]` entry)**: append a new top-level changelog entry above `## [0.1.0]` documenting Chunk 1's changes.

All three edits are in the same file. Use 3 separate Edit tool calls.

**Edit 4a (line 82)**: in `hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md`:
- **Find**:
```
- Assets (`*.stl`) still live at `hardware/elrobot/simulation/assets/`,
  not inside this package. MJCF uses `meshdir="../../assets"`. This
  prevents the package from being truly self-contained for upstream
  contribution; a future chunk will move assets into the package.
```
- **Replace with** (this turns the "Known limitation" bullet into a status note that Chunk 1 resolved it):
```
- Assets (`*.stl`) still live at `hardware/elrobot/simulation/assets/`,
  not inside this package at v0.1.0. MJCF uses `meshdir="../../assets"`.
  **Resolved in v0.2.0 (MVP-3 Chunk 1)** — see [0.2.0] entry above.
```

**Edit 4b (TBD backfill at ~line 96)**: same file:
- **Find**: `MVP-3 Chunk 0 commit: TBD (pending this chunk's execution)`
- **Replace with**: `MVP-3 Chunk 0 commit: 6ef605b on main (2026-04-12)`

**Edit 4c (new `[0.2.0]` entry)**: same file. Add a new section above `## [0.1.0] — 2026-04-12`:
- **Find**: `## [0.1.0] — 2026-04-12`
- **Replace with**:
```
## [0.2.0] — 2026-04-12

### Changed (structural)

- Moved `assets/` (19 STL meshes) and `elrobot_follower.urdf` **into** this
  package from `hardware/elrobot/simulation/`. The package is now fully
  self-contained — `cp -r mujoco/elrobot_follower /tmp/test && cd /tmp/test
  && pytest tests/` passes without needing the NormaCore checkout.
- Simplified `elrobot_follower.xml`'s `<compiler meshdir="../../assets">`
  to `meshdir="assets"` since assets now live in the same directory.
- Upgraded `tests/test_urdf_parity.py` URDF fixture from `pytest.skip` to
  hard `assert` — URDF is now mandatory package content; silent skip
  would hide off-by-one path bugs.

### Fixed (Chunk 0 余债 fold-ins, per MVP-3 Engine Package Completion roadmap Section 7)

- `tests/test_urdf_parity.py:50-65` — turned the previously-unused
  `elrobot_mjcf_path` parameter in
  `test_urdf_and_mjcf_agree_on_actuated_joint_count` into an `assert
  model.nu == 8` cross-check (lint smell → meaningful belt-and-suspenders
  assertion). Codex iter-1 recommendation.
- `tests/test_urdf_parity.py:53` — fixed docstring drift: "8 actuated
  joints (7 revolute + 1 gripper primary)" → "8 revolute joints (7 arm
  DoF + 1 gripper primary)". URDF parser treats all 8 as
  `<joint type="revolute">`.
- `CHANGELOG.md:96` — backfilled the `[0.1.0]` entry's "MVP-3 Chunk 0
  commit: TBD" placeholder with the actual SHA `6ef605b`.

### Physics gate results (at this version)

- Floor §3.1 acceptance gate: GREEN (no physics changes; same as v0.1.0).
- Engine-tier package tests: 4 passed + 1 skipped (mjx if absent).
- `cp -r /tmp` self-containment: 4 passed + 1 skipped (mjx if absent) —
  **first version where this is meaningful**.

### Integration context

- NormaCore main HEAD before this version: `aa65fd3` (or whatever the
  spec landing commit was)
- MVP-3 Engine Package Completion Chunk 1 commit: (this commit)
- Roadmap spec: `docs/superpowers/specs/2026-04-12-mvp3-foundation-roadmap-design.md`

## [0.1.0] — 2026-04-12
```

(The `## [0.1.0] — 2026-04-12` line at the end is the existing header — the Edit appends the new section above it.)

Verify:

```bash
grep -c '^## \[0\.2\.0\]' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md
grep -c '^## \[0\.1\.0\]' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md
grep -n 'TBD' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md || echo "OK: no TBD remaining"
grep -n '6ef605b' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md
grep -n 'Resolved in v0.2.0' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md
```

Expected:
- First: `1` (one [0.2.0] header)
- Second: `1` (one [0.1.0] header still present)
- Third: `OK: no TBD remaining`
- Fourth: matches `6ef605b` (1 hit, in the [0.1.0] integration-context paragraph)
- Fifth: matches the line 82 bullet that now references the v0.2.0 resolution

---

#### Phase D: Metadata bumps

Phase D bumps the package's version metadata. The three places that must stay in sync:
- `VERSION` file
- `robot.yaml` `version.current` field
- `CHANGELOG.md` (already has the new `[0.2.0]` section from Step C.4)

- [ ] **Step D.1: Bump `VERSION`**

In `hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION`:
- **Find**: `0.1.0`
- **Replace with**: `0.2.0`

(The file is one line + trailing newline. The Edit tool will preserve the trailing newline.)

Verify:

```bash
cat hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION
wc -c hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION
```

Expected: content is `0.2.0`, byte count is `6` (5 chars + newline). Same byte count as before because `0.1.0` and `0.2.0` are both 5 chars.

- [ ] **Step D.2: Bump `robot.yaml` `version.current`**

In `hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml`:
- **Find**: `  current: "0.1.0"`
- **Replace with**: `  current: "0.2.0"`

(The leading 2 spaces are because `current` is nested under `version:`. The Edit tool's exact match requires preserving indentation.)

Verify:

```bash
python3 -c "
import yaml
with open('hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml') as f:
    data = yaml.safe_load(f)
assert data['version']['current'] == '0.2.0', f'expected 0.2.0, got {data[\"version\"][\"current\"]}'
assert data['robot']['id'] == 'elrobot_follower'
assert data['kinematics']['actuated_dof'] == 8
assert data['actuators']['count'] == 8
print('robot.yaml version bumped to 0.2.0; all other fields intact')
"
```

Expected: `robot.yaml version bumped to 0.2.0; all other fields intact`

- [ ] **Step D.3: Sanity-check the three-way version sync**

```bash
echo "VERSION file:        $(cat hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION)"
echo "robot.yaml current:  $(grep 'current:' hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml | head -1 | tr -d ' ')"
echo "CHANGELOG top entry: $(grep -m1 '^## \[' hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md)"
```

Expected (whitespace may vary, but the version strings must match):
```
VERSION file:        0.2.0
robot.yaml current:  current:"0.2.0"
CHANGELOG top entry: ## [0.2.0] — 2026-04-12
```

If the three don't all show `0.2.0`, STOP. One of Steps C.4, D.1, or D.2 was misapplied.

---

#### Phase E: Verification gates (8 checks)

Phase E runs the 8 verification gates from Section 3 success criteria. Do **NOT** commit until every gate passes. If any gate fails, investigate and fix before proceeding.

- [ ] **Step E.1: MJCF compiles from the new location with simplified meshdir**

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
- `meshdir="assets"` not applied (Step C.1) — re-verify
- Some STL file failed to move (Step B.2) — re-verify per-file presence
- A path-resolution edge case — verify by running `python3 -c "import mujoco; mujoco.MjModel.from_xml_path('...'); import os; os.listdir('hardware/elrobot/simulation/mujoco/elrobot_follower/assets')"` to confirm assets/ exists at the expected path

- [ ] **Step E.2: Engine-tier tests pass without `PYTHONPATH`**

```bash
python3 -m pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -v 2>&1 | tail -20
```

Expected output ends with one of:
- `4 passed, 1 skipped in <N>s` (mjx skipped because not installed)
- `5 passed in <N>s` (mjx installed, all passed)

The 4 passes are: `test_mimic_gripper_*` × 2 + `test_urdf_and_mjcf_agree_*` × 2.

**Critical**: this run must NOT have `PYTHONPATH=software/sim-server` set. The whole point of the engine-tier package is that it runs without sim-server on the path. If you have a `.envrc` or shell init that sets PYTHONPATH, run with `PYTHONPATH= python3 -m pytest ...` to explicitly clear it.

- [ ] **Step E.3: `test_urdf_parity` PASSED count is exactly 2 (skip-vs-fail trap)**

```bash
python3 -m pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py -v 2>&1 | grep -c PASSED
```

Expected output: `2`

This is a **separate count check** (not just "no failures") because the prior version of the fixture used `pytest.skip("URDF not found")` — if the path layer is wrong, the test silently skips and the summary still shows `0 failed`. After Step C.2 the fixture uses `assert p.exists()` which raises an error on missing URDF, so this trap is now hard-defeated, but the explicit count remains as a defensive check.

If the count is < 2, investigate:
- Maybe the URDF didn't move (Step B.4 failed)
- Maybe the fixture path layer is still wrong (Edit 2a not applied)
- Maybe the assert message is confusing pytest into a different output format — try `pytest -v` to see the full trace

- [ ] **Step E.4: cp -r /tmp self-containment check**

This is the **first chunk where this check is meaningful**. Before Chunk 1, `cp -r` of the package would yield 0 passed / 2 failed (because `meshdir="../../assets"` cannot resolve in `/tmp`). After Chunk 1, the package owns its assets + URDF.

```bash
rm -rf /tmp/elrobot-self-test
cp -r hardware/elrobot/simulation/mujoco/elrobot_follower /tmp/elrobot-self-test
cd /tmp/elrobot-self-test
python3 -m pytest tests/ -v 2>&1 | tail -10
cd /home/yuan/proj/norma-core
```

Expected output ends with one of:
- `4 passed, 1 skipped in <N>s` (mjx skipped because not installed)
- `5 passed in <N>s` (mjx installed)

If the cp -r run produces failures, the package is NOT self-contained. Likely cause: a stale path reference in the MJCF, README, or test fixture. Investigate.

Cleanup after:

```bash
rm -rf /tmp/elrobot-self-test
```

- [ ] **Step E.5: `make sim-test` total delta = +0 (baseline-relative assertion)**

Run the full pipeline:

```bash
make sim-test 2>&1 | tail -3
```

Expected output: `<BASELINE_PASSED> passed, <BASELINE_SKIPPED> skipped in <N>s` — **the same numbers** captured in Step A.2.

**Why delta = +0**: Chunk 1 doesn't add or remove any tests. The 19 STL moves are file system operations only; the URDF move is also file system; the MJCF/README/CHANGELOG/VERSION/robot.yaml edits don't affect test count; the test_urdf_parity.py edit is in-place (no new test functions, no removed test functions).

If the count drifted from baseline, investigate which test changed. Likely causes:
- Step C.2's test_urdf_parity.py edit accidentally renamed a function (test count would drop or change)
- Some `import` in the edited fixture broke (collection error)
- A test that depends on the old fixture path is now broken

- [ ] **Step E.6: Phase G.8 grep — empty**

```bash
grep -rn 'simulation/assets\|hardware/elrobot/simulation/elrobot_follower\.urdf' \
    software/ hardware/ Makefile docs/ 2>&1 \
    | grep -v 'docs/superpowers/.*2026-04-1[012]' \
    | grep -v 'vendor/menagerie/VENDOR.md'
```

Expected: **no output** (empty).

**Crucial difference from Step A.3**: Step A.3 expected 2 hits (the two internal CHANGELOG/README references that we've now edited). After Step C.3 and C.4, those references should be gone — the new wording uses "Chunk 1 moved them in" / "self-contained" / etc. instead of the old "shared assets live one level up at `hardware/elrobot/simulation/assets/`".

If the grep returns ANY hit, that hit is a remaining stale reference. Investigate:
- If it's in a file we already touched in Phase C, that step's Edit didn't apply correctly — re-run it
- If it's in a file we did NOT touch in Phase C, the plan author missed a hit at write-time — escalate (NEEDS_CONTEXT)

- [ ] **Step E.7: `make check-arch-invariants` green**

```bash
make check-arch-invariants 2>&1 | tail -5
```

Expected: ends with `All architecture invariants hold ✓` (or equivalent green output).

If this fails, Chunk 1 may have accidentally introduced a cross-layer violation. Unlikely (we only touched MJCF + tests + docs + metadata), but the gate is the same one Chunk 0 used so it stays consistent.

- [ ] **Step E.8: `git status` shows only the expected changes**

```bash
git status --short
```

Expected:
- 19 `R` (rename) entries for STL files (Step B.2)
- 1 `R` entry for URDF (Step B.4)
- 5 `M` (modified) entries for `elrobot_follower.xml`, `tests/test_urdf_parity.py`, `README.md`, `CHANGELOG.md`, `VERSION`, `robot.yaml`
- Total: 20 R + 6 M = 26 entries
- Untracked: `MUJOCO_LOG.TXT`, `station_data/` (expected)

If you see fewer `R` or `M` entries than expected, some operation didn't apply. If you see MORE entries (e.g., a stray `M` in a file outside the planned scope), investigate.

---

#### Phase F: Atomic commit

Only proceed here if **all 8 verification gates** in Phase E passed.

- [ ] **Step F.1: Final review of the change set**

```bash
git status --short
git diff --stat --staged   # may be empty if nothing staged yet — that's fine
git diff --stat            # all changes (since they're not staged yet)
```

Expected: 26 file entries total. No surprises (e.g., no unrelated files modified).

- [ ] **Step F.2: Stage all changes with explicit paths**

```bash
git add hardware/elrobot/simulation/mujoco/elrobot_follower/ \
        hardware/elrobot/simulation/assets/ \
        hardware/elrobot/simulation/elrobot_follower.urdf
```

**Do NOT use `git add -A`** — explicit paths prevent accidental staging of `MUJOCO_LOG.TXT`, `station_data/`, or any other untracked files.

(The second and third paths might already not exist as files because the renames moved them — `git add` will still process the staged rename for them.)

Verify:

```bash
git status --short
```

Expected: every changed file is shown with a staged-only indicator (single character at column 1). No remaining unstaged changes for the files in the chunk.

- [ ] **Step F.3: Atomic commit**

Use a HEREDOC for the multi-paragraph commit message:

```bash
git commit -m "$(cat <<'EOF'
mvp3-c1: move assets+urdf into mujoco/elrobot_follower/ package

Chunk 1 of MVP-3 Engine Package Completion: moves all 19 STL meshes
under hardware/elrobot/simulation/assets/ and hardware/elrobot/simulation/
elrobot_follower.urdf INTO the engine-tier robot package at
hardware/elrobot/simulation/mujoco/elrobot_follower/. The package is now
fully self-contained:

  cp -r mujoco/elrobot_follower /tmp/test && cd /tmp/test && pytest tests/

passes for the first time (4 passed + 1 mjx skip). This is the only true
hard prerequisite for Chunk 3 (test_elrobot_acceptance.py rewrite) per
roadmap spec Section 6.

What moved (20 git rename operations):
  hardware/elrobot/simulation/assets/*.stl
    -> hardware/elrobot/simulation/mujoco/elrobot_follower/assets/*.stl
       (19 files: Gripper_Base/Gear/Jaw_01/Jaw_02_v1_1.stl,
        Joint_01_1/02_1/03..06_v1_1.stl,
        ST3215_1..8_v1_1.stl, base_link.stl)
  hardware/elrobot/simulation/elrobot_follower.urdf
    -> hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.urdf

Content edits:
- mujoco/elrobot_follower/elrobot_follower.xml: meshdir simplified
  from "../../assets" to "assets" (line 21). Identical compiler
  resolution semantics (codex iter-1 reframe — meshdir is relative to
  the containing MJCF file; no compiler edge case).
- mujoco/elrobot_follower/tests/test_urdf_parity.py: 4 in-file edits
  (1 path fixture layer correction, 1 skip->fail upgrade, 1 unused-
  parameter fix promoted to assert nu==8 cross-check, 1 docstring drift
  fix). Codex iter-1 recommended the skip->fail upgrade because URDF is
  now mandatory package content.
- mujoco/elrobot_follower/README.md: Structure section adds assets/ and
  elrobot_follower.urdf entries; "shared assets live one level up"
  paragraph rewritten to reflect the new self-contained state.
- mujoco/elrobot_follower/CHANGELOG.md: new [0.2.0] entry documenting
  this chunk; [0.1.0] line 82 "Known limitation" turned into a
  resolution note; [0.1.0] integration-context "MVP-3 Chunk 0 commit:
  TBD" backfilled with 6ef605b (3 Chunk 0 余债 fold-ins resolved here).
- mujoco/elrobot_follower/VERSION: 0.1.0 -> 0.2.0 (minor — structural
  package layout change).
- mujoco/elrobot_follower/robot.yaml: version.current bumped to 0.2.0
  in sync with VERSION file.

Codex iter-1 fold-ins also addressed in this chunk (per roadmap Section 7):
- Item 1: tests/test_urdf_parity.py:50-65 unused elrobot_mjcf_path
  parameter -> upgraded to "assert model.nu == 8" cross-check
- Item 2: tests/test_urdf_parity.py:53 docstring drift "8 actuated joints
  (7 revolute + 1 gripper primary)" -> "8 revolute joints (7 arm DoF +
  1 gripper primary)" for terminology consistency with the URDF parser
  predicate
- Item 3: CHANGELOG.md:96 "MVP-3 Chunk 0 commit: TBD" -> "6ef605b"

Verification (all post-commit, baseline-relative deltas per roadmap
Section 7 alpha-prime rule):
- make sim-test: BASELINE_PASSED + 0 / BASELINE_SKIPPED + 0 (no test
  added, no test removed; cross-repo total unchanged)
- make check-arch-invariants: All architecture invariants hold ✓
- pytest mujoco/elrobot_follower/tests/ (no PYTHONPATH): 4 passed +
  1 mjx skip (engine-tier package owns its own absolute count)
- cp -r /tmp self-containment: 4 passed + 1 mjx skip (FIRST chunk
  where this is meaningful)
- Phase G.8 grep for stale 'simulation/assets|simulation/elrobot_follower.urdf'
  (with same exclusions as Chunk 0): empty
- robot.yaml upstream.prerequisites: 1/5 done (Chunk 0) -> still 1/5
  done (Chunk 1 doesn't claim "tests in isolation" or "scene.xml
  wrapper" yet — those prereqs flip in Chunks 3 and 2 respectively)

Files unchanged in this chunk (explicit boundary - codex iter-1 zero
physics drift rule):
- elrobot_follower.xml <default>, <contact>, <actuator>, <tendon>
  blocks (only the <compiler meshdir> attribute changed)
- All Rust crates (sim-runtime, station, st3215-* bridges)
- norma_sim Python library
- vendor/menagerie/
- software/station/clients/station-viewer/public/elrobot/ (frontend
  has its OWN independent URDF + STL copies, intentionally untouched)
- hardware/elrobot/simulation/manifests/norma/*.scene.yaml
- All other tests in software/sim-server/tests/

Roadmap spec: docs/superpowers/specs/2026-04-12-mvp3-foundation-roadmap-design.md
Predecessor commit: 6ef605b (MVP-3 Chunk 0)
Plan: docs/superpowers/plans/2026-04-12-mvp3-chunk1-assets-urdf-move.md

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
- HEAD commit message starts with `mvp3-c1: move assets+urdf into mujoco/elrobot_follower/ package`
- `git status` shows clean tree (only the 2 expected untracked items)
- `git show --stat HEAD` shows ~26 distinct files: 19 STL renames + 1 URDF rename + 6 modified files (xml, test, README, CHANGELOG, VERSION, robot.yaml)

- [ ] **Step F.4: Post-commit `make sim-test` re-run sanity check**

```bash
make sim-test 2>&1 | tail -10
```

Expected: same `BASELINE_PASSED passed, BASELINE_SKIPPED skipped` as Step E.5. This re-verifies that the commit didn't accidentally omit any staged change.

---

#### Phase G: Self-review report

- [ ] **Step G.1: Produce a short report answering**

1. How many files were in the final commit? (Expected: **26 distinct files** = 19 STL renames + 1 URDF rename + 6 in-place modifications.)
2. Did all 8 verification checks in Phase E pass? (Expected: yes.)
3. What was the actual `BASELINE_PASSED` and `BASELINE_SKIPPED` measured in Step A.2, and did they match after Step F.4? (Expected: yes, exactly the same.)
4. Did the cp -r /tmp self-containment check produce 4 passed + 1 skipped (or 5 passed if mjx)?
5. Any surprises during execution? (E.g., a `git mv` failed, a verification gate had unexpected output, the grep at Step A.3 had a different result from plan-write time.)
6. Any steps that were ambiguous or required judgment beyond what the plan specified? (These should be noted so the plan can be updated.)
7. Final `git log --oneline -3` output showing the new commit at HEAD with `aa65fd3` (or whatever the spec landing commit was) directly beneath it.

The report goes into the task completion message back to the controller (or, if executing manually, into the session log).

---

## Completion Criteria

Task 1 is complete when:

1. ✅ The single commit exists on `main` with the exact commit message from Step F.3.
2. ✅ `make sim-test` shows `BASELINE_PASSED passed, BASELINE_SKIPPED skipped` (delta = +0 from Step A.2 baseline).
3. ✅ `make check-arch-invariants` passes.
4. ✅ Engine-tier tests pass without `PYTHONPATH` (4 passed + 1 mjx skip, OR 5 passed if mjx installed).
5. ✅ `cp -r mujoco/elrobot_follower /tmp/test && pytest tests/` passes (4 passed + 1 mjx skip, OR 5 passed if mjx installed).
6. ✅ Phase G.8 grep for `simulation/assets|simulation/elrobot_follower.urdf` (with exclusions) is empty.
7. ✅ `git status` is clean.
8. ✅ The three-way version sync (VERSION file, robot.yaml `version.current`, CHANGELOG `[0.2.0]`) all show `0.2.0`.

If all 8 criteria are met, Chunk 1 is done. Proceed to MVP-3 Chunk 2 (A3 — Scene Wrapper) brainstorming as a separate session.

---

## Risks and Rollback

**Primary risk**: a single step in Phases B/C/D introduces a typo that breaks Phase E verification. Rollback is trivial because no intermediate commits exist:

```bash
cd /home/yuan/proj/norma-core
git restore --staged .
git restore .
git clean -fd hardware/elrobot/simulation/mujoco/elrobot_follower/assets hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.urdf
```

After a rollback, the repo should show the pre-Chunk-1 state (HEAD `aa65fd3` or wherever you started). Verify with:

```bash
git status --short
ls hardware/elrobot/simulation/assets/                           # should exist (un-moved) with 19 STLs
ls hardware/elrobot/simulation/elrobot_follower.urdf             # should exist (un-moved)
ls hardware/elrobot/simulation/mujoco/elrobot_follower/assets 2>&1 || echo "OK: assets/ not yet in package"
ls hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.urdf 2>&1 || echo "OK: urdf not yet in package"
```

Then investigate the root cause of the failure and re-attempt from Step A.1.

**Post-commit rollback** (if a regression is discovered AFTER Phase F.3 lands the commit): use `git revert <commit_sha>`, **NOT** `git reset --hard`. Per roadmap spec Section 6 hardening (codex iter-1): revert creates a forward-moving "undo commit" that preserves history; reset rewrites history. Even though `main` is never pushed (per `git_topology` memory), revert is the better hygiene habit.

```bash
git revert <chunk_1_commit_sha>   # creates a new commit that undoes Chunk 1
```

**Secondary risk**: a partial commit lands if Step F.2 stages too much or too little. Mitigation: Step F.1's `git status --short` review before staging. If the commit is wrong, `git reset --soft HEAD~1` undoes the commit but keeps changes staged for re-attempt.

**Do NOT**:
- Use `git add -A` (would accidentally stage `MUJOCO_LOG.TXT`, `station_data/`, etc.)
- Amend the commit after the fact (the commit is meant to be the single atomic unit per roadmap spec)
- Commit partway through Phases B/C/D (atomicity is a chunk requirement)
- Skip verification checks in Phase E (they catch real issues)
- Touch `software/station/clients/station-viewer/public/elrobot/` (frontend's INDEPENDENT URDF + STL copies — chunk 1 must not affect them)
- Touch any Rust file, any norma_sim file, any vendor/menagerie file, or any MJCF body element other than the `<compiler meshdir>` attribute (zero physics drift, zero Rust changes per roadmap Section 8)

---

## Execution Notes

- **Every `git mv` preserves history** — `git log --follow` on the new path will walk back through the old path's commits. No file history is lost.
- **Every content edit is small** — typically 1-3 lines per file, with the exception of the CHANGELOG `[0.2.0]` entry (~30 lines) and the test_urdf_parity rewrite (~10 lines net change). Use the Edit tool, not bulk sed, so each change is auditable.
- **PYTHONPATH matters**:
  - Step E.2 / E.4 (engine-tier tests, cp -r tests) do NOT need PYTHONPATH (pure mujoco)
  - Step E.5 (`make sim-test`) sets PYTHONPATH automatically via the Makefile
  - If your shell init sets PYTHONPATH, run Step E.2 with `PYTHONPATH= python3 -m pytest ...` to clear it
- **The `cp -r /tmp` self-containment check (Step E.4) is the headline value** of this chunk — it's the moment the package becomes truly portable. If it works, MVP-3 Engine Package Completion is on track. If it fails, the package isn't actually self-contained and the bug needs to be found before committing.
- **The frontend's `public/elrobot/elrobot_follower.urdf` is a SEPARATE copy** with different physics values (different masses). Chunk 1 must NOT touch the frontend. If the spec or plan ever conflates them, that's a bug.
- **Three-way version sync** (VERSION file, robot.yaml, CHANGELOG): if these drift, the next chunk will start with the wrong baseline. Step D.3's sanity check prevents this.

*End of plan.*
