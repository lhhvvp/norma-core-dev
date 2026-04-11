# MVP-2 Menagerie Walking Skeleton — Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix MVP-1's M1-slider-unresponsive physics debt by forking MuJoCo Menagerie's `trs_so_arm100` MJCF as the new sim physics baseline, adapting it to ElRobot's 8-joint topology, and validating that MVP-1's Rust/IPC/bridge infrastructure is robot-agnostic via a walking-skeleton Phase 1 test.

**Architecture:** URDF-first → MJCF-first migration. MJCF becomes sim's single source of truth; `gen.py` URDF→MJCF pipeline is deleted. URDF is retained as ROS/MoveIt reference artifact. Two-phase walking skeleton: Phase 1 runs Menagerie's SO-ARM100 verbatim through MVP-1's stack (hypothesis A: infra is robot-agnostic); Phase 2 hand-writes ElRobot's 8-joint MJCF using Menagerie's hand-tuned parameters as the baseline (hypothesis B: Menagerie params transfer to ElRobot topology).

**Tech Stack:** MuJoCo 3.x (Python bindings), Python 3.12, pytest, Rust 2024 edition (unchanged — zero modifications to existing MVP-1 Rust crates), prost, normfs, Station subsystem framework.

**Spec:** `docs/superpowers/specs/2026-04-11-mvp2-menagerie-walking-skeleton-design.md`

**Related skills to reference during implementation:**
- `superpowers:systematic-debugging` — if Phase 1 exposes hidden ElRobot assumptions in norma_sim
- `superpowers:test-driven-development` — every functional change follows TDD
- `superpowers:verification-before-completion` — every chunk ends with running acceptance gates

---

## Chunks Overview

| # | Chunk | Gate |
|---|---|---|
| 1 | Phase 0 — Reconnaissance + Vendor | Menagerie trs_so_arm100 exists, vendored, loads in mujoco.viewer |
| 2 | norma_sim source migration (code only) | `urdf_joint` field renamed to `mjcf_joint`, `source_hash` removed, new `load_manifest` parses `.scene.yaml` schema and enumerates actuators from MJCF, new unit tests green against Menagerie MJCF |
| 3 | norma_sim test-fixture migration | `conftest.py` + 6 MVP-1 test files migrated to `menagerie_scene_yaml` / `elrobot_scene_yaml` fixtures; all non-ElRobot tests green, ElRobot-specific tests skip cleanly until Chunk 5 |
| 4 | Phase 1 — Walking skeleton configs + test | Menagerie SO-ARM100 runs end-to-end through station + norma_sim + bridge + web UI with sliders responding; permanent regression test `test_menagerie_walking_skeleton.py` green |
| 5 | Phase 2 — ElRobot MJCF construction | `elrobot_follower.xml` loads, passes `mj_forward` with `ncon=0`, all 8 actuators enumerated |
| 6 | Phase 2 — Acceptance tests | All 6 Floor criteria pass (including per-motor step response parametrize) |
| 7 | Phase 2 — Manual smoke + docs + wrap-up | All 10 DoD items in spec §3.4 ticked |

---

## Chunk 1: Phase 0 — Reconnaissance + Vendor

**Purpose:** Verify the Menagerie-fork strategy is viable by actually looking at `trs_so_arm100` content, and vendor the files into the repo with proper attribution.

**Gate:** Phase 0 reconnaissance steps all pass (spec §6.1). If any step fails, STOP and escalate — the spec assumes Menagerie has a usable SO-ARM100 model; if it doesn't, the whole spec needs revision.

**Files:**
- Create: `hardware/elrobot/simulation/vendor/menagerie/VENDOR.md`
- Create: `hardware/elrobot/simulation/vendor/menagerie/LICENSE`
- Create: `hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml` (copied from Menagerie, unmodified)
- Create: `hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/trs_so_arm100.xml` (copied from Menagerie, unmodified)
- Create: `hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/assets/*` (copied from Menagerie, unmodified — exact file list depends on what trs_so_arm100 references)

---

### Task 1.1: Clone mujoco_menagerie to /tmp and verify structure

**Files:** None (read-only exploration, external working dir)

**Rationale:** Before touching the repo, verify that `mujoco_menagerie` has a `trs_so_arm100/` directory that contains a loadable MJCF with usable hand-tuned parameters. This is the Phase 0 reconnaissance gate from spec §6.1 — if it fails, STOP.

- [ ] **Step 1: Clone mujoco_menagerie to /tmp**

```bash
cd /tmp && rm -rf menagerie && git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git menagerie
```

Expected: clone succeeds, `/tmp/menagerie/` directory exists.

- [ ] **Step 2: Verify `trs_so_arm100` directory exists**

```bash
ls /tmp/menagerie/trs_so_arm100/
```

Expected output should contain at minimum: `scene.xml`, `trs_so_arm100.xml` (or equivalent top-level MJCF), and an `assets/` directory. Note exact file list for step 1.2.

**If the directory does not exist** or has different naming: STOP. Report to user and re-check spec §10 Risk "Menagerie 没有 SO-ARM100". Do NOT proceed to step 3.

- [ ] **Step 3: Read scene.xml and note structure**

Use the Read tool on `/tmp/menagerie/trs_so_arm100/scene.xml`. Note:
- Does it use `<include>` to reference `trs_so_arm100.xml`?
- Does it add worldbody elements (floor, lights)?
- Record the file contents into a scratch note.

- [ ] **Step 4: Read trs_so_arm100.xml and characterize joints**

Use the Read tool on `/tmp/menagerie/trs_so_arm100/trs_so_arm100.xml`. Characterize:
- Number of `<joint>` elements (**expected: 5 revolute + 1 gripper = 6 actuators total**; verified against Menagerie CHANGELOG 2025-06-09 tuning as 5-DOF SO-100; see also upstream plan research notes)
- Presence of `<default>` block with `<joint armature="..." damping="..."/>` (REQUIRED for spec viability)
- Actuator type (expected: `<position>` for revolute + something for gripper)
- Gripper implementation: `<tendon>` + `<equality>`? `<weld>`? Or simple prismatic?
- Mesh path convention: relative to `assets/`?
- `<option>` block with timestep/gravity/integrator

Document findings in a scratch note. This info is needed for Chunk 4 (ElRobot MJCF construction). **Also record the tuning timestamp from the CHANGELOG** — the 2025-06-09 update matched joint limits to real hardware and is the current canonical version.

**If no `armature` attribute is present anywhere in the default block**: STOP. The whole MVP-2 fork strategy relies on Menagerie having tuned armature values. Report to user and re-examine spec.

**Topology gap warning (read before Chunk 5)**: Menagerie ships SO-100 only as 5-DOF. There is **no SO-101 variant in Menagerie** as of 2026-04. ElRobot is 7+1 DOF = 8 actuators, so the gap is **3 extra joints** (likely M2 Shoulder Roll + M7 Wrist Yaw + one more shoulder or wrist DOF — confirm by comparing URDF axes in Chunk 5 Task 5.1). If you want a 6-DOF fallback, the upstream [TheRobotStudio/SO-ARM100/Simulation/SO101](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101) has SO-101 MJCF, but it is **auto-generated via onshape-to-robot and not hand-tuned** — armature values will be missing/zero. Prefer Menagerie's hand-tuned 5-DOF as parameter source + nearest-neighbor estimate the 3 extra joints.

- [ ] **Step 5: Verify LICENSE is Apache 2.0**

```bash
head -5 /tmp/menagerie/LICENSE
```

Expected: "Apache License, Version 2.0" or equivalent. Also check if `trs_so_arm100/LICENSE` exists separately (some model directories have their own license file).

**If the license is not Apache 2.0 or MIT or similar permissive**: STOP. Spec Risk §10 "Menagerie license 比预期严" kicks in — vendor approach may not be legal. Report to user.

- [ ] **Step 6: Record the commit SHA of the cloned copy**

```bash
cd /tmp/menagerie && git rev-parse HEAD
```

Record the SHA. This will go into VENDOR.md in task 1.2.

- [ ] **Step 7: Clone and read `lachlanhurst/so100-mujoco-sim` as architecture reference**

**Rationale:** `lachlanhurst/so100-mujoco-sim` is a MIT-licensed project that layers `MuJoCo + Menagerie SO-ARM100 MJCF + LeRobot control + Qt UI` — **architecturally almost identical to MVP-2** (just swap the Qt UI + LeRobot control for station's web UI + `st3215-compat-bridge`). Reading how they bridged Menagerie MJCF into a live control stack may save significant debugging time in Chunk 4 (walking skeleton) and Chunk 5 (ElRobot MJCF construction).

This is **research-only**: no code is copied, nothing is vendored. We read it, take notes, and move on. License is MIT so even if we did copy patterns, it would be attribution-compatible.

```bash
cd /tmp && rm -rf so100-mujoco-sim && git clone --depth 1 https://github.com/lachlanhurst/so100-mujoco-sim.git
```

Expected: clone succeeds. Then read:
- The main sim loop file (likely `sim_runner.py`, `main.py`, or `app.py` — find it via `ls /tmp/so100-mujoco-sim/`)
- How they load the Menagerie MJCF (path resolution, scene file vs main file, any override strategy)
- How they wire slider inputs to MuJoCo `data.ctrl` (does it use the MVP-1 `ActuationApplier` pattern or something else?)
- How they handle the gripper specifically (tendon equality, or a different approach?)

**Document findings in a scratch note** — specifically any technique that differs from MVP-1's approach. Topics to look for:
1. Does the repo compute `actuator_gaintype`/`biastype` for classification, or does it use a simpler approach?
2. Does it handle collision primitive substitution (what MVP-1 struggled with)?
3. Does it use `<include>` for scene.xml composition, or a flat MJCF?

These findings inform Chunk 4's walking skeleton implementation decisions and Chunk 5 Task 5.2's hand-written MJCF structure.

**No commit, no file copy — pure research.** After reading, leave the clone at `/tmp/so100-mujoco-sim` for potential reference during Chunks 4-5.

**Gate for Task 1.1:** All 7 steps pass. If any fail, STOP and report. Do not proceed to task 1.2.

---

### Task 1.2: Vendor Menagerie files into `hardware/elrobot/simulation/vendor/menagerie/`

**Files:**
- Create: `hardware/elrobot/simulation/vendor/menagerie/LICENSE`
- Create: `hardware/elrobot/simulation/vendor/menagerie/VENDOR.md`
- Create: `hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/` (directory + files copied from `/tmp/menagerie/trs_so_arm100/`)

**Rationale:** Copy the exact Menagerie files into our repo as "vendored" artifacts with proper license attribution. These files are read by `python -m mujoco.viewer` for Phase 1 visual baseline and by Chunk 4 as the parameter source for ElRobot adaptation.

- [ ] **Step 1: Create vendor directory structure**

```bash
mkdir -p hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100
```

Expected: directory exists, is empty.

- [ ] **Step 2: Copy Menagerie LICENSE**

```bash
cp /tmp/menagerie/LICENSE hardware/elrobot/simulation/vendor/menagerie/LICENSE
```

Expected: file exists, contents match `/tmp/menagerie/LICENSE`.

- [ ] **Step 3: Copy trs_so_arm100 contents**

```bash
cp -r /tmp/menagerie/trs_so_arm100/* hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/
```

Expected: all MJCF files and `assets/` subdirectory copied. Verify with:

```bash
ls -la hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/
```

- [ ] **Step 4: Write VENDOR.md**

Create `hardware/elrobot/simulation/vendor/menagerie/VENDOR.md` with the following content (fill in {SHA} with the value from Task 1.1 Step 6):

```markdown
# Menagerie Vendor Import

This directory contains files vendored verbatim from
[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie).

## Source

- **Repository:** https://github.com/google-deepmind/mujoco_menagerie
- **Commit SHA:** `{SHA}`
- **Import date:** 2026-04-11
- **License:** Apache License 2.0 (see `LICENSE` in this directory)

## Vendored Content

- `trs_so_arm100/` — The Robot Studio SO-ARM100 MJCF model, copied unmodified.
  Used as physics parameter reference for the ElRobot MVP-2 sim (see
  `docs/superpowers/specs/2026-04-11-mvp2-menagerie-walking-skeleton-design.md`).

## Modifications

**None.** All files under `trs_so_arm100/` are byte-identical to the Menagerie
source at the commit above. If you need to modify MJCF content for ElRobot's
8-joint adaptation, create files at `hardware/elrobot/simulation/elrobot_follower.xml`
instead (see Chunk 4 of the MVP-2 plan).

## Update Procedure

To refresh vendored content from a newer Menagerie commit:

1. `git clone https://github.com/google-deepmind/mujoco_menagerie /tmp/menagerie`
2. Verify `trs_so_arm100/` still exists and license is still Apache 2.0
3. `rm -rf hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100`
4. `cp -r /tmp/menagerie/trs_so_arm100 hardware/elrobot/simulation/vendor/menagerie/`
5. `cp /tmp/menagerie/LICENSE hardware/elrobot/simulation/vendor/menagerie/LICENSE`
6. Update this VENDOR.md with the new commit SHA and date
7. Run the Phase 1 walking skeleton tests (`pytest software/sim-server/tests/integration/test_menagerie_walking_skeleton.py`) to verify nothing broke
```

- [ ] **Step 5: Verify directory layout**

```bash
find hardware/elrobot/simulation/vendor/menagerie -type f | sort
```

Expected: at minimum `VENDOR.md`, `LICENSE`, `trs_so_arm100/scene.xml`, `trs_so_arm100/trs_so_arm100.xml`, plus whatever asset files Menagerie uses.

- [ ] **Step 6: Commit the vendor import**

```bash
git add hardware/elrobot/simulation/vendor/menagerie
git commit -m "$(cat <<'EOF'
vendor: import mujoco_menagerie trs_so_arm100 (MVP-2 Phase 0)

Vendored from https://github.com/google-deepmind/mujoco_menagerie at
commit {SHA} on 2026-04-11. Used as physics parameter reference for
ElRobot's MVP-2 sim (see spec 2026-04-11-mvp2-menagerie-walking-skeleton).

Contents are byte-identical to upstream; see VENDOR.md for update
procedure. License is Apache 2.0.
EOF
)"
```

(Fill in `{SHA}` with the commit SHA recorded in Task 1.1 Step 6.)

Expected: commit succeeds, `git status` clean.

---

### Task 1.3: MuJoCo native viewer smoke test (visual baseline)

**Files:** None (pure verification, no file changes)

**Rationale:** Before attempting to load Menagerie's MJCF through our `norma_sim` stack, verify it loads correctly in MuJoCo's own viewer. This establishes a visual baseline ("this is what a good sim looks like") and catches any Menagerie-side issues (missing assets, MJCF syntax errors) before they mix with our infra.

- [ ] **Step 1: Launch MuJoCo viewer on vendored Menagerie MJCF**

```bash
python -m mujoco.viewer hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml
```

Expected: a window opens showing the SO-ARM100 arm rendered against Menagerie's default scene (floor + lighting).

**If it fails to open** with a mesh-not-found error: check that `trs_so_arm100/assets/` was copied in Task 1.2 step 3. If assets are missing, re-copy from `/tmp/menagerie/trs_so_arm100/assets/`.

**If it fails with "mujoco is not installed" or similar**: ensure `python3 -c "import mujoco; print(mujoco.__version__)"` works. If not, install via `pip install mujoco` (non-goal exception from spec §2.2 allows `mujoco` upgrades if required).

- [ ] **Step 2: Visually verify the arm loads in home pose and doesn't fall**

In the viewer window:
- Observe that the arm is rendered in a sensible pose (not collapsed through the floor, not flying apart)
- Observe that the simulation is running (timer ticks in the status bar)
- After several seconds, the arm should be stationary (gravity is balanced by holding torques)

**If the arm falls apart or oscillates**: Menagerie's `trs_so_arm100` may have a broken scene. STOP and examine `scene.xml` — possibly the `<option>` block has been tweaked by recent Menagerie commits. Fallback: try loading `trs_so_arm100.xml` directly (without the scene wrapper):

```bash
python -m mujoco.viewer hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/trs_so_arm100.xml
```

- [ ] **Step 3: Manually drag a joint in the viewer (visual interactivity check)**

In the MuJoCo viewer, hold `Ctrl + Left-click-drag` on a revolute joint (e.g., the shoulder). The arm segment should rotate smoothly in response.

Expected:
- Smooth response with visible inertia
- No flickering or snapping
- No NaN artifacts (arm disappearing)
- When released, the arm holds position (or settles under gravity) without oscillating wildly

This establishes the **visual quality bar** for the ceiling acceptance criterion (spec §3.2). Take a mental note or a screenshot for comparison with Phase 1 browser view and Phase 2 final ElRobot render.

- [ ] **Step 4: Close the viewer**

Press Esc or close the window. No commit — this step is pure verification.

**Gate for Task 1.3:** viewer opens, arm renders stably, manual drag works smoothly. If any step fails, do NOT proceed to Chunk 2 — infrastructure may be broken.

---

### Chunk 1 Completion Summary

At the end of Chunk 1:

1. ✅ Menagerie cloned and structure verified
2. ✅ `trs_so_arm100/` vendored into repo with LICENSE + VENDOR.md
3. ✅ 1 commit: the vendor import
4. ✅ MuJoCo viewer smoke test passes on vendored files

**State checks before proceeding to Chunk 2:**

```bash
git log --oneline -1                # Should show the vendor commit
git status                           # Should be clean
find hardware/elrobot/simulation/vendor/menagerie -type f | wc -l   # > 2 files
```

**No MVP-1 regression check needed yet** — Chunk 1 touches no MVP-1 code; all 143 existing tests should still be green by definition. But as a sanity check you can run:

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/ -q
```

Expected: same number of tests passing as at main HEAD before starting MVP-2 (should be 31 in the `world/` subdirectory).

If any MVP-1 test regressed, STOP and investigate — something unexpected happened during the vendor import (possible: a conftest.py picked up files from the vendor directory).

---

**Next:** Chunk 2 starts the norma_sim source migration (code only). Chunk 3 follows up with the test-fixture migration.

---

## Chunk 2: `norma_sim` source migration — code only

**Purpose:** Rewrite `norma_sim.world.*` source files from parsing the MVP-1 `.world.yaml` schema (which listed 8 actuators explicitly and verified a `source_hash` against a URDF) to parsing the MVP-2 `.scene.yaml` schema (which enumerates actuators from the MJCF itself and uses annotations only for non-default capabilities like GRIPPER_PARALLEL). **Chunk 2 touches only source files and adds NEW unit tests against new code. It does not migrate pre-existing MVP-1 test files — that is Chunk 3's job.** Between the end of Chunk 2 and the end of Chunk 3 the pre-existing MVP-1 test files will be red/erroring (the "red window"); this is by design.

**Gate:** All new Chunk 2 unit tests pass. `source_hash` machinery is gone. `urdf_joint` field is renamed to `mjcf_joint` across all `norma_sim.world.*` source files. `_enumerate_mjcf_actuators(mjcf_path)` returns `(actuator_name, joint_name, type_tag)` tuples. `load_manifest(scene_yaml_path)` loads the Menagerie vendor MJCF successfully via the new schema. `MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)` loads without crashing. `make check-arch-invariants` still passes. `cargo test -p sim-runtime -p st3215-wire -p st3215-compat-bridge` unchanged (Chunk 2 touches zero Rust code).

**Prerequisites:** Chunk 1 complete. The directory `hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/` must exist and contain a loadable `scene.xml`.

**Red window warning:** Between Task 2.3 (which renames `urdf_joint` → `mjcf_joint` via a bulk rename) and Chunk 3 Task 3.x (which migrates MVP-1 test files to the new fixtures), any pre-existing test that calls `MuJoCoWorld.from_manifest_path(world_yaml_path)` or inspects `ActuatorManifest.urdf_joint` will be broken. This is expected. Do NOT pause mid-chunk to chase these failures — Chunk 3 closes the window. The Chunk 2 gate explicitly runs only the NEW tests, not the pre-existing ones.

**Files touched by Chunk 2:**
- Modify: `software/sim-server/norma_sim/world/manifest.py` — remove `source_hash`, rename field, rewrite `load_manifest`, add `_enumerate_mjcf_actuators`, `_synthesize_revolute_actuator`, `_parse_annotated_actuator`
- Modify: `software/sim-server/norma_sim/world/model.py` — remove `verify_source_hash` call, use `mjcf_joint` instead of `urdf_joint`
- Modify: `software/sim-server/norma_sim/world/actuation.py` — use `mjcf_joint` instead of `urdf_joint` (1 line)
- Modify: `software/sim-server/norma_sim/world/snapshot.py` — use `mjcf_joint` instead of `urdf_joint` (1 line)
- Modify: `software/sim-server/norma_sim/world/descriptor.py` — docstring only (signature unchanged)
- Modify: `software/sim-server/scripts/probe_manifest.py` — handle `urdf_path=None` gracefully (1 line)
- Create: `software/sim-server/tests/world/test_manifest_enumerate.py` — new unit tests for `_enumerate_mjcf_actuators`
- Create: `software/sim-server/tests/world/test_manifest_new_schema.py` — new unit tests for the new `.scene.yaml` loader
- Modify: `software/sim-server/tests/world/test_manifest_load.py` — delete the 3 `test_source_hash_*` tests only; leave the other tests alone (they stay broken until Chunk 3)

**Unchanged by Chunk 2:**
- `software/sim-server/norma_sim/world/capabilities.py` — pure functions over `ActuatorManifest`, field rename ripples via the dataclass
- `software/sim-server/norma_sim/world/_proto.py`
- `software/sim-server/norma_sim/ipc/*`
- `software/sim-server/norma_sim/cli.py` — updated in Chunk 3 Task 3.9
- `software/sim-server/tests/conftest.py` — updated in Chunk 3 Task 3.1
- All Rust crates

---

### Task 2.1: Baseline snapshot — capture pre-MVP-2 test state

**Files:** None (pure verification)

- [ ] **Step 1: Capture Python norma_sim pass count (should be 58)**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/ -q 2>&1 | tail -3
```

Expected: `58 passed in Xs` (MVP-1 baseline from `sim_starting_point.md`). Record exact count — it is the reference for Chunk 3 gate comparison.

- [ ] **Step 2: Capture Rust test counts (used to verify zero Rust changes at Chunk 2 gate)**

```bash
cargo test -p sim-runtime -p st3215-wire -p st3215-compat-bridge 2>&1 | grep 'test result'
```

Expected: 3 `test result: ok` lines with 28 / 15 / 16 passing (MVP-1 baseline). Record.

- [ ] **Step 3: Enumerate tests that will break in the red window**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_manifest_load.py --collect-only -q
```

Expected: 6 tests including `test_source_hash_matches`, `test_source_hash_mismatch_raises`, `test_source_hash_missing_comment_raises`, `test_manifest_load_happy`, `test_manifest_scene_config`, `test_manifest_missing_gripper_fields_raises`. The 3 source_hash tests are deleted in Task 2.2; the other 3 stay broken until Chunk 3 Task 3.7.

- [ ] **Step 4: No commit (pure verification)**

---

### Task 2.2: Delete `source_hash` machinery and its 3 tests

**Files:**
- Modify: `software/sim-server/norma_sim/world/manifest.py` (delete `verify_source_hash` and unused imports)
- Modify: `software/sim-server/norma_sim/world/model.py` (remove `verify_source_hash` call in `from_manifest_path`)
- Modify: `software/sim-server/norma_sim/cli.py` (remove `verify_source_hash` import + call at line 70)
- Modify: `software/sim-server/tests/world/test_manifest_load.py` (delete 3 `test_source_hash_*` functions only)

- [ ] **Step 1: Delete the 3 source_hash tests AND remove the `verify_source_hash` import from `test_manifest_load.py`**

Use the Edit tool twice:

1. Remove the 3 functions: `test_source_hash_matches`, `test_source_hash_mismatch_raises`, `test_source_hash_missing_comment_raises`. Leave the remaining 3 tests (`test_manifest_load_happy`, `test_manifest_scene_config`, `test_manifest_missing_gripper_fields_raises`) in place — they are migrated in Chunk 3 Task 3.7.

2. Edit the import statement at the top of `test_manifest_load.py` (around line 10-15) to remove `verify_source_hash` from the import list. MVP-1 baseline has:

```python
from norma_sim.world.manifest import (
    ...
    verify_source_hash,
    ...
)
```

Remove the `verify_source_hash,` line. If any other helper in the import list becomes orphaned (e.g., something used only by the deleted tests), remove it too. Verify with:

```bash
grep -n 'verify_source_hash' software/sim-server/tests/world/test_manifest_load.py
```

Expected: no matches. Without this fix, Task 2.7 Step 6's pytest collection will fail with `ImportError: cannot import name 'verify_source_hash' from 'norma_sim.world.manifest'` at module load time, before any test runs.

- [ ] **Step 2: Delete `verify_source_hash` from `manifest.py`**

Edit `software/sim-server/norma_sim/world/manifest.py`:

- Delete the entire `verify_source_hash` function (starts at line 186 in MVP-1 baseline: `def verify_source_hash(manifest_path: Path, mjcf_path: Path) -> None:`).
- Delete the `# --- source_hash verification` comment block header that precedes it.
- Delete `import hashlib` and `import re` at the top of the file (they are used only by `verify_source_hash`).
- Verify no other function references `verify_source_hash` or `hashlib.sha256` or `re.search`:

```bash
grep -n 'hashlib\|re\.\|verify_source_hash\|source_hash' software/sim-server/norma_sim/world/manifest.py
```

Expected: no matches.

- [ ] **Step 2b: Remove `verify_source_hash` call from `cli.py`**

`software/sim-server/norma_sim/cli.py` imports `verify_source_hash` (MVP-1 baseline line 32) and calls it (line 70). Remove both:

1. Change the import:

```python
# before:
from .world.manifest import load_manifest, verify_source_hash
# after:
from .world.manifest import load_manifest
```

2. Delete the call site (MVP-1 baseline line 70):

```python
# delete:
verify_source_hash(args.manifest, manifest.mjcf_path)
```

Verify:

```bash
grep -n 'verify_source_hash' software/sim-server/norma_sim/cli.py
```

Expected: no matches. Without this step, `python -m norma_sim --help` would `ImportError` at module load time for the duration of the red window, and `test_full_loop.py` (Chunk 3 Task 3.9) would fail to spawn the subprocess.

- [ ] **Step 3: Remove `verify_source_hash` call from `model.py`**

In `software/sim-server/norma_sim/world/model.py`, the current `from_manifest_path` has:

```python
@classmethod
def from_manifest_path(cls, manifest_path, verify_hash: bool = True) -> "MuJoCoWorld":
    from .manifest import load_manifest
    manifest = load_manifest(manifest_path)
    if verify_hash:
        verify_source_hash(manifest_path, manifest.mjcf_path)
    return cls(manifest, verify_hash=False)
```

Rewrite to:

```python
@classmethod
def from_manifest_path(cls, manifest_path) -> "MuJoCoWorld":
    """Canonical constructor: load the scene yaml, open the MJCF it
    references, build manifest + MuJoCo model in one call."""
    from .manifest import load_manifest
    manifest = load_manifest(manifest_path)
    return cls(manifest)
```

Also:
- Remove `verify_hash: bool = True` parameter from `__init__` and delete the body `if verify_hash: pass` block (lines 23-32 in MVP-1 baseline).
- Remove `from .manifest import ... verify_source_hash` from the import list at the top of the file.

- [ ] **Step 4: Run the remaining norma_sim tests to verify imports are clean**

```bash
PYTHONPATH=software/sim-server python3 -c "from norma_sim.world.manifest import load_manifest; from norma_sim.world.model import MuJoCoWorld; print('imports OK')"
```

Expected: `imports OK`.

Do NOT run the full test suite yet — at this point the MVP-1 tests will fail because `test_manifest_load.py` has dangling fixture references. That's expected.

- [ ] **Step 5: Commit**

```bash
git add software/sim-server/norma_sim/world/manifest.py \
        software/sim-server/norma_sim/world/model.py \
        software/sim-server/norma_sim/cli.py \
        software/sim-server/tests/world/test_manifest_load.py
git commit -m "norma_sim: remove source_hash machinery (MVP-2)"
```

Commit body (via `$(cat <<'EOF' ... EOF)` pattern from Task 1.2 Step 6):

```
MVP-1 verified that a generated MJCF's embedded source_hash matched
sha256(urdf_bytes + manifest_bytes). That was a safety net for the
URDF → MJCF gen.py pipeline. Under MVP-2's MJCF-first architecture,
MJCF is hand-written and the pipeline is deleted, so the hash check
is meaningless.

Removed:
- verify_source_hash() function from manifest.py
- hashlib / re imports (no other callers)
- 3 test_source_hash_* tests from test_manifest_load.py
- verify_hash parameter from MuJoCoWorld.__init__ / .from_manifest_path
```

---

### Task 2.3: Rename `urdf_joint` → `mjcf_joint` across all `norma_sim.world.*` source files

**Files:**
- Modify: `software/sim-server/norma_sim/world/manifest.py` (`ActuatorManifest.urdf_joint` → `mjcf_joint`; `_parse_actuator` field mapping)
- Modify: `software/sim-server/norma_sim/world/model.py` (`_build_lookups` uses `act.mjcf_joint`)
- Modify: `software/sim-server/norma_sim/world/actuation.py` (1 line: `actuator.urdf_joint` → `actuator.mjcf_joint`)
- Modify: `software/sim-server/norma_sim/world/snapshot.py` (1 line: `act.urdf_joint` → `act.mjcf_joint`)

**Rationale:** The MVP-1 `ActuatorManifest` has a field named `urdf_joint` that stores the name of the joint the actuator controls. In MVP-2 we parse the field from the MJCF (there's no URDF in the sim path), so the name is semantically wrong. Rename it to `mjcf_joint` **before** rewriting `load_manifest`, so downstream changes to `load_manifest` can set the new field name directly. This is a pure rename — behaviour is unchanged at this task's commit, but the rename cascades through `model.py`, `actuation.py`, and `snapshot.py`. Capability and descriptor modules don't touch this field so they are unchanged.

**Blocker note:** Without this rename, Task 2.5's `_synthesize_revolute_actuator` would have to set `urdf_joint=""` (because there's no URDF), which makes `MuJoCoWorld._build_lookups()` call `mj_name2id(model, mjOBJ_JOINT, "")` → returns -1 → raises `ValueError`. This task prevents that.

- [ ] **Step 1: Locate all call sites**

```bash
grep -rn 'urdf_joint' software/sim-server/norma_sim/ software/sim-server/tests/ software/sim-server/scripts/
```

Expected call sites (from MVP-1 baseline):
- `norma_sim/world/manifest.py`: `ActuatorManifest` dataclass field + `_parse_actuator` that reads `raw["urdf_joint"]`
- `norma_sim/world/model.py`: `_build_lookups` reads `act.urdf_joint` and calls `mj_name2id`
- `norma_sim/world/actuation.py`: reads `actuator.urdf_joint` (1 site)
- `norma_sim/world/snapshot.py`: reads `act.urdf_joint` (1 site)
- `tests/world/test_capabilities.py`: test fixtures construct `ActuatorManifest(urdf_joint=...)` (2 sites)
- `tests/world/test_manifest_load.py`: 1 site (in `test_manifest_load_happy`)
- Possibly `scripts/probe_manifest.py` if it prints the field

Record the exact list.

- [ ] **Step 2: Rename in `manifest.py` dataclass + parser**

In `software/sim-server/norma_sim/world/manifest.py`:

```python
@dataclass(frozen=True)
class ActuatorManifest:
    actuator_id: str
    display_name: str
    mjcf_joint: str  # was urdf_joint — in MVP-2 this is the MJCF joint name
    mjcf_actuator: str
    capability: ActuatorCapability
    actuator_gains: dict
    gripper: Optional[GripperMeta] = None
```

And in `_parse_actuator`:

```python
return ActuatorManifest(
    actuator_id=raw["actuator_id"],
    display_name=raw["display_name"],
    mjcf_joint=raw["urdf_joint"],  # MVP-1 yaml still calls it urdf_joint;
                                    # the field rename normalizes within the
                                    # dataclass even when reading legacy yaml
    mjcf_actuator=raw["mjcf_actuator"],
    capability=cap,
    actuator_gains=dict(raw["actuator_gains"]),
    gripper=gripper,
)
```

(Note: `_parse_actuator` is the legacy MVP-1 parser. It remains briefly until Task 2.5 replaces it. The temporary use of legacy yaml field name `raw["urdf_joint"]` is intentional — we preserve backward-reading during the transition; Task 2.5 deletes `_parse_actuator` entirely.)

- [ ] **Step 3: Rename in `model.py::_build_lookups`**

In `software/sim-server/norma_sim/world/model.py`, change the two occurrences of `act.urdf_joint` inside `_build_lookups` to `act.mjcf_joint`:

```python
def _build_lookups(self) -> None:
    for robot in self.manifest.robots:
        for act in robot.actuators:
            idx = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act.mjcf_actuator
            )
            if idx < 0:
                raise ValueError(
                    f"MJCF has no <position name='{act.mjcf_actuator}'> "
                    f"for manifest actuator '{act.actuator_id}'"
                )
            self._actuator_id_cache[act.mjcf_actuator] = idx
            joint_idx = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, act.mjcf_joint
            )
            if joint_idx < 0:
                raise ValueError(
                    f"MJCF has no joint '{act.mjcf_joint}' for "
                    f"manifest actuator '{act.actuator_id}'"
                )
            self._joint_qposadr_cache[act.mjcf_joint] = int(
                self.model.jnt_qposadr[joint_idx]
            )
```

- [ ] **Step 4: Rename in `actuation.py` and `snapshot.py`**

Use Grep to find exact line numbers:

```bash
grep -n 'urdf_joint' software/sim-server/norma_sim/world/actuation.py software/sim-server/norma_sim/world/snapshot.py
```

For each hit, use Edit to change `.urdf_joint` → `.mjcf_joint`. Should be 1 site in each file.

- [ ] **Step 5: Rename `joint_qposadr_for` parameter `urdf_joint` → `mjcf_joint` in `model.py`**

In `software/sim-server/norma_sim/world/model.py`, the public method signature at line 81 (MVP-1 baseline) is:

```python
def joint_qposadr_for(self, urdf_joint: str) -> Optional[int]:
    return self._joint_qposadr_cache.get(urdf_joint)
```

Rename the parameter and its body reference:

```python
def joint_qposadr_for(self, mjcf_joint: str) -> Optional[int]:
    return self._joint_qposadr_cache.get(mjcf_joint)
```

(Safe: the parameter is passed positionally at both call sites in `actuation.py` and `snapshot.py` after their Step 4 updates.)

- [ ] **Step 6: Verify no `urdf_joint` references remain in `norma_sim/world/*`**

```bash
grep -rn 'urdf_joint' software/sim-server/norma_sim/world/
```

Expected: no matches. (If there are any, they're in comments or docstrings — fix those too, since the rename is semantic.)

- [ ] **Step 7: Smoke test — imports still work**

```bash
PYTHONPATH=software/sim-server python3 -c "
from norma_sim.world.manifest import ActuatorManifest, load_manifest
from norma_sim.world.model import MuJoCoWorld
from norma_sim.world.actuation import ActuationApplier
from norma_sim.world.snapshot import SnapshotBuilder
print('all imports OK')
assert hasattr(ActuatorManifest, '__dataclass_fields__')
assert 'mjcf_joint' in ActuatorManifest.__dataclass_fields__
assert 'urdf_joint' not in ActuatorManifest.__dataclass_fields__
print('field rename confirmed')
"
```

Expected: `all imports OK` then `field rename confirmed`.

Do NOT run the test suite. MVP-1 test fixtures still reference `urdf_joint` in some places (e.g., `test_capabilities.py` constructs ActuatorManifest by keyword) — Task 3.2 and later tasks fix those.

- [ ] **Step 8: Commit**

```bash
git add software/sim-server/norma_sim/world/manifest.py \
        software/sim-server/norma_sim/world/model.py \
        software/sim-server/norma_sim/world/actuation.py \
        software/sim-server/norma_sim/world/snapshot.py
git commit -m "norma_sim: rename ActuatorManifest.urdf_joint → mjcf_joint"
```

Commit body:

```
In MVP-2 there is no URDF in the sim path; the field stores the
MJCF joint name that the actuator controls. Rename the field to
match its MVP-2 semantic, and propagate through all source
call sites: model.py (lookups), actuation.py, snapshot.py.

test_capabilities.py and test_manifest_load.py still reference
the old field name; those files are migrated in Chunk 3.
```

---

### Task 2.4: Add `_enumerate_mjcf_actuators` helper returning `(name, joint, type)` tuples

**Files:**
- Modify: `software/sim-server/norma_sim/world/manifest.py` (add `_enumerate_mjcf_actuators` private helper)
- Create: `software/sim-server/tests/world/test_manifest_enumerate.py` (new test file)

**Rationale:** The new `.scene.yaml` schema does not list actuators — they are enumerated from the MJCF at load time. The enumerator must return **both** the actuator name AND the joint name it controls (so Task 2.5's `_synthesize_revolute_actuator` can populate `mjcf_joint`). The third tuple element (`type_tag`) distinguishes `<position>` (synthesized as REVOLUTE_POSITION) from `<motor>` / `<general>` (require explicit annotation).

- [ ] **Step 1: Write the failing test**

Create `software/sim-server/tests/world/test_manifest_enumerate.py`:

```python
"""Tests for manifest._enumerate_mjcf_actuators — the MJCF → actuator list
helper used by the MVP-2 scene.yaml loader. Tests run against Menagerie's
vendored trs_so_arm100 MJCF (Chunk 1 dependency)."""
from pathlib import Path

import pytest

from norma_sim.world.manifest import _enumerate_mjcf_actuators


@pytest.fixture
def menagerie_scene_xml() -> Path:
    """Locate the Chunk 1 vendored Menagerie MJCF without relying on
    conftest.py (which is migrated in Chunk 3)."""
    here = Path(__file__).resolve()
    # parents: [0]=tests/world, [1]=tests, [2]=sim-server, [3]=software, [4]=repo
    repo_root = here.parents[4]
    p = repo_root / "hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml"
    if not p.exists():
        pytest.skip(f"Menagerie vendor not found at {p}; Chunk 1 has not been run yet")
    return p


def test_enumerate_menagerie_returns_nonempty_list(menagerie_scene_xml: Path):
    actuators = _enumerate_mjcf_actuators(menagerie_scene_xml)
    assert len(actuators) >= 5, (
        f"Menagerie trs_so_arm100 should have >= 5 actuators, got {len(actuators)}"
    )


def test_enumerate_returns_three_tuple_name_joint_type(menagerie_scene_xml: Path):
    actuators = _enumerate_mjcf_actuators(menagerie_scene_xml)
    for entry in actuators:
        assert len(entry) == 3, f"expected 3-tuple, got {entry}"
        name, joint, type_tag = entry
        assert isinstance(name, str) and name
        assert isinstance(joint, str) and joint
        assert type_tag in ("position", "motor", "general", "velocity"), (
            f"unexpected actuator type: {type_tag}"
        )


def test_enumerate_joint_name_resolves_in_mjcf(menagerie_scene_xml: Path):
    """Verify each returned joint name actually exists in the MJCF
    (so MuJoCoWorld._build_lookups won't fail when it constructs
    from a synthesized ActuatorManifest)."""
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(menagerie_scene_xml))
    actuators = _enumerate_mjcf_actuators(menagerie_scene_xml)
    for name, joint, _ in actuators:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        assert joint_id >= 0, (
            f"enumerator returned joint '{joint}' for actuator '{name}' "
            f"but MJCF has no such joint"
        )


def test_enumerate_raises_on_nonexistent_file(tmp_path: Path):
    with pytest.raises((FileNotFoundError, ValueError)):
        _enumerate_mjcf_actuators(tmp_path / "does_not_exist.xml")
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_manifest_enumerate.py -v
```

Expected: `ImportError: cannot import name '_enumerate_mjcf_actuators'`.

- [ ] **Step 3: Implement `_enumerate_mjcf_actuators`**

Add to `software/sim-server/norma_sim/world/manifest.py` (near the bottom of the file):

```python
def _enumerate_mjcf_actuators(mjcf_path: Path) -> list[tuple[str, str, str]]:
    """Parse an MJCF file via MuJoCo's compiler (which resolves <include>)
    and return the actuator list as `(actuator_name, joint_name, type_tag)`
    tuples.

    `type_tag` values:
      - "position" — `<position>` actuator: synthesized as REVOLUTE_POSITION
        when no annotation is provided
      - "motor"    — `<motor>` actuator: requires explicit annotation
      - "general"  — `<general>` actuator: requires explicit annotation
      - "velocity" — `<velocity>` actuator: requires explicit annotation

    The type distinction is derived from the gain/bias type enum pair:
      position: gain=FIXED, bias=AFFINE
      motor:    gain=FIXED, bias=NONE
      general:  anything else
    """
    import mujoco  # imported lazily so this module stays lightweight

    if not mjcf_path.exists():
        raise FileNotFoundError(f"MJCF not found: {mjcf_path}")

    try:
        model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    except Exception as e:
        raise ValueError(f"failed to compile MJCF {mjcf_path}: {e}") from e

    # Resolve enum values via the typed enums (robust to MuJoCo version bumps)
    gain_fixed = int(mujoco.mjtGain.mjGAIN_FIXED)
    bias_affine = int(mujoco.mjtBias.mjBIAS_AFFINE)
    bias_none = int(mujoco.mjtBias.mjBIAS_NONE)

    results: list[tuple[str, str, str]] = []
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if not name:
            continue  # skip unnamed actuators (rare)
        gain_type = int(model.actuator_gaintype[i])
        bias_type = int(model.actuator_biastype[i])
        if gain_type == gain_fixed and bias_type == bias_affine:
            type_tag = "position"
        elif gain_type == gain_fixed and bias_type == bias_none:
            type_tag = "motor"
        else:
            type_tag = "general"

        # Resolve the joint name this actuator controls.
        # actuator_trntype[i] can be JOINT (1) or other (tendon, site).
        # actuator_trnid[i, 0] is the joint id when trntype == JOINT.
        joint_type = int(mujoco.mjtTrn.mjTRN_JOINT)
        if int(model.actuator_trntype[i]) != joint_type:
            continue  # non-joint actuators (tendons, sites) are not supported in MVP-2
        joint_id = int(model.actuator_trnid[i, 0])
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if not joint_name:
            continue  # actuator controlling unnamed joint — rare edge case, skip

        results.append((name, joint_name, type_tag))
    return results
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_manifest_enumerate.py -v
```

Expected: 4 tests pass.

- [ ] **Step 5: Commit**

```bash
git add software/sim-server/norma_sim/world/manifest.py \
        software/sim-server/tests/world/test_manifest_enumerate.py
git commit -m "norma_sim: add _enumerate_mjcf_actuators helper"
```

Commit body:

```
Returns (actuator_name, joint_name, type_tag) tuples by compiling the
MJCF via mujoco.MjModel and reading actuator_gaintype / actuator_biastype
/ actuator_trnid. type_tag classifies <position> / <motor> / <general>
so Task 2.5's load_manifest can auto-synthesize REVOLUTE_POSITION
ActuatorManifest for unannotated <position> actuators. joint_name
resolves the MJCF joint the actuator controls — used to populate
ActuatorManifest.mjcf_joint without requiring yaml input.

Tests run against Chunk 1's vendored Menagerie trs_so_arm100.
```

---

### Task 2.5: Rewrite `load_manifest` for the MVP-2 `.scene.yaml` schema

**Files:**
- Modify: `software/sim-server/norma_sim/world/manifest.py` (replace `load_manifest` body, add `_synthesize_revolute_actuator`, `_parse_annotated_actuator`; delete old `_parse_actuator`, `_parse_sensor`; make `WorldManifest.urdf_path` Optional)
- Create: `software/sim-server/tests/world/test_manifest_new_schema.py` (new test file)

**Rationale:** This is the core of the migration. The new schema has `world_name`, `mjcf_path`, optional `scene_overrides`, optional `scene_extras`, and optional `actuator_annotations`. Revolute actuators are synthesized from MJCF enumeration; only non-default capabilities (GRIPPER_PARALLEL) need annotations.

**Spec alignment note:** Per spec §8.1 yaml example, `normalized_range` lives under `capability:` and `primary_joint_range_rad` + `mimic_joints` live under `gripper:`. The loader must read these from the correct locations.

- [ ] **Step 1: Write the failing tests (5 tests)**

Create `software/sim-server/tests/world/test_manifest_new_schema.py`:

```python
"""Tests for the MVP-2 scene.yaml schema loader. Uses tmp_path to
build test fixtures on the fly, referencing the Menagerie vendored
MJCF for `mjcf_path`."""
from pathlib import Path

import pytest

from norma_sim.world.manifest import _enumerate_mjcf_actuators, load_manifest


@pytest.fixture
def menagerie_mjcf_path() -> Path:
    here = Path(__file__).resolve()
    # parents: [0]=tests/world, [1]=tests, [2]=sim-server, [3]=software, [4]=repo
    repo_root = here.parents[4]
    p = repo_root / "hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml"
    if not p.exists():
        pytest.skip(f"Menagerie vendor not found at {p}; run Chunk 1 first")
    return p


def _write_minimal_scene(tmp_path: Path, mjcf_path: Path) -> Path:
    scene_yaml = tmp_path / "minimal.scene.yaml"
    scene_yaml.write_text(
        f"world_name: test_world\n"
        f"mjcf_path: {mjcf_path}\n"
    )
    return scene_yaml


def test_minimal_scene_yaml_loads(tmp_path: Path, menagerie_mjcf_path: Path):
    """Simplest valid scene.yaml: world_name + mjcf_path."""
    scene_yaml = _write_minimal_scene(tmp_path, menagerie_mjcf_path)
    manifest = load_manifest(scene_yaml)
    assert manifest.world_name == "test_world"
    assert manifest.mjcf_path == menagerie_mjcf_path.resolve()
    assert len(manifest.robots) == 1
    assert len(manifest.robots[0].actuators) >= 5


def test_scene_yaml_synthesizes_revolute_actuators_with_mjcf_joint(
    tmp_path: Path, menagerie_mjcf_path: Path
):
    """Revolute <position> actuators should become REVOLUTE_POSITION
    ActuatorManifest entries with `mjcf_joint` populated from the MJCF."""
    scene_yaml = _write_minimal_scene(tmp_path, menagerie_mjcf_path)
    manifest = load_manifest(scene_yaml)
    revolute = [
        a for r in manifest.robots for a in r.actuators
        if a.capability.kind == "REVOLUTE_POSITION"
    ]
    assert len(revolute) >= 4
    for act in revolute:
        assert act.mjcf_joint, f"mjcf_joint empty on {act.actuator_id}"
        assert act.mjcf_actuator, f"mjcf_actuator empty on {act.actuator_id}"


def test_scene_yaml_annotation_overrides_capability(
    tmp_path: Path, menagerie_mjcf_path: Path
):
    """An actuator_annotation with kind=GRIPPER_PARALLEL overrides the
    default REVOLUTE_POSITION synthesis. Uses dynamic actuator discovery
    so the test doesn't hardcode a specific Menagerie joint name."""
    actuators = _enumerate_mjcf_actuators(menagerie_mjcf_path)
    assert len(actuators) > 0
    # Pick the last actuator as the "stand-in gripper" for test purposes
    target_mjcf_name, target_joint, _ = actuators[-1]

    scene_yaml = tmp_path / "with_annotation.scene.yaml"
    scene_yaml.write_text(
        f"world_name: test_world\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
        f"actuator_annotations:\n"
        f"  - mjcf_actuator: {target_mjcf_name}\n"
        f"    actuator_id: test_gripper\n"
        f"    display_name: Test Gripper\n"
        f"    capability:\n"
        f"      kind: GRIPPER_PARALLEL\n"
        f"      normalized_range: [0.0, 1.0]\n"
        f"    gripper:\n"
        f"      primary_joint_range_rad: [0.0, 1.0]\n"
        f"      mimic_joints: []\n"
    )
    manifest = load_manifest(scene_yaml)
    gripper_acts = [
        a for r in manifest.robots for a in r.actuators
        if a.capability.kind == "GRIPPER_PARALLEL"
    ]
    assert len(gripper_acts) == 1
    g = gripper_acts[0]
    assert g.actuator_id == "test_gripper"
    assert g.display_name == "Test Gripper"
    assert g.mjcf_joint == target_joint  # populated from MJCF via annotation
    assert g.gripper is not None
    assert g.gripper.normalized_range == (0.0, 1.0)
    assert g.gripper.primary_joint_range_rad == (0.0, 1.0)


def test_scene_yaml_missing_mjcf_path_raises(tmp_path: Path):
    scene_yaml = tmp_path / "bad.scene.yaml"
    scene_yaml.write_text("world_name: test\n")
    with pytest.raises((ValueError, KeyError)):
        load_manifest(scene_yaml)


def test_scene_yaml_annotation_for_nonexistent_actuator_raises(
    tmp_path: Path, menagerie_mjcf_path: Path
):
    scene_yaml = tmp_path / "bad_annotation.scene.yaml"
    scene_yaml.write_text(
        f"world_name: test_world\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
        f"actuator_annotations:\n"
        f"  - mjcf_actuator: actuator_that_does_not_exist\n"
        f"    actuator_id: fake\n"
        f"    display_name: Fake\n"
        f"    capability:\n"
        f"      kind: GRIPPER_PARALLEL\n"
        f"      normalized_range: [0.0, 1.0]\n"
        f"    gripper:\n"
        f"      primary_joint_range_rad: [0.0, 1.0]\n"
        f"      mimic_joints: []\n"
    )
    with pytest.raises(ValueError, match="no such actuator|not found"):
        load_manifest(scene_yaml)
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_manifest_new_schema.py -v
```

Expected: all fail with `KeyError: 'urdf_source'` or similar (the old parser still requires legacy fields).

- [ ] **Step 3: Update `WorldManifest` dataclass to make `urdf_path` optional**

In `software/sim-server/norma_sim/world/manifest.py`, keep the existing field types unchanged and add only the `urdf_path` default:

```python
@dataclass(frozen=True)
class WorldManifest:
    world_name: str
    scene: SceneConfig
    robots: tuple  # tuple[RobotManifest, ...]  (keep MVP-1 type annotation as-is)
    mjcf_path: Path
    urdf_path: Optional[Path] = None  # MVP-2: sim no longer consumes URDF
```

(Do NOT tighten the `robots` type annotation as a drive-by change — this task is scoped to `urdf_path` only.)

- [ ] **Step 4: Replace `load_manifest` function body**

Replace the entire `load_manifest` function in `software/sim-server/norma_sim/world/manifest.py` with:

```python
DEFAULT_ROBOT_ID = "default_robot"


def load_manifest(manifest_path: Path) -> WorldManifest:
    """Load and validate an MVP-2 scene.yaml.

    Schema (see spec §8.1):

        world_name: str               # required
        mjcf_path: str                # required, relative to the yaml file
        robot_id: str                 # optional, default='default_robot'
        scene_overrides:              # optional, overrides MJCF <option>
          timestep: float
          gravity: [x, y, z]
          integrator: str
          solver: str
          iterations: int
        scene_extras:                 # optional, runtime-added worldbody items
          lights: [...]
          floor: {...}
        actuator_annotations:         # optional; only for non-default capabilities
          - mjcf_actuator: str        # must exist in MJCF
            actuator_id: str          # id used by bridge + descriptor
            display_name: str
            capability:
              kind: REVOLUTE_POSITION | PRISMATIC_POSITION | GRIPPER_PARALLEL
              limit_min: float        # optional
              limit_max: float
              effort_limit: float
              velocity_limit: float
              normalized_range: [lo, hi]   # required when kind=GRIPPER_PARALLEL
            gripper:                  # required when kind=GRIPPER_PARALLEL
              primary_joint_range_rad: [lo, hi]
              mimic_joints:
                - {joint: str, multiplier: float}

    Actuators in the MJCF that are NOT listed in `actuator_annotations`
    and are MuJoCo `<position>` type are auto-synthesized as
    REVOLUTE_POSITION ActuatorManifest entries. `<motor>`, `<velocity>`,
    or `<general>` actuators without annotation are silently skipped
    (MVP-2 only ships the REVOLUTE_POSITION default).
    """
    manifest_path = Path(manifest_path)  # accept str too, matches MVP-1 duck-typing
    with manifest_path.open() as f:
        raw = yaml.safe_load(f) or {}

    if "mjcf_path" not in raw:
        raise ValueError(
            f"scene.yaml {manifest_path} missing required 'mjcf_path'"
        )
    if "world_name" not in raw:
        raise ValueError(
            f"scene.yaml {manifest_path} missing required 'world_name'"
        )

    manifest_dir = manifest_path.parent
    mjcf_path = (manifest_dir / raw["mjcf_path"]).resolve()
    if not mjcf_path.exists():
        raise ValueError(
            f"scene.yaml {manifest_path} references non-existent "
            f"mjcf_path: {mjcf_path}"
        )

    # Scene config — overrides MJCF <option>. Defaults match MVP-1 baseline
    # for backward compatibility when a yaml omits scene_overrides entirely.
    scene_overrides = raw.get("scene_overrides") or {}
    scene = SceneConfig(
        timestep=float(scene_overrides.get("timestep", 0.002)),
        gravity=tuple(scene_overrides.get("gravity", [0.0, 0.0, -9.81])),
        integrator=scene_overrides.get("integrator", "RK4"),
        solver=scene_overrides.get("solver", "Newton"),
        iterations=int(scene_overrides.get("iterations", 50)),
    )

    # Enumerate MJCF actuators → (name, joint_name, type_tag)
    mjcf_actuators = _enumerate_mjcf_actuators(mjcf_path)
    mjcf_actuator_names = {name for name, _, _ in mjcf_actuators}

    # Build annotation lookup (keyed by mjcf_actuator name)
    annotations = raw.get("actuator_annotations") or []
    annotation_by_name: dict[str, dict] = {}
    for ann in annotations:
        if "mjcf_actuator" not in ann:
            raise ValueError(
                f"actuator_annotation in {manifest_path} missing "
                f"required field 'mjcf_actuator'"
            )
        mjcf_name = ann["mjcf_actuator"]
        if mjcf_name not in mjcf_actuator_names:
            raise ValueError(
                f"actuator_annotation references mjcf_actuator "
                f"'{mjcf_name}' but no such actuator exists in "
                f"{mjcf_path}. Available: {sorted(mjcf_actuator_names)}"
            )
        annotation_by_name[mjcf_name] = ann

    # Synthesize ActuatorManifest list. Annotation takes precedence;
    # otherwise default to REVOLUTE_POSITION for <position> actuators.
    actuators: list[ActuatorManifest] = []
    joint_by_mjcf_name = {name: joint for name, joint, _ in mjcf_actuators}
    for mjcf_name, joint_name, type_tag in mjcf_actuators:
        if mjcf_name in annotation_by_name:
            actuators.append(
                _parse_annotated_actuator(
                    annotation_by_name[mjcf_name], joint_name
                )
            )
        elif type_tag == "position":
            actuators.append(
                _synthesize_revolute_actuator(mjcf_name, joint_name)
            )
        else:
            # <motor> / <general> / <velocity> without annotation → skip
            continue

    robots = (
        RobotManifest(
            robot_id=raw.get("robot_id", DEFAULT_ROBOT_ID),
            actuators=tuple(actuators),
            sensors=(),  # MVP-2 does not consume sensors; see spec §2.3 deferred
        ),
    )

    return WorldManifest(
        world_name=raw["world_name"],
        scene=scene,
        robots=robots,
        mjcf_path=mjcf_path,
        urdf_path=None,
    )


def _synthesize_revolute_actuator(
    mjcf_name: str, mjcf_joint: str
) -> ActuatorManifest:
    """Default ActuatorManifest for a <position> actuator with no
    scene.yaml annotation. actuator_id = mjcf_name, display_name humanized.
    All capability limits left as None (MJCF's ctrlrange / forcerange is
    the source of truth — downstream code reads them from the MjModel,
    not from the manifest)."""
    return ActuatorManifest(
        actuator_id=mjcf_name,
        display_name=mjcf_name.replace("_", " ").title(),
        mjcf_joint=mjcf_joint,
        mjcf_actuator=mjcf_name,
        capability=ActuatorCapability(kind="REVOLUTE_POSITION"),
        actuator_gains={},
        gripper=None,
    )


def _parse_annotated_actuator(
    ann: dict, mjcf_joint: str
) -> ActuatorManifest:
    """Parse an actuator_annotations entry into ActuatorManifest.
    `mjcf_joint` is resolved by the caller from MJCF (not from yaml)."""
    cap_raw = ann["capability"]
    cap = ActuatorCapability(
        kind=cap_raw["kind"],
        limit_min=cap_raw.get("limit_min"),
        limit_max=cap_raw.get("limit_max"),
        effort_limit=cap_raw.get("effort_limit"),
        velocity_limit=cap_raw.get("velocity_limit"),
    )
    gripper: Optional[GripperMeta] = None
    if cap.kind == "GRIPPER_PARALLEL":
        if "normalized_range" not in cap_raw:
            raise ValueError(
                f"GRIPPER_PARALLEL capability on '{ann['mjcf_actuator']}' "
                f"missing 'normalized_range' (should live under capability:)"
            )
        normalized_range = tuple(cap_raw["normalized_range"])
        g_raw = ann.get("gripper")
        if g_raw is None:
            raise ValueError(
                f"actuator_annotation for '{ann['mjcf_actuator']}' has "
                f"kind GRIPPER_PARALLEL but no 'gripper:' metadata"
            )
        mimic = tuple(
            GripperMimic(joint=m["joint"], multiplier=float(m["multiplier"]))
            for m in g_raw.get("mimic_joints", [])
        )
        gripper = GripperMeta(
            primary_joint_range_rad=tuple(g_raw["primary_joint_range_rad"]),
            normalized_range=normalized_range,
            mimic_joints=mimic,
        )
    return ActuatorManifest(
        actuator_id=ann["actuator_id"],
        display_name=ann["display_name"],
        mjcf_joint=mjcf_joint,
        mjcf_actuator=ann["mjcf_actuator"],
        capability=cap,
        actuator_gains={},
        gripper=gripper,
    )
```

Also delete the old `_parse_actuator` and `_parse_sensor` private functions from `manifest.py` — they are fully replaced. And verify no remaining legacy-schema references:

```bash
grep -n 'urdf_source\|mjcf_output\|_parse_actuator\|_parse_sensor' software/sim-server/norma_sim/world/manifest.py
```

Expected: no matches (except possibly in comments that should also be deleted).

- [ ] **Step 5: Run the new tests to verify they pass**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_manifest_new_schema.py -v
```

Expected: 5 tests pass.

- [ ] **Step 6: Functional smoke test — `MuJoCoWorld.from_manifest_path` works end-to-end**

This is the critical verification that the rename in Task 2.3 + the new loader in Task 2.5 let `_build_lookups` succeed:

```bash
PYTHONPATH=software/sim-server python3 -c "
from pathlib import Path
from norma_sim.world.model import MuJoCoWorld

here = Path.cwd()
mjcf = here / 'hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml'
scene = here / '/tmp/probe.scene.yaml'
scene.write_text(f'world_name: probe\nmjcf_path: {mjcf}\n')

world = MuJoCoWorld.from_manifest_path(scene)
print(f'nu={world.model.nu}, nv={world.model.nv}')
print(f'lookups: {len(world._actuator_id_cache)} actuators, {len(world._joint_qposadr_cache)} joints')
assert world.model.nu > 0
assert len(world._actuator_id_cache) > 0
print('MuJoCoWorld smoke test: PASS')
"
```

Expected: prints `nu=5` (or whatever Menagerie has), lookups counts, and `PASS`.

**If this fails with `ValueError: MJCF has no joint`**: Task 2.3 rename is incomplete OR `_enumerate_mjcf_actuators` returned a wrong joint name. Debug before committing.

- [ ] **Step 7: Commit**

```bash
git add software/sim-server/norma_sim/world/manifest.py \
        software/sim-server/tests/world/test_manifest_new_schema.py
git commit -m "norma_sim: rewrite load_manifest for MVP-2 scene.yaml schema"
```

Commit body:

```
The new schema:
- world_name + mjcf_path required
- scene_overrides optional (override MJCF <option>)
- actuator_annotations optional; only for GRIPPER_PARALLEL or other
  non-default capabilities. Revolute actuators are synthesized from
  MJCF <position> enumeration (via _enumerate_mjcf_actuators).

Replaces old _parse_actuator/_parse_sensor with:
- _synthesize_revolute_actuator(mjcf_name, mjcf_joint)
- _parse_annotated_actuator(ann, mjcf_joint)

mjcf_joint comes from _enumerate_mjcf_actuators, not yaml — this is
why Task 2.3 renamed ActuatorManifest.urdf_joint to mjcf_joint first.

Gripper yaml placement matches spec §8.1:
  capability.normalized_range: [lo, hi]
  gripper.primary_joint_range_rad: [lo, hi]
  gripper.mimic_joints: [...]

WorldManifest.urdf_path is now Optional[Path], default None.

Tests use tmp_path + Menagerie vendored MJCF (Chunk 1 dependency).
```

---

### Task 2.6: Update `descriptor.py` docstring + `probe_manifest.py` for Optional urdf_path

**Files:**
- Modify: `software/sim-server/norma_sim/world/descriptor.py` (docstring only)
- Modify: `software/sim-server/scripts/probe_manifest.py` (handle `urdf_path=None`)

- [ ] **Step 1: Update `descriptor.py` docstring**

In `software/sim-server/norma_sim/world/descriptor.py`, update `build_world_descriptor`'s docstring:

```python
def build_world_descriptor(
    manifest: WorldManifest,
    world: "MuJoCoWorld | None" = None,
    publish_hz: int = 100,
    physics_hz: int = 500,
) -> "world_pb.WorldDescriptor":
    """Assemble a `WorldDescriptor` proto from a WorldManifest.

    MVP-2 notes:
    - `manifest.robots[i].sensors` is always empty (sensor enumeration
      from MJCF is deferred; see spec §2.3).
    - `manifest.urdf_path` may be None in MVP-2; this function does not
      read urdf_path, so the None case is transparent.

    `world` is accepted for future capability-derived limit validation
    but is currently unused.
    """
    del world
    ...  # rest of function body unchanged
```

No functional changes — MVP-1's list comprehension already handles empty sensor tuples.

- [ ] **Step 2: Verify descriptor.py imports cleanly**

```bash
PYTHONPATH=software/sim-server python3 -c "
from norma_sim.world.descriptor import build_world_descriptor
from norma_sim.world.manifest import WorldManifest, SceneConfig, RobotManifest
m = WorldManifest(
    world_name='test',
    scene=SceneConfig(timestep=0.002, gravity=(0,0,-9.81), integrator='RK4', solver='Newton', iterations=50),
    robots=(RobotManifest(robot_id='r', actuators=(), sensors=()),),
    mjcf_path=__import__('pathlib').Path('/tmp'),
    urdf_path=None,
)
d = build_world_descriptor(m)
print(f'descriptor.world_name={d.world_name}')
print('OK')
"
```

Expected: `descriptor.world_name=test` then `OK`.

- [ ] **Step 3: Update `probe_manifest.py` — remove `verify_source_hash` usage + handle None urdf_path**

`probe_manifest.py` still imports `verify_source_hash` (line 18 in MVP-1 baseline) and has a `--no-verify-hash` argparse flag + a verify block (lines 24-28 and 41-49). After Task 2.2 removed `verify_source_hash` from `manifest.py`, the import is orphaned and the `--help` invocation in Step 4 would `ImportError` before argparse runs.

Make 4 edits:

1. **Remove `verify_source_hash` from the import at line 18**:

```python
# before:
from norma_sim.world.manifest import load_manifest, verify_source_hash
# after:
from norma_sim.world.manifest import load_manifest
```

2. **Remove the `--no-verify-hash` argparse option** (MVP-1 baseline lines 24-28):

```python
# delete:
ap.add_argument(
    "--no-verify-hash",
    action="store_true",
    help="Skip the sha256 check (e.g. when testing a gen.py in progress)",
)
```

3. **Delete the entire hash-verification block** (MVP-1 baseline lines 41-49):

```python
# delete:
if not args.no_verify_hash:
    try:
        verify_source_hash(args.manifest, manifest.mjcf_path)
        hash_line = f"source_hash OK ({manifest.mjcf_path.name})"
    except Exception as e:
        print(f"ERROR: source_hash: {e}", file=sys.stderr)
        return 2
else:
    hash_line = "source_hash verification: SKIPPED"
```

Also delete the corresponding `print(hash_line)` line in the output section.

4. **Handle `None` urdf_path** in the `print(f"urdf_path: ...")` line (MVP-1 baseline line 54):

```python
# before:
print(f"urdf_path:       {manifest.urdf_path}")
# after:
print(f"urdf_path:       {manifest.urdf_path if manifest.urdf_path else '(not used in MVP-2)'}")
```

5. **Update the module docstring** (MVP-1 baseline lines 1-5 mention "source_hash against the inputs"). Remove the source_hash reference; something like "Loads a world manifest, builds a WorldDescriptor, and prints a readable summary" is enough.

Verify the cleanup:

```bash
grep -n 'verify_source_hash\|source_hash\|--no-verify-hash' software/sim-server/scripts/probe_manifest.py
```

Expected: no matches.

- [ ] **Step 4: Smoke-test `probe_manifest.py` doesn't crash**

```bash
PYTHONPATH=software/sim-server python3 software/sim-server/scripts/probe_manifest.py --help 2>&1 | head -5
```

Expected: prints help text, exit code 0.

(We don't run it against a real scene yaml yet because conftest fixtures are still MVP-1.)

- [ ] **Step 5: Commit**

```bash
git add software/sim-server/norma_sim/world/descriptor.py \
        software/sim-server/scripts/probe_manifest.py
git commit -m "norma_sim: descriptor + probe_manifest handle MVP-2 optional urdf_path"
```

---

### Task 2.7: Chunk 2 gate — verify new tests green, source files consistent

**Files:** None (verification)

- [ ] **Step 1: Run new Chunk 2 unit tests**

```bash
PYTHONPATH=software/sim-server python3 -m pytest \
    software/sim-server/tests/world/test_manifest_enumerate.py \
    software/sim-server/tests/world/test_manifest_new_schema.py -v
```

Expected: all 9 tests pass (4 enumerate + 5 new schema).

- [ ] **Step 2: Verify zero Rust changes**

```bash
cargo test -p sim-runtime -p st3215-wire -p st3215-compat-bridge 2>&1 | grep 'test result' | head
```

Expected: identical counts to Task 2.1 Step 2 baseline (28 + 15 + 16).

- [ ] **Step 3: Architecture invariants still hold**

```bash
make check-arch-invariants
```

Expected: `All architecture invariants hold ✓`.

- [ ] **Step 4: `urdf_joint` field rename is complete**

```bash
grep -rn 'urdf_joint' software/sim-server/norma_sim/world/
```

Expected: no matches.

- [ ] **Step 5: Confirm `source_hash` is gone**

```bash
grep -rn 'source_hash\|verify_source_hash\|hashlib' software/sim-server/norma_sim/world/ software/sim-server/tests/world/
```

Expected: no matches.

- [ ] **Step 6: Confirm red window exists (pre-existing tests ARE broken — this is expected)**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_manifest_load.py software/sim-server/tests/world/test_model.py -v 2>&1 | tail -15
```

Expected: several failures/errors about `urdf_source` / `world_yaml_path` / `urdf_joint`. **This is by design.** Chunk 3 fixes them all.

Record which tests are red — Chunk 3 gate (Task 3.10) must turn all of them green or skipped.

- [ ] **Step 7: Chunk 2 completion summary**

At the end of Chunk 2:

1. ✅ `source_hash` removed from `manifest.py`, `model.py`, and `test_manifest_load.py`
2. ✅ `urdf_joint` field renamed to `mjcf_joint` throughout `norma_sim.world.*`
3. ✅ `_enumerate_mjcf_actuators(mjcf_path)` returns `(name, joint, type_tag)` tuples
4. ✅ New `load_manifest` parses `.scene.yaml` schema against Menagerie MJCF
5. ✅ `MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)` works end-to-end
6. ✅ 9 new unit tests green (4 enumerate + 5 new schema)
7. ✅ Rust test counts unchanged (zero Rust touched)
8. ✅ Architecture invariants still hold
9. 🔴 (Expected) pre-existing MVP-1 tests are red — Chunk 3 fixes

**Do NOT proceed to Chunk 3 if any of 1-8 fail.** Item 9 is expected state.

---

**Next:** Chunk 3 migrates the MVP-1 test files to the new fixtures, closing the red window.

---

## Chunk 3: `norma_sim` test-fixture migration (closes the red window)

**Purpose:** Update `conftest.py`, then migrate every MVP-1 test file to use the new `menagerie_scene_yaml` / `elrobot_scene_yaml` fixtures. Any test that asserts ElRobot-specific shape (nu=8, specific joint names, gripper mimic multipliers) is routed to the `elrobot_scene_yaml` fixture, which `pytest.skip`s until Chunk 5 lands the hand-written ElRobot MJCF. Robot-agnostic tests use `menagerie_scene_yaml` for immediate coverage.

**Gate:** `make sim-test` passes with zero failures. Green pass count + ElRobot-specific skipped count add up to at least the MVP-1 baseline of 58 (some Menagerie-flavored tests are new, some ElRobot-flavored skip). `test_mimic_gripper.py` P0 tests show SKIPPED with message about ElRobot MJCF missing. `make check-arch-invariants` still holds. Rust test counts unchanged.

**Prerequisites:** Chunk 2 complete (new schema loader + rename in place).

**Files touched by Chunk 3:**
- Modify: `software/sim-server/tests/conftest.py`
- Modify: `software/sim-server/tests/world/test_model.py`
- Modify: `software/sim-server/tests/world/test_snapshot.py`
- Modify: `software/sim-server/tests/world/test_actuation.py`
- Modify: `software/sim-server/tests/world/test_mimic_gripper.py`
- Modify: `software/sim-server/tests/world/test_capabilities.py`
- Modify: `software/sim-server/tests/world/test_manifest_load.py` (3 remaining tests)
- Modify: `software/sim-server/tests/world/test_descriptor_build.py`
- Modify: `software/sim-server/tests/integration/test_full_loop.py`
- Modify: `software/sim-server/norma_sim/cli.py` (docstring + help text only)

---

### Task 3.1: Rewrite `conftest.py` with MVP-2 fixtures

**Files:**
- Modify: `software/sim-server/tests/conftest.py`

- [ ] **Step 1: Replace conftest.py contents**

Replace the entire contents of `software/sim-server/tests/conftest.py` with:

```python
"""Shared pytest fixtures for norma_sim tests (MVP-2 layout)."""
from pathlib import Path

import pytest


@pytest.fixture
def repo_root() -> Path:
    # tests/conftest.py → sim-server/ → software/ → repo root
    return Path(__file__).resolve().parents[3]


# --- Menagerie fixtures: immediately available after Chunk 1 ---

@pytest.fixture
def menagerie_mjcf_path(repo_root: Path) -> Path:
    """Path to the vendored Menagerie trs_so_arm100 scene.xml.
    Chunk 1 dependency — skipped if the vendor directory is absent."""
    p = repo_root / "hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml"
    if not p.exists():
        pytest.skip(f"Menagerie vendor not found at {p}; run Chunk 1 first")
    return p


@pytest.fixture
def menagerie_scene_yaml(tmp_path: Path, menagerie_mjcf_path: Path) -> Path:
    """Minimal scene.yaml pointing at the Menagerie MJCF, generated in tmp_path.
    Tests needing annotations should write their own yaml instead."""
    scene_yaml = tmp_path / "menagerie.scene.yaml"
    scene_yaml.write_text(
        f"world_name: menagerie_test\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
    )
    return scene_yaml


# --- ElRobot fixtures: skipped until Chunk 5 lands the hand-written MJCF ---

@pytest.fixture
def elrobot_mjcf_path(repo_root: Path) -> Path:
    """Path to the hand-written ElRobot MJCF (Chunk 5 artifact).
    Skipped during Chunks 2-4."""
    p = repo_root / "hardware/elrobot/simulation/elrobot_follower.xml"
    if not p.exists():
        pytest.skip(f"ElRobot MJCF not found at {p}; run Chunk 5 first")
    return p


@pytest.fixture
def elrobot_scene_yaml(repo_root: Path) -> Path:
    """Path to the hand-written ElRobot scene.yaml (Chunk 5 artifact).
    Skipped during Chunks 2-4."""
    p = repo_root / "hardware/elrobot/simulation/elrobot_follower.scene.yaml"
    if not p.exists():
        pytest.skip(f"ElRobot scene.yaml not found at {p}; run Chunk 5 first")
    return p
```

- [ ] **Step 2: Verify conftest loads without errors**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/conftest.py --collect-only -q 2>&1 | head -5
```

Expected: no errors.

- [ ] **Step 3: Commit**

```bash
git add software/sim-server/tests/conftest.py
git commit -m "norma_sim/tests: rewrite conftest.py for MVP-2 fixtures"
```

---

### Task 3.2: Migrate `test_model.py` — split ElRobot-strict vs Menagerie-loose

**Files:**
- Modify: `software/sim-server/tests/world/test_model.py`

**Rationale:** `test_model.py` currently has 5 tests using `world_yaml_path` fixture. All 5 have ElRobot-specific assertions (nu=8, neq=2, ntendon=2, specific `act_motor_01..08` and `rev_motor_01..08` names). The migration: rename each test to `*_elrobot` variant using `elrobot_scene_yaml` (skipped for now), and add a Menagerie-loose equivalent using `menagerie_scene_yaml` for immediate coverage.

- [ ] **Step 1: Read the current test_model.py to confirm test names**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_model.py --collect-only -q
```

Expected: 5 tests:
- `test_mujoco_world_loads_chunk1_mjcf`
- `test_mujoco_world_actuator_lookups`
- `test_mujoco_world_joint_qposadr_lookups`
- `test_mujoco_world_step_advances_time`
- `test_mujoco_world_actuator_by_mjcf_name`

- [ ] **Step 2: Rewrite the file with split tests**

Replace the contents of `software/sim-server/tests/world/test_model.py` with:

```python
"""Tests for MuJoCoWorld wrapper. Split into ElRobot-strict (assert
specific 8-actuator shape) and Menagerie-loose (assert any valid
MuJoCoWorld) variants. The ElRobot variants skip until Chunk 5."""
from norma_sim.world.model import MuJoCoWorld


# --- ElRobot-strict tests (skip until Chunk 5) --------------------

def test_mujoco_world_loads_elrobot_mjcf(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    assert world.model.nu == 8
    assert world.model.neq == 2
    assert world.model.ntendon == 2


def test_mujoco_world_elrobot_actuator_lookups(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    for i in range(1, 9):
        name = f"act_motor_{i:02d}"
        idx = world.actuator_id_for(name)
        assert idx is not None, f"{name} missing from cache"
        assert 0 <= idx < world.model.nu


def test_mujoco_world_elrobot_joint_qposadr_lookups(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    for i in range(1, 9):
        name = f"rev_motor_{i:02d}"
        addr = world.joint_qposadr_for(name)
        assert addr is not None, f"qposadr for {name} missing"


def test_mujoco_world_elrobot_actuator_by_mjcf_name(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    gripper = world.actuator_by_mjcf_name("act_motor_08")
    assert gripper is not None
    assert gripper.capability.kind == "GRIPPER_PARALLEL"
    assert gripper.gripper is not None
    assert world.actuator_by_mjcf_name("nonexistent") is None


# --- Menagerie-loose tests (run immediately) ----------------------

def test_mujoco_world_loads_menagerie_mjcf(menagerie_scene_yaml):
    """Menagerie trs_so_arm100 should load as a valid MuJoCoWorld with
    at least 5 actuators (SO-ARM100 class). No gripper assertion —
    Menagerie's gripper may or may not be mimic-based."""
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    assert world.model.nu >= 5


def test_mujoco_world_menagerie_actuator_lookups(menagerie_scene_yaml):
    """Every auto-synthesized revolute actuator should resolve to a
    valid cache entry."""
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    for robot in world.manifest.robots:
        for act in robot.actuators:
            idx = world.actuator_id_for(act.mjcf_actuator)
            assert idx is not None
            addr = world.joint_qposadr_for(act.mjcf_joint)
            assert addr is not None


def test_mujoco_world_step_advances_time(menagerie_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    t0 = float(world.data.time)
    for _ in range(10):
        world.step()
    t1 = float(world.data.time)
    assert t1 > t0
```

- [ ] **Step 3: Run the file**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_model.py -v
```

Expected:
- 4 tests SKIPPED (ElRobot variants)
- 3 tests PASSED (Menagerie variants)

- [ ] **Step 4: Commit**

```bash
git add software/sim-server/tests/world/test_model.py
git commit -m "norma_sim/tests: split test_model.py ElRobot vs Menagerie variants"
```

---

### Task 3.3: Migrate `test_snapshot.py`

**Files:**
- Modify: `software/sim-server/tests/world/test_snapshot.py`

**Rationale:** Current `test_snapshot.py` has 4 tests using `world_yaml_path`:
- `test_snapshot_initial_state` — asserts 8 actuators with `rev_motor_*` names (ElRobot-strict)
- `test_snapshot_tracks_ctrl_goal` — drives `rev_motor_01` specifically (ElRobot-strict)
- `test_snapshot_gripper_reports_normalized` — gripper-specific at `act_motor_08` (ElRobot-strict)
- `test_snapshot_with_clock` — generic clock check (Menagerie-OK)

- [ ] **Step 1: Rewrite test_snapshot.py**

Apply the same split pattern as Task 3.2. The 3 ElRobot-specific tests use `elrobot_scene_yaml` (skipped). Add one Menagerie generic test for `test_snapshot_with_clock`.

Concrete rewrite:

```python
"""Tests for SnapshotBuilder."""
import pytest

try:
    from norma_sim.world._proto import world_pb  # noqa: F401
    from norma_sim.world.actuation import ActuationApplier
    from norma_sim.world.model import MuJoCoWorld
    from norma_sim.world.snapshot import SnapshotBuilder
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"proto not importable: {_ERR}")


# --- ElRobot-strict (skipped until Chunk 5) -----------------------

def test_snapshot_initial_state_elrobot(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    builder = SnapshotBuilder(world)
    snap = builder.build(clock=None)
    assert len(snap.actuators) == 8
    ids = sorted(a.ref.actuator_id for a in snap.actuators)
    assert ids == [f"rev_motor_{i:02d}" for i in range(1, 9)]
    for a in snap.actuators:
        assert a.ref.robot_id == "elrobot_follower"


def test_snapshot_tracks_ctrl_goal_elrobot(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    applier = ActuationApplier(world)
    applier.drain_and_apply(
        world_pb.ActuationBatch(
            commands=[
                world_pb.ActuationCommand(
                    ref=world_pb.ActuatorRef(
                        robot_id="elrobot_follower",
                        actuator_id="rev_motor_01",
                    ),
                    set_position=world_pb.SetPosition(value=0.7, max_velocity=0.0),
                ),
            ],
        )
    )
    snap = SnapshotBuilder(world).build(clock=None)
    rev1 = next(a for a in snap.actuators if a.ref.actuator_id == "rev_motor_01")
    assert rev1.goal_position_value == pytest.approx(0.7, abs=1e-9)


def test_snapshot_gripper_reports_normalized_elrobot(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    idx = world.actuator_id_for("act_motor_08")
    world.data.ctrl[idx] = 2.2028 / 2  # joint midpoint
    snap = SnapshotBuilder(world).build(clock=None)
    g = next(a for a in snap.actuators if a.ref.actuator_id == "rev_motor_08")
    assert g.goal_position_value == pytest.approx(0.5, abs=1e-6)


# --- Menagerie generic ---------------------------------------------

def test_snapshot_with_clock(menagerie_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    clock = world_pb.WorldClock(world_tick=42, sim_time_ns=84_000_000, wall_time_ns=0)
    snap = SnapshotBuilder(world).build(clock=clock)
    assert snap.clock is not None
    assert snap.clock.world_tick == 42


def test_snapshot_menagerie_initial_state_loads(menagerie_scene_yaml):
    """Menagerie snapshot should produce an actuator list (any shape)."""
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    builder = SnapshotBuilder(world)
    snap = builder.build(clock=None)
    assert len(snap.actuators) >= 5
```

- [ ] **Step 2: Run the file**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_snapshot.py -v
```

Expected: 3 SKIPPED (ElRobot), 2 PASSED (Menagerie).

- [ ] **Step 3: Commit**

```bash
git add software/sim-server/tests/world/test_snapshot.py
git commit -m "norma_sim/tests: migrate test_snapshot.py to MVP-2 fixtures"
```

---

### Task 3.4: Migrate `test_actuation.py`

**Files:**
- Modify: `software/sim-server/tests/world/test_actuation.py`

**Rationale:** `test_actuation.py` has 5 MVP-1 tests. Four assert ElRobot-specific shape (`rev_motor_01`/`rev_motor_02`/`rev_motor_08` + `act_motor_01`/`act_motor_02`/`act_motor_08`). One is arguably robot-agnostic (`test_apply_unknown_actuator_increments_counter` uses the bogus name `rev_motor_99` and only asserts the counter). Route the 4 ElRobot-strict tests to `elrobot_scene_yaml` (skipped until Chunk 5) and add a parallel Menagerie variant for the "unknown actuator counter" test to keep that coverage alive during Chunks 2-4.

- [ ] **Step 1: Rewrite `test_actuation.py` with split variants**

Replace the entire contents of `software/sim-server/tests/world/test_actuation.py` with:

```python
"""Tests for ActuationApplier: proto batch → MjData.ctrl writes.

Split into ElRobot-strict variants (assert rev_motor_01..08 shape,
skipped until Chunk 5) and a Menagerie-loose variant covering the
robot-agnostic "unknown actuator" counter path."""
import pytest

try:
    from norma_sim.world._proto import world_pb  # noqa: F401
    from norma_sim.world.actuation import ActuationApplier
    from norma_sim.world.manifest import DEFAULT_ROBOT_ID
    from norma_sim.world.model import MuJoCoWorld
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"proto not importable: {_ERR}")


# --- ElRobot helpers ---

def _ref_elrobot(actuator_id: str) -> "world_pb.ActuatorRef":
    return world_pb.ActuatorRef(robot_id="elrobot_follower", actuator_id=actuator_id)


# --- ElRobot-strict tests (skipped until Chunk 5) ---

def test_apply_set_position_revolute_elrobot(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        as_of=None,
        commands=[
            world_pb.ActuationCommand(
                ref=_ref_elrobot("rev_motor_01"),
                set_position=world_pb.SetPosition(value=0.5, max_velocity=0.0),
            ),
        ],
        lane=world_pb.QosLane.QOS_LOSSY_SETPOINT,
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 1
    assert stats.unknown_actuator == 0
    idx = world.actuator_id_for("act_motor_01")
    assert world.data.ctrl[idx] == pytest.approx(0.5, abs=1e-9)


def test_apply_set_position_gripper_normalized_elrobot(elrobot_scene_yaml):
    """Gripper ctrl should receive the de-normalized rad value.
    ElRobot-specific: the 2.2028 rad value is ElRobot's primary joint range."""
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        commands=[
            world_pb.ActuationCommand(
                ref=_ref_elrobot("rev_motor_08"),
                set_position=world_pb.SetPosition(value=0.5, max_velocity=0.0),
            ),
        ],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 1
    idx = world.actuator_id_for("act_motor_08")
    assert world.data.ctrl[idx] == pytest.approx(2.2028 / 2, abs=1e-6)


def test_apply_command_without_intent_counts_unsupported_elrobot(elrobot_scene_yaml):
    """Uses rev_motor_01 ref but only asserts the unsupported_intent counter."""
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        commands=[world_pb.ActuationCommand(ref=_ref_elrobot("rev_motor_01"))],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 0
    assert stats.unsupported_intent == 1


def test_apply_multi_command_batch_elrobot(elrobot_scene_yaml):
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        commands=[
            world_pb.ActuationCommand(
                ref=_ref_elrobot("rev_motor_01"),
                set_position=world_pb.SetPosition(value=0.3, max_velocity=0.0),
            ),
            world_pb.ActuationCommand(
                ref=_ref_elrobot("rev_motor_02"),
                set_position=world_pb.SetPosition(value=-0.2, max_velocity=0.0),
            ),
        ],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 2
    assert world.data.ctrl[world.actuator_id_for("act_motor_01")] == pytest.approx(0.3)
    assert world.data.ctrl[world.actuator_id_for("act_motor_02")] == pytest.approx(-0.2)


# --- Menagerie-loose tests (run immediately) ---

def test_apply_unknown_actuator_increments_counter_menagerie(menagerie_scene_yaml):
    """Robot-agnostic: send a command with a nonexistent actuator_id
    and verify the counter increments. Works against any MJCF because
    the target name is chosen specifically to NOT exist."""
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    applier = ActuationApplier(world)
    batch = world_pb.ActuationBatch(
        commands=[
            world_pb.ActuationCommand(
                ref=world_pb.ActuatorRef(
                    robot_id=DEFAULT_ROBOT_ID,
                    actuator_id="definitely_not_an_actuator_name_xyz",
                ),
                set_position=world_pb.SetPosition(value=0.0, max_velocity=0.0),
            ),
        ],
    )
    stats = applier.drain_and_apply(batch)
    assert stats.applied == 0
    assert stats.unknown_actuator == 1


def test_apply_set_position_revolute_menagerie(menagerie_scene_yaml):
    """Robot-agnostic: drive the FIRST REVOLUTE_POSITION actuator found
    in the MJCF to 0.1 rad and verify ctrl receives the value.
    Uses dynamic discovery so it works against any Menagerie model."""
    world = MuJoCoWorld.from_manifest_path(menagerie_scene_yaml)
    rev_acts = [
        a for r in world.manifest.robots for a in r.actuators
        if a.capability.kind == "REVOLUTE_POSITION"
    ]
    assert rev_acts, "Menagerie MJCF should have at least one REVOLUTE_POSITION"
    target = rev_acts[0]
    applier = ActuationApplier(world)
    applier.drain_and_apply(world_pb.ActuationBatch(commands=[
        world_pb.ActuationCommand(
            ref=world_pb.ActuatorRef(
                robot_id=DEFAULT_ROBOT_ID,
                actuator_id=target.actuator_id,
            ),
            set_position=world_pb.SetPosition(value=0.1, max_velocity=0.0),
        ),
    ]))
    idx = world.actuator_id_for(target.mjcf_actuator)
    assert world.data.ctrl[idx] == pytest.approx(0.1, abs=1e-9)
```

Note: `DEFAULT_ROBOT_ID` is imported from `norma_sim.world.manifest` (set in Chunk 2 Task 2.5 as the constant `"default_robot"`). This avoids hardcoding the string and drifts automatically if the default ever changes.

- [ ] **Step 2: Run the file**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_actuation.py -v
```

Expected: 4 SKIPPED (ElRobot variants), 2 PASSED (Menagerie variants).

- [ ] **Step 3: Commit**

```bash
git add software/sim-server/tests/world/test_actuation.py
git commit -m "norma_sim/tests: split test_actuation.py into ElRobot + Menagerie"
```

---

### Task 3.5: Migrate `test_mimic_gripper.py` — P0 stays ElRobot-strict (skipped)

**Files:**
- Modify: `software/sim-server/tests/world/test_mimic_gripper.py`

**Rationale:** P0 gripper mimic tests assert the specific multiplier `-0.0115` / `+0.0115` which is hardcoded to ElRobot's gripper. These tests are **ElRobot-specific by design** and will `pytest.skip` during Chunks 2-4, then resume at Chunk 5. There is no Menagerie analog — Menagerie's gripper may use different construction.

- [ ] **Step 1: Rename fixture references**

In `software/sim-server/tests/world/test_mimic_gripper.py`, change:

```python
def test_mimic_gripper_equality_works(mjcf_path):
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    ...
```

to:

```python
def test_mimic_gripper_equality_works(elrobot_mjcf_path):
    model = mujoco.MjModel.from_xml_path(str(elrobot_mjcf_path))
    ...
```

Apply the same rename to `test_mimic_gripper_zero_setpoint_holds_zero`.

- [ ] **Step 2: Run the file**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_mimic_gripper.py -v
```

Expected: 2 SKIPPED with message "ElRobot MJCF not found...; run Chunk 5 first".

- [ ] **Step 3: Commit**

```bash
git add software/sim-server/tests/world/test_mimic_gripper.py
git commit -m "norma_sim/tests: P0 gripper mimic tests use elrobot_mjcf_path"
```

---

### Task 3.6: Migrate `test_capabilities.py`

**Files:**
- Modify: `software/sim-server/tests/world/test_capabilities.py`

**Rationale:** `test_capabilities.py` may construct `ActuatorManifest` inline with keyword arguments. The `urdf_joint` → `mjcf_joint` rename in Chunk 2 Task 2.3 breaks any such construction. Migration: find + rename the keyword argument.

- [ ] **Step 1: Find `urdf_joint` references**

```bash
grep -n 'urdf_joint' software/sim-server/tests/world/test_capabilities.py
```

Expected: 2 sites (approximately lines 25 and 36 per reviewer notes).

- [ ] **Step 2: Rename each to `mjcf_joint`**

For each construction site like:

```python
ActuatorManifest(
    actuator_id="rev_motor_01",
    display_name="Shoulder Pitch",
    urdf_joint="rev_motor_01",  # ← rename
    mjcf_actuator="act_motor_01",
    ...
)
```

change to:

```python
ActuatorManifest(
    actuator_id="rev_motor_01",
    display_name="Shoulder Pitch",
    mjcf_joint="rev_motor_01",  # ← renamed
    mjcf_actuator="act_motor_01",
    ...
)
```

- [ ] **Step 3: Run the file**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_capabilities.py -v
```

Expected: all tests PASS (they construct `ActuatorManifest` inline with no fixture dependency).

- [ ] **Step 4: Commit**

```bash
git add software/sim-server/tests/world/test_capabilities.py
git commit -m "norma_sim/tests: rename urdf_joint → mjcf_joint in test_capabilities"
```

---

### Task 3.7: Rewrite the 3 remaining `test_manifest_load.py` tests

**Files:**
- Modify: `software/sim-server/tests/world/test_manifest_load.py`

**Rationale:** Chunk 2 Task 2.2 deleted the 3 `test_source_hash_*` tests. The 3 remaining tests (`test_manifest_load_happy`, `test_manifest_scene_config`, `test_manifest_missing_gripper_fields_raises`) still use the MVP-1 schema. Rewrite them for the new schema using the `menagerie_mjcf_path` fixture.

- [ ] **Step 1: Replace the 3 tests**

Open `software/sim-server/tests/world/test_manifest_load.py`. Replace the 3 remaining tests with:

```python
def test_manifest_load_happy(tmp_path, menagerie_mjcf_path):
    scene_yaml = tmp_path / "test.scene.yaml"
    scene_yaml.write_text(
        f"world_name: happy_world\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
    )
    manifest = load_manifest(scene_yaml)
    assert manifest.world_name == "happy_world"
    assert manifest.mjcf_path == menagerie_mjcf_path.resolve()
    assert len(manifest.robots) == 1
    assert len(manifest.robots[0].actuators) >= 5


def test_manifest_scene_config(tmp_path, menagerie_mjcf_path):
    scene_yaml = tmp_path / "test.scene.yaml"
    scene_yaml.write_text(
        f"world_name: test\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
        f"scene_overrides:\n"
        f"  timestep: 0.001\n"
        f"  gravity: [0, 0, -5]\n"
        f"  iterations: 100\n"
    )
    manifest = load_manifest(scene_yaml)
    assert manifest.scene.timestep == 0.001
    assert manifest.scene.gravity == (0, 0, -5)
    assert manifest.scene.iterations == 100


def test_manifest_missing_gripper_fields_raises(tmp_path, menagerie_mjcf_path):
    """GRIPPER_PARALLEL annotation missing the 'gripper:' block raises."""
    from norma_sim.world.manifest import _enumerate_mjcf_actuators
    actuators = _enumerate_mjcf_actuators(menagerie_mjcf_path)
    target_name = actuators[0][0]

    scene_yaml = tmp_path / "bad.scene.yaml"
    scene_yaml.write_text(
        f"world_name: test\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
        f"actuator_annotations:\n"
        f"  - mjcf_actuator: {target_name}\n"
        f"    actuator_id: bad_gripper\n"
        f"    display_name: Bad\n"
        f"    capability:\n"
        f"      kind: GRIPPER_PARALLEL\n"
        f"      normalized_range: [0.0, 1.0]\n"
        f"    # missing: gripper: block\n"
    )
    with pytest.raises(ValueError, match="gripper"):
        load_manifest(scene_yaml)
```

- [ ] **Step 2: Run the file**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_manifest_load.py -v
```

Expected: all 3 tests pass.

- [ ] **Step 3: Commit**

```bash
git add software/sim-server/tests/world/test_manifest_load.py
git commit -m "norma_sim/tests: rewrite 3 remaining test_manifest_load.py for MVP-2"
```

---

### Task 3.8: Migrate `test_descriptor_build.py`

**Files:**
- Modify: `software/sim-server/tests/world/test_descriptor_build.py`

**Rationale:** The MVP-1 file has 4 tests, all ElRobot-specific. Each needs explicit handling:

1. `test_build_world_descriptor_happy` — asserts world_name `"elrobot_follower_empty"`, `len(actuators) == 8`, `robot_id == "elrobot_follower"`. **Split into ElRobot-strict (skipped) + Menagerie-loose (runs now).**
2. `test_build_world_descriptor_actuator_kinds` — asserts first 7 actuators are REVOLUTE, M8 is GRIPPER with `effort_limit == 2.94`, `velocity_limit == 4.71`. **ElRobot-strict only; no Menagerie analog** (Menagerie's gripper may differ).
3. `test_build_world_descriptor_sensors` — asserts `len(robot.sensors) == 1` with `sensor_id == "joint_state_all"`. **DELETE outright**: Chunk 2 Task 2.5's new `load_manifest` hard-codes `sensors=()` (see spec §2.3: sensor enumeration is deferred). No replacement test is needed because the deferred status is documented in spec.
4. `test_build_world_descriptor_encodes` — sanity round-trip via `desc.encode()`. **Split into ElRobot-strict + Menagerie-loose.**

**Note on proto API**: sim-server uses gremlin-py dataclasses with a `.encode() -> bytes` method (see MVP-1 `test_descriptor_build.py:75-77`). Do NOT use `SerializeToString()` / `ParseFromString()` — those are google.protobuf APIs and will raise `AttributeError` on gremlin classes.

- [ ] **Step 1: Rewrite `test_descriptor_build.py`**

Replace the entire contents of `software/sim-server/tests/world/test_descriptor_build.py` with:

```python
"""Tests for build_world_descriptor: manifest → proto mapping.

Skip-gate: if the gremlin-generated proto module isn't importable
(e.g. `make protobuf` hasn't been run), the whole file is skipped
with an actionable reason.

Split into ElRobot-strict (assert 8 actuators, specific names,
M8 gripper limits) and Menagerie-loose (assert any valid descriptor
encodes). The sensors test is deleted because MVP-2 defers sensor
enumeration (spec §2.3)."""
import pytest

try:
    from norma_sim.world._proto import world_pb  # noqa: F401
    from norma_sim.world.descriptor import build_world_descriptor
    from norma_sim.world.manifest import load_manifest
    _PROTO_OK = True
    _PROTO_ERR = ""
except Exception as e:  # pragma: no cover
    _PROTO_OK = False
    _PROTO_ERR = str(e)


pytestmark = pytest.mark.skipif(
    not _PROTO_OK,
    reason=f"gremlin proto not importable; run 'make protobuf' first: {_PROTO_ERR}",
)


# --- ElRobot-strict (skipped until Chunk 5) ---

def test_build_world_descriptor_happy_elrobot(elrobot_scene_yaml):
    manifest = load_manifest(elrobot_scene_yaml)
    desc = build_world_descriptor(manifest)
    assert desc.world_name == "elrobot_follower"
    assert desc.publish_hz == 100
    assert desc.physics_hz == 500
    assert len(desc.robots) == 1
    robot = desc.robots[0]
    assert robot.robot_id == "elrobot_follower"
    assert len(robot.actuators) == 8


def test_build_world_descriptor_actuator_kinds_elrobot(elrobot_scene_yaml):
    manifest = load_manifest(elrobot_scene_yaml)
    desc = build_world_descriptor(manifest)
    robot = desc.robots[0]

    for i in range(7):
        kind = robot.actuators[i].capability.kind
        assert kind == world_pb.ActuatorCapability_Kind.CAP_REVOLUTE_POSITION, (
            f"actuator {i} unexpected kind {kind}"
        )

    m8 = robot.actuators[7]
    assert m8.capability.kind == world_pb.ActuatorCapability_Kind.CAP_GRIPPER_PARALLEL
    assert m8.capability.limit_min == 0.0
    assert m8.capability.limit_max == 1.0
    assert abs(m8.capability.effort_limit - 2.94) < 1e-9
    assert abs(m8.capability.velocity_limit - 4.71) < 1e-9


def test_build_world_descriptor_encodes_elrobot(elrobot_scene_yaml):
    """ElRobot strict: 8-actuator descriptor encodes to non-empty bytes."""
    manifest = load_manifest(elrobot_scene_yaml)
    desc = build_world_descriptor(manifest)
    buf = desc.encode()
    assert isinstance(buf, (bytes, bytearray))
    assert len(buf) > 0


# --- Menagerie-loose ---

def test_build_world_descriptor_happy_menagerie(menagerie_scene_yaml):
    """Generic: any valid MJCF produces a non-empty descriptor."""
    manifest = load_manifest(menagerie_scene_yaml)
    desc = build_world_descriptor(manifest)
    assert desc.world_name == "menagerie_test"
    assert desc.publish_hz == 100
    assert desc.physics_hz == 500
    assert len(desc.robots) == 1
    robot = desc.robots[0]
    assert len(robot.actuators) >= 5
    # default robot_id applies since Menagerie scene yaml doesn't set one
    assert robot.robot_id == "default_robot"


def test_build_world_descriptor_encodes_menagerie(menagerie_scene_yaml):
    """Generic: Menagerie descriptor encodes to non-empty bytes."""
    manifest = load_manifest(menagerie_scene_yaml)
    desc = build_world_descriptor(manifest)
    buf = desc.encode()
    assert isinstance(buf, (bytes, bytearray))
    assert len(buf) > 0


# test_build_world_descriptor_sensors DELETED:
# MVP-2 load_manifest hard-codes sensors=() because sensor enumeration
# from MJCF is deferred (spec §2.3). The "joint_state_all" sensor that
# MVP-1 gen.py placed in the yaml no longer exists in the manifest.
# No replacement test — this is an intentional scope reduction.
```

- [ ] **Step 2: Run the file**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/world/test_descriptor_build.py -v
```

Expected:
- 3 SKIPPED (ElRobot variants)
- 2 PASSED (Menagerie variants)
- 0 FAILED

- [ ] **Step 3: Commit**

```bash
git add software/sim-server/tests/world/test_descriptor_build.py
git commit -m "norma_sim/tests: rewrite test_descriptor_build.py for MVP-2 schema"
```

Commit body:

```
Split into ElRobot-strict (3 tests, skipped until Chunk 5) and
Menagerie-loose (2 tests, pass immediately) variants. The MVP-1
test_build_world_descriptor_sensors is DELETED because MVP-2's
load_manifest sets sensors=() per spec §2.3 (sensor enumeration
deferred).
```

---

### Task 3.9: Migrate `test_full_loop.py` integration test

**Files:**
- Modify: `software/sim-server/tests/integration/test_full_loop.py`

**Rationale:** The file has 3 integration tests. Concrete routing:

1. `test_full_loop` — asserts `welcome.welcome.world.world_name == "elrobot_follower_empty"` (line 119) and uses `robot_id="elrobot_follower"` + `actuator_id="rev_motor_01"` (lines 131-132). **ElRobot-strict**: route to `elrobot_scene_yaml`, the test will pytest.skip until Chunk 5 lands the ElRobot MJCF. Also update the expected `world_name` from `"elrobot_follower_empty"` → `"elrobot_follower"` to match Chunk 5's new scene yaml.
2. `test_multi_client_fan_out` — purely tests that 2 clients both receive snapshots; no actuator-id / robot-id assertions. **Robot-agnostic**: route to `menagerie_scene_yaml`.
3. `test_subprocess_clean_shutdown` — purely tests SIGTERM + exit cleanup. **Robot-agnostic**: route to `menagerie_scene_yaml`.

Also update the `_spawn_sim(socket_path, world_yaml_path)` helper signature: rename the `world_yaml_path` parameter to `scene_yaml_path` since it's no longer a `world.yaml`. The spawned CLI still uses `--manifest` flag name (Task 3.10 keeps the flag name, only updates help text), so the `--manifest` subprocess arg is unchanged.

- [ ] **Step 1: Rewrite `_spawn_sim` helper signature**

In `software/sim-server/tests/integration/test_full_loop.py`, change the helper signature (MVP-1 baseline line 51):

```python
# before:
async def _spawn_sim(socket_path: Path, world_yaml_path: Path) -> asyncio.subprocess.Process:
    ...
    return await asyncio.create_subprocess_exec(
        sys.executable, "-m", "norma_sim",
        "--manifest", str(world_yaml_path),
        ...
    )

# after:
async def _spawn_sim(socket_path: Path, scene_yaml_path: Path) -> asyncio.subprocess.Process:
    ...
    return await asyncio.create_subprocess_exec(
        sys.executable, "-m", "norma_sim",
        "--manifest", str(scene_yaml_path),
        ...
    )
```

The `--manifest` CLI flag name is preserved — only the Python parameter name changes.

- [ ] **Step 2: Update `test_full_loop` (ElRobot-strict)**

Change the fixture from `world_yaml_path` to `elrobot_scene_yaml` and update the world_name assertion:

```python
def test_full_loop(elrobot_scene_yaml, tmp_path):
    """Launch norma_sim as a subprocess; handshake; send an
    actuation; receive at least one snapshot; terminate cleanly.
    ElRobot-strict: uses robot_id='elrobot_follower' and
    actuator_id='rev_motor_01' which are ElRobot-specific."""

    async def _inner():
        socket_path = tmp_path / "sim.sock"
        proc = await _spawn_sim(socket_path, elrobot_scene_yaml)
        try:
            assert await _wait_for_socket(socket_path, timeout=5.0), (
                "sim server did not bind socket within 5s; "
                f"stderr={(await proc.stderr.read()) if proc.stderr else b''!r}"
            )

            reader, writer = await asyncio.open_unix_connection(str(socket_path))
            welcome = await _handshake(reader, writer, "full-loop", "c1")
            assert welcome.welcome is not None
            assert welcome.welcome.world is not None
            assert welcome.welcome.world.world_name == "elrobot_follower"
            #  ^^^ updated from "elrobot_follower_empty" (MVP-1) to match
            #      Chunk 5's new scene yaml world_name

            # rest of the function body unchanged; still uses
            # robot_id="elrobot_follower" + actuator_id="rev_motor_01"
            ...
        finally:
            proc.terminate()
            ...

    _run(_inner())
```

Leave the actuation + snapshot assertion body unchanged (it's ElRobot-specific which is fine since the test now skips).

- [ ] **Step 3: Update `test_multi_client_fan_out` (Menagerie-loose)**

Change the fixture parameter from `world_yaml_path` to `menagerie_scene_yaml`. No other changes needed because this test only checks that both connected clients receive snapshots — no actuator-id assertions.

- [ ] **Step 4: Update `test_subprocess_clean_shutdown` (Menagerie-loose)**

Change the fixture parameter from `world_yaml_path` to `menagerie_scene_yaml`. Like test_multi_client_fan_out, no other changes are needed.

- [ ] **Step 5: Run the file**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/integration/test_full_loop.py -v
```

Expected: 1 SKIPPED (`test_full_loop`), 2 PASSED (`test_multi_client_fan_out`, `test_subprocess_clean_shutdown`), 0 FAILED.

If the Menagerie tests fail with a subprocess spawn error, it's likely because `cli.py`'s `verify_source_hash` import (if Chunk 2 Task 2.2 Step 2b was skipped) is still broken. Re-check Task 2.2.

- [ ] **Step 6: Commit**

```bash
git add software/sim-server/tests/integration/test_full_loop.py
git commit -m "norma_sim/tests: route test_full_loop.py tests to MVP-2 fixtures"
```

---

### Task 3.10: Update `cli.py` help text for new schema (flag name unchanged)

**Files:**
- Modify: `software/sim-server/norma_sim/cli.py` (help text + module docstring only)

**Important:** The CLI flag name `--manifest` is **kept** for backward compatibility with `test_full_loop.py`'s `_spawn_sim` helper and any other call sites. Only the **help text** and **module docstring** change. Do NOT rename the flag to `--config`.

- [ ] **Step 1: Find the current `--manifest` argument definition + module docstring**

```bash
grep -n '"""\|--manifest\|add_argument' software/sim-server/norma_sim/cli.py
```

Expected: module docstring at line 1, `ap.add_argument("--manifest", ...)` around line 39-43.

- [ ] **Step 2: Update module docstring**

The MVP-1 module docstring at line 1 says something like `"""python -m norma_sim --manifest <path> ..."""`. Update it to reference the new schema:

```python
"""`python -m norma_sim --manifest <path>` entry point.

The `--manifest` flag accepts the MVP-2 `.scene.yaml` schema
(see docs/superpowers/specs/2026-04-11-mvp2-menagerie-walking-skeleton-design.md
section 8.1). MVP-1's `.world.yaml` schema is no longer supported.
"""
```

- [ ] **Step 3: Update `--manifest` help text**

In the `ap.add_argument("--manifest", ...)` block, update only the `help=` string:

```python
ap.add_argument(
    "--manifest",
    type=Path,
    required=True,
    help=(
        "Path to the sim scene config yaml (MVP-2 .scene.yaml schema; "
        "see spec 2026-04-11-mvp2-menagerie-walking-skeleton-design.md "
        "section 8.1)."
    ),
)
```

Do **not** change the flag name, `type`, or `required` — only the `help=` string.

- [ ] **Step 4: Smoke-test help output**

```bash
PYTHONPATH=software/sim-server python3 -m norma_sim --help 2>&1 | grep -i 'scene\|manifest'
```

Expected:
- help text shows `--manifest` (not `--config`)
- description mentions `.scene.yaml`

Also verify the subprocess spawn still works with the flag unchanged:

```bash
PYTHONPATH=software/sim-server python3 -m norma_sim --manifest /nonexistent.yaml 2>&1 | head -3
```

Expected: argparse accepts `--manifest`, then errors on the nonexistent file (not on the flag). This confirms `test_full_loop.py`'s `_spawn_sim` helper will still work.

- [ ] **Step 5: Commit**

```bash
git add software/sim-server/norma_sim/cli.py
git commit -m "norma_sim/cli: update --manifest help text for MVP-2 scene.yaml"
```

---

### Task 3.11: Chunk 3 gate — red window closed, full test suite green

**Files:** None (verification)

- [ ] **Step 1: Run full Python test suite**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/ -v 2>&1 | tail -40
```

Expected:
- Many tests PASSED (Menagerie flavor + non-fixture tests + new schema tests)
- Several tests SKIPPED (ElRobot flavor, waiting for Chunk 5)
- **Zero FAILED**

Record the pass + skip counts.

- [ ] **Step 2: Run `make sim-test`**

```bash
make sim-test 2>&1 | tail -15
```

Expected:
- Architecture invariants ✓
- Rust test counts match Task 2.1 Step 2 baseline (28 + 15 + 16)
- Python: pass + skip with no failures

- [ ] **Step 3: Confirm no lingering `urdf_joint` / `source_hash` references**

```bash
grep -rn 'urdf_joint\|verify_source_hash\|source_hash' \
    software/sim-server/norma_sim/ software/sim-server/tests/
```

Expected: no matches.

- [ ] **Step 4: Chunk 3 completion summary**

1. ✅ `conftest.py` rewritten with `menagerie_*` and `elrobot_*` fixtures
2. ✅ `test_model.py` split into ElRobot-strict + Menagerie-loose variants
3. ✅ `test_snapshot.py`, `test_actuation.py`, `test_mimic_gripper.py` migrated
4. ✅ `test_capabilities.py` `urdf_joint` → `mjcf_joint` rename applied
5. ✅ `test_manifest_load.py` 3 remaining tests rewritten for new schema
6. ✅ `test_descriptor_build.py` split variants
7. ✅ `test_full_loop.py` migrated
8. ✅ `cli.py` help text updated
9. ✅ `make sim-test` green (zero failures; some ElRobot-specific skips expected)
10. ✅ Architecture invariants + Rust counts unchanged

**Do NOT proceed to Chunk 4 if any of 1-10 fail.**

---

**Next:** Chunk 4 writes the Menagerie walking skeleton configs (scene yaml + bridge preset + station yaml) and the walking skeleton integration test.

---

## Chunk 4: Phase 1 — Walking skeleton configs + integration test

**Purpose:** Run Menagerie SO-ARM100 verbatim through MVP-1's full stack (station + sim-runtime + norma_sim + bridge + web UI). This proves hypothesis A ("infra is robot-agnostic"). Produces a permanent regression fixture `test_menagerie_walking_skeleton.py` that stays green after Chunks 5-7 land ElRobot.

**Gate:** `test_menagerie_walking_skeleton.py` passes. Manual browser smoke test against `station-sim-menagerie.yaml` shows Menagerie's N motors as draggable sliders. `cargo test -p sim-runtime -p st3215-wire -p st3215-compat-bridge` unchanged (zero Rust touched). `make check-arch-invariants` still passes.

**Prerequisites:** Chunks 1-3 complete.

**Files touched:**
- Create: `hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml`
- Create: `software/sim-bridges/st3215-compat-bridge/presets/menagerie-so-arm100.yaml`
- Create: `software/station/bin/station/station-sim-menagerie.yaml`
- Create: `software/sim-server/tests/integration/test_menagerie_walking_skeleton.py`
- Unchanged: all Rust source files (bridge preset is data, not code)

---

### Task 4.1: Write `menagerie_so_arm100.scene.yaml`

**Files:**
- Create: `hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml`

**Rationale:** Phase 1's scene yaml is minimal: it points at the vendored Menagerie MJCF, lets `<option>` come from MJCF directly (no overrides), and annotates the gripper only if Menagerie's gripper needs GRIPPER_PARALLEL treatment. At plan-write time we don't know Menagerie's exact gripper topology — this task includes a research step to decide.

- [ ] **Step 1: Inspect Menagerie's gripper implementation**

```bash
grep -n 'gripper\|finger\|tendon\|equality' hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/trs_so_arm100.xml
```

Record:
- Does Menagerie have an actuator named `gripper` (or similar)? What type — `<position>`, `<general>`?
- Is there a `<tendon><fixed>` + `<equality><tendon>` mimic structure like ElRobot's gripper?
- What are the joint/ctrl ranges?

If Menagerie's gripper is a plain `<position>` with no tendon (just a single prismatic finger joint), **no annotation is needed** — auto-synthesis as REVOLUTE_POSITION works fine for Phase 1 smoke. The web UI will show a slider that linearly controls the ctrl value.

If Menagerie's gripper uses tendon-based mimic like ours, decide whether to annotate it as GRIPPER_PARALLEL for Phase 1 (better UX, normalized [0,1] range) or leave as auto-synthesized REVOLUTE_POSITION (easier, just works).

**For Phase 1 purposes: prefer the simpler path (no annotation)** unless the gripper is visibly broken. Phase 1's goal is infra validation, not gripper UX.

- [ ] **Step 2: Write the scene yaml**

Create `hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml`:

```yaml
# MVP-2 Phase 1 — walking skeleton scene config for Menagerie trs_so_arm100.
#
# This yaml runs Menagerie's SO-ARM100 MJCF (vendored in Chunk 1, unmodified)
# through MVP-1's full station stack. Its purpose is to verify that our
# Rust/IPC/bridge infrastructure is robot-agnostic — it accepts any
# MuJoCo-valid MJCF with any actuator topology.
#
# This file is a PERMANENT REGRESSION FIXTURE: it stays in the repo after
# Phase 2 lands the ElRobot MJCF, as the baseline for assumption A ("infra
# is robot-agnostic"). If something in sim-runtime / norma_sim regresses
# and only ElRobot breaks, this scene yaml + test_menagerie_walking_skeleton.py
# prove the infra is still sound and the problem is ElRobot-specific.

world_name: menagerie_trs_so_arm100
mjcf_path: ./vendor/menagerie/trs_so_arm100/scene.xml
# No scene_overrides — Menagerie's <option> block is authoritative.
# No scene_extras — Menagerie's scene.xml already has lights + floor.
# No actuator_annotations — auto-synthesize all <position> actuators as
# REVOLUTE_POSITION. If Phase 1 smoke test reveals that Menagerie's
# gripper needs GRIPPER_PARALLEL semantics for usable UX, add an
# annotation here; otherwise leave minimal.
```

- [ ] **Step 3: Smoke test loader**

```bash
PYTHONPATH=software/sim-server python3 -c "
from norma_sim.world.manifest import load_manifest
from pathlib import Path
m = load_manifest(Path('hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml'))
print(f'world_name={m.world_name}')
print(f'mjcf_path={m.mjcf_path}')
print(f'n_actuators={sum(len(r.actuators) for r in m.robots)}')
for r in m.robots:
    for a in r.actuators:
        print(f'  {a.actuator_id:<30} kind={a.capability.kind}')
"
```

Expected: loads successfully, prints actuators with kind=REVOLUTE_POSITION (and potentially one GRIPPER_PARALLEL if annotated).

- [ ] **Step 4: Commit**

```bash
git add hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml
git commit -m "mvp2: add Menagerie walking skeleton scene.yaml"
```

---

### Task 4.2: Write `menagerie-so-arm100.yaml` preset for st3215-compat-bridge

**Files:**
- Create: `software/sim-bridges/st3215-compat-bridge/presets/menagerie-so-arm100.yaml`

**Rationale:** The `st3215-compat-bridge` uses a preset yaml to map `actuator_id` (from snapshots) to fake ST3215 `motor_id` (for the web UI's ST3215 slider viewer). The existing `elrobot.preset.yaml` maps `rev_motor_01..08` to motor_id 1..8. For Phase 1, we need a preset that maps Menagerie's actuator_ids to fake motor_ids so the web UI can show sliders for them.

Because Phase 1's scene yaml does NOT list actuator_annotations (auto-synthesis mode), the `actuator_id` values are derived from Menagerie's `mjcf_actuator` names (e.g., `shoulder_pan`, `shoulder_lift`, `elbow`, ...). The preset needs to know those names.

- [ ] **Step 1: Enumerate Menagerie's actuator_ids**

Use the smoke test output from Task 4.1 Step 3. Record the actuator_ids Menagerie exposes (likely something like `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`).

- [ ] **Step 2: Inspect the existing `elrobot.preset.yaml` format**

```bash
cat software/sim-bridges/st3215-compat-bridge/presets/elrobot-follower.yaml
```

Record the schema — it's the template for the Menagerie preset.

- [ ] **Step 3: Write the Menagerie preset**

Create `software/sim-bridges/st3215-compat-bridge/presets/menagerie-so-arm100.yaml` following the same schema as the existing `elrobot-follower.yaml`. Substitute Menagerie's actuator names for `rev_motor_01..08`, and assign motor_id 1..N where N is Menagerie's actuator count. Example (substitute real names from Task 4.1):

```yaml
# MVP-2 Phase 1 — ST3215 compat bridge preset for Menagerie trs_so_arm100.
# Maps Menagerie's actuator names (auto-synthesized from MJCF) to fake
# ST3215 motor_ids so the web UI's slider viewer can display them.
#
# This preset is a PERMANENT REGRESSION FIXTURE alongside
# menagerie_so_arm100.scene.yaml. Do not delete when Phase 2 ships.

robot_id: default_robot
legacy_bus_serial: "sim://menagerie-so-arm100"
motors:
  - actuator_id: shoulder_pan        # ← replace with Menagerie's actual name
    motor_id: 1
    min_angle_steps: 0
    max_angle_steps: 4095
    offset_steps: 2048
    torque_limit: 500
    voltage_nominal_v: 12.0
  - actuator_id: shoulder_lift
    motor_id: 2
    min_angle_steps: 0
    max_angle_steps: 4095
    offset_steps: 2048
    torque_limit: 500
    voltage_nominal_v: 12.0
  # ... repeat for every actuator Menagerie exposes
```

**Important:** `robot_id: default_robot` matches `DEFAULT_ROBOT_ID` from Chunk 2 Task 2.5 (because Menagerie's scene yaml doesn't set an explicit `robot_id`). Do NOT use `elrobot_follower` here — that'd cause bridge commands to reference the wrong robot.

- [ ] **Step 4: Validate preset loads via bridge preset loader**

```bash
cargo test -p st3215-compat-bridge preset_loader 2>&1 | tail -20
```

Expected: existing bridge preset tests pass. The new Menagerie preset file doesn't have a dedicated test yet (that's Task 4.4's walking skeleton test).

- [ ] **Step 5: Commit**

```bash
git add software/sim-bridges/st3215-compat-bridge/presets/menagerie-so-arm100.yaml
git commit -m "st3215-compat-bridge: add Menagerie walking skeleton preset"
```

---

### Task 4.3: Write `station-sim-menagerie.yaml`

**Files:**
- Create: `software/station/bin/station/station-sim-menagerie.yaml`

**Rationale:** Station's sim scenario yaml ties together `sim_runtime` (pointing at the scene yaml) + `st3215_compat_bridge` (pointing at the preset yaml). MVP-1's `station-sim.yaml` does this for ElRobot; we add a parallel file for Menagerie.

- [ ] **Step 1: Read the existing `station-sim.yaml` as template**

```bash
cat software/station/bin/station/station-sim.yaml
```

Record the structure — it's the template.

- [ ] **Step 2: Write `station-sim-menagerie.yaml`**

Create `software/station/bin/station/station-sim-menagerie.yaml` following the same structure as `station-sim.yaml`, but change:
- Any `world_manifest:` / `scene_yaml:` path → points at `hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml`
- Any `preset:` path for the bridge → points at `software/sim-bridges/st3215-compat-bridge/presets/menagerie-so-arm100.yaml`
- Any `legacy_bus_serial:` → `"sim://menagerie-so-arm100"` to match the preset's bus_serial
- Any `robot_id:` → `default_robot` to match Menagerie's DEFAULT_ROBOT_ID

Add a header comment explaining this is the Phase 1 walking skeleton scenario and is a permanent regression fixture.

- [ ] **Step 3: Validate yaml parses via station config validator**

```bash
cargo test -p station_iface test_config_validate 2>&1 | tail -10
```

Expected: existing station_iface validation tests pass. The new yaml isn't loaded by any test yet.

Alternatively, have station itself parse the file:

```bash
./target/debug/station -c software/station/bin/station/station-sim-menagerie.yaml --validate-only 2>&1 || true
```

(if `--validate-only` exists; otherwise skip this step and rely on Task 4.5's runtime check.)

- [ ] **Step 4: Commit**

```bash
git add software/station/bin/station/station-sim-menagerie.yaml
git commit -m "station: add Menagerie walking skeleton scenario yaml"
```

---

### Task 4.4: Write `test_menagerie_walking_skeleton.py`

**Files:**
- Create: `software/sim-server/tests/integration/test_menagerie_walking_skeleton.py`

**Rationale:** This is the **permanent regression fixture** for assumption A. It runs Menagerie through norma_sim's Python stack (bypassing the Rust bridge for speed) and verifies: (1) MJCF loads via new schema, (2) `mj_forward` has no self-collisions, (3) every actuator can be driven and the result reaches `data.ctrl`, (4) 10000 random-ctrl steps produce no NaN. Not a full stack test — that's the manual smoke in Task 4.5.

- [ ] **Step 1: Write the test file**

Create `software/sim-server/tests/integration/test_menagerie_walking_skeleton.py`:

```python
"""Walking skeleton: prove norma_sim infra works with Menagerie SO-ARM100
verbatim. Baseline for assumption A ("infra is robot-agnostic").

MUST remain green indefinitely — if this file breaks, infra has regressed
even if ElRobot still works. The Menagerie MJCF is vendored unmodified,
so any change here is a signal that the change was to norma_sim, not to
ElRobot."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

try:
    import mujoco
    from norma_sim.world.manifest import load_manifest
    from norma_sim.world.model import MuJoCoWorld
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"imports not OK: {_ERR}")


@pytest.fixture
def menagerie_walking_skeleton_yaml() -> Path:
    here = Path(__file__).resolve()
    # parents: [0]=tests/integration, [1]=tests, [2]=sim-server, [3]=software, [4]=repo
    repo_root = here.parents[4]
    p = repo_root / "hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml"
    if not p.exists():
        pytest.skip(
            f"Menagerie scene yaml not found at {p}; run Chunk 4 Task 4.1 first"
        )
    return p


def test_menagerie_scene_yaml_loads(menagerie_walking_skeleton_yaml: Path):
    """The scene yaml parses, the referenced MJCF exists, and load_manifest
    produces a non-empty actuator list."""
    manifest = load_manifest(menagerie_walking_skeleton_yaml)
    assert manifest.world_name == "menagerie_trs_so_arm100"
    assert manifest.mjcf_path.exists()
    assert len(manifest.robots) == 1
    assert len(manifest.robots[0].actuators) >= 5


def test_menagerie_mujoco_world_loads(menagerie_walking_skeleton_yaml: Path):
    """MuJoCoWorld.from_manifest_path succeeds end-to-end: load yaml,
    open MJCF, build lookups."""
    world = MuJoCoWorld.from_manifest_path(menagerie_walking_skeleton_yaml)
    assert world.model.nu >= 5
    assert world.model.nv >= 5
    # Every actuator in the manifest should have a resolved MJCF index
    for robot in world.manifest.robots:
        for act in robot.actuators:
            idx = world.actuator_id_for(act.mjcf_actuator)
            assert idx is not None, f"{act.mjcf_actuator} not cached"


def test_menagerie_no_self_collision_at_rest(menagerie_walking_skeleton_yaml: Path):
    """mj_forward at the default pose should produce zero contacts.
    Menagerie's trs_so_arm100 is hand-tuned to avoid the self-collision
    issues the MVP-1 ElRobot URDF had."""
    world = MuJoCoWorld.from_manifest_path(menagerie_walking_skeleton_yaml)
    mujoco.mj_forward(world.model, world.data)
    assert world.data.ncon == 0, (
        f"Menagerie should have clean collision at rest, got {world.data.ncon} contacts"
    )


def test_menagerie_step_advances_time(menagerie_walking_skeleton_yaml: Path):
    world = MuJoCoWorld.from_manifest_path(menagerie_walking_skeleton_yaml)
    t0 = float(world.data.time)
    for _ in range(100):
        world.step()
    t1 = float(world.data.time)
    assert t1 > t0
    # All qpos values still finite after 100 steps at rest
    assert np.isfinite(world.data.qpos).all()
    assert np.isfinite(world.data.qvel).all()


def test_menagerie_all_actuators_drivable(menagerie_walking_skeleton_yaml: Path):
    """Every actuator should accept a ctrl write and step without NaN.
    Drive each actuator to its ctrlrange midpoint for 200 steps
    (~0.4 sec sim) and verify qpos stays finite."""
    world = MuJoCoWorld.from_manifest_path(menagerie_walking_skeleton_yaml)
    ctrl_mid = (
        world.model.actuator_ctrlrange[:, 0] + world.model.actuator_ctrlrange[:, 1]
    ) / 2
    world.data.ctrl[:] = ctrl_mid
    for _ in range(200):
        world.step()
        assert np.isfinite(world.data.qpos).all(), "NaN during mid-ctrl drive"


def test_menagerie_stress_10000_random_steps_no_nan(menagerie_walking_skeleton_yaml: Path):
    """Stress test: 10000 random-ctrl steps, resampling every 100 steps.
    This is the Floor 3 analog for the Menagerie baseline."""
    world = MuJoCoWorld.from_manifest_path(menagerie_walking_skeleton_yaml)
    rng = np.random.default_rng(42)
    lo = world.model.actuator_ctrlrange[:, 0]
    hi = world.model.actuator_ctrlrange[:, 1]
    for step in range(10000):
        if step % 100 == 0:
            world.data.ctrl[:] = rng.uniform(lo, hi)
        world.step()
        if step % 1000 == 0:
            assert np.isfinite(world.data.qpos).all(), f"NaN at step {step}"
    assert np.isfinite(world.data.qpos).all()
    assert np.isfinite(world.data.qvel).all()
```

- [ ] **Step 2: Run the test**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/integration/test_menagerie_walking_skeleton.py -v
```

Expected: 6 tests PASSED. If any fails, the walking skeleton for Menagerie is broken — STOP and debug before proceeding.

- [ ] **Step 3: Commit**

```bash
git add software/sim-server/tests/integration/test_menagerie_walking_skeleton.py
git commit -m "test: add Menagerie walking skeleton regression suite"
```

---

### Task 4.5: Manual browser smoke test for Phase 1

**Files:** None (manual verification)

**Rationale:** The automated test in Task 4.4 runs Menagerie through `norma_sim` (Python only, no Rust). The full stack smoke test starts the station binary with `station-sim-menagerie.yaml`, connects a browser, and drags sliders. This is the **Phase 1 gate** — if the browser shows sliders and they respond, hypothesis A is confirmed.

- [ ] **Step 1: Start station with Menagerie config**

```bash
./target/debug/station -c software/station/bin/station/station-sim-menagerie.yaml --web 0.0.0.0:8889 2>&1 | tail -20
```

Expected stdout within ~5 seconds:
- Lines about NormFS mounting
- Lines about sim-runtime starting subprocess
- Line about subprocess welcomed with Menagerie world_name
- Line about st3215-compat-bridge registering inference queue
- Line about web server listening on `0.0.0.0:8889`

If any error appears, STOP and capture the output. Common issues:
- `ImportError: cannot import name 'verify_source_hash'` → Chunk 2 Task 2.2 Step 2b was skipped. Go back and do it.
- `ValueError: scene.yaml ... missing required 'mjcf_path'` → Chunk 4 Task 4.1 yaml has a typo.
- `no such actuator 'shoulder_pan' in MJCF` → Task 4.2 preset references wrong actuator_id names.

- [ ] **Step 2: Open `http://localhost:8889` in a browser**

Expected:
- Page loads (no "connect a robot" empty state — Chunk 2 cli.py fix should have propagated)
- ST3215 viewer shows a bus with motor_id 1..N where N is the Menagerie actuator count
- Each motor has a "POS" reading (currently at the midpoint or zero)

If the empty state appears: the bridge isn't registering its inference queue. Check station logs for `register_queue` call.

- [ ] **Step 3: Drag one slider, verify response**

In the ST3215 viewer:
- Change any motor's "Control Source" dropdown to "Web" (if not already)
- Drag its "Goal Position" slider slowly
- Expected: the 3D view (if visible) shows the corresponding joint rotating smoothly
- Expected: the POS reading updates to track the slider
- Release the slider and verify the motor holds position (doesn't spring back)

Repeat for a different motor to confirm multiple motors work simultaneously.

- [ ] **Step 4: Side-by-side with MuJoCo native viewer (visual quality baseline)**

In a separate terminal:

```bash
python -m mujoco.viewer hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml
```

In the MuJoCo viewer window, drag the same joints you tested in the browser. Compare visual quality:
- Do they render equivalently? (No obvious frame drops or artifacts in the browser that aren't in the MuJoCo native view.)
- Does the arm's physical response feel the same? (No obvious damping differences or lag.)

If the browser view visibly lags or differs from MuJoCo viewer: the web rendering path may have latency issues (Chunk 3 inference-states pipeline) or frame-drop issues. Record the discrepancy but do not block on it — Phase 1 is about infra working, not pixel-perfect parity.

- [ ] **Step 5: Shutdown cleanly**

In the station terminal, press Ctrl+C. Expected:
- Station subprocess terminates cleanly (no "kill -9 required" messages)
- Station process exits with code 0
- No "BrokenPipeError" or "socket address already in use" on next startup

- [ ] **Step 6: Record the smoke test outcome**

No commit — this is manual verification. But record a short note in `/tmp/phase1_smoke_results.txt` (or similar) with:
- Date + time
- Menagerie commit SHA (from Chunk 1 Task 1.2's VENDOR.md)
- Station commit SHA
- Number of motors observed in the UI
- Any visual or functional discrepancies noted
- Whether Phase 1 gate passes (Y/N)

---

### Task 4.6: Chunk 4 gate

**Files:** None (verification)

- [ ] **Step 1: Run full test suite**

```bash
make sim-test 2>&1 | tail -20
```

Expected:
- Architecture invariants ✓
- All Rust tests pass (zero Rust touched in Chunk 4)
- Python: Chunk 3's expected pass + Chunk 4's new 6 tests = even higher green count
- Zero failures

- [ ] **Step 2: Confirm Phase 1 manual smoke passed**

Task 4.5 must have a Y outcome. If N, STOP and debug before proceeding to Chunk 5.

- [ ] **Step 3: Chunk 4 completion summary**

1. ✅ `menagerie_so_arm100.scene.yaml` exists and loads
2. ✅ `presets/menagerie-so-arm100.yaml` exists and validates
3. ✅ `station-sim-menagerie.yaml` exists and starts station successfully
4. ✅ `test_menagerie_walking_skeleton.py` has 6 green tests
5. ✅ Phase 1 manual browser smoke test passed
6. ✅ MuJoCo viewer side-by-side comparison shows equivalent rendering
7. ✅ Rust + arch invariants unchanged
8. ✅ No new test failures

**Do NOT proceed to Chunk 5 if any of 1-8 fail.** Phase 1 is the gate for hypothesis A; Chunk 5 would be building Phase 2 on unverified infra otherwise.

---

**Next:** Chunk 5 hand-writes the ElRobot 8-joint MJCF by adapting Menagerie's parameters.

---

## Chunk 5: Phase 2 — ElRobot 8-joint MJCF construction

**Purpose:** Hand-write `elrobot_follower.xml` as the ElRobot-specific MJCF, using Menagerie's hand-tuned parameters as the physics baseline and ElRobot's URDF as the kinematic topology reference. Produce the Menagerie comparison table as a spec artifact. Delete `gen.py` and the old `worlds/` directory. Restore `station-sim.yaml` to point at ElRobot (from Chunk 4's Menagerie detour).

**Gate:** `hardware/elrobot/simulation/elrobot_follower.xml` loads via `MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)` without errors. `mj_forward` produces `data.ncon == 0`. All 8 actuators enumerate. The tendon-based gripper mimic (Chunk 5's P0 invariant) still works. `gen.py` is deleted. `worlds/` directory is deleted.

**Prerequisites:** Chunks 1-4 complete. Chunk 4's walking skeleton test is green and stays green throughout Chunk 5.

**Files touched:**
- Create: `hardware/elrobot/simulation/elrobot_follower.xml` (hand-written MJCF)
- Create: `hardware/elrobot/simulation/elrobot_follower.scene.yaml` (new scene config)
- Create: `docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md` (spec artifact)
- Delete: `hardware/elrobot/simulation/worlds/gen.py`
- Delete: `hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml`
- Delete: `hardware/elrobot/simulation/worlds/elrobot_follower.xml` (the old gen.py-produced one)
- Delete: `hardware/elrobot/simulation/worlds/README.md` (if it exists)
- Delete: `hardware/elrobot/simulation/worlds/` directory (empty after above)
- Modify: `software/station/bin/station/station-sim.yaml` (update scene yaml path to new location)
- Modify: `Makefile` (remove `regen-mjcf` target and references)

---

### Task 5.1: Produce the Menagerie comparison table (research spike)

**Files:**
- Create: `docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md`

**Rationale:** Before writing the MJCF, read Menagerie's `trs_so_arm100.xml` carefully and produce an explicit mapping from each ElRobot joint to its Menagerie analog (if any), recording the armature/damping/inertial values we plan to copy. This makes Chunk 5's subsequent MJCF work deterministic and auditable.

**Topology baseline (confirmed 2026-04-11 via external research):**

- **Menagerie SO-ARM100 is 5-DOF** (5 revolute + 1 gripper = 6 actuators). Last tuning 2025-06-09 (joint limits matched to real hardware).
- **Menagerie has no SO-101 variant.** The upstream `TheRobotStudio/SO-ARM100/Simulation/SO101` has a 6-DOF SO-101 MJCF but it is auto-generated via `onshape-to-robot` and **not hand-tuned** — armature values are missing/zero and cannot be directly copied.
- **ElRobot is 7+1 DOF = 8 actuators**, so the gap is **3 extra joints** to nearest-neighbor estimate, not 2-3. Likely candidates (to be confirmed by URDF axis comparison in Step 2): M2 Shoulder Roll + M3 Shoulder Yaw + M7 Wrist Yaw, with M1/M4/M5/M6/M8 mapping cleanly to Menagerie's 5-DOF chain + gripper.

Chunk 1 Task 1.1 Step 7's reading of `lachlanhurst/so100-mujoco-sim` may have surfaced additional insights about how another project mapped ElRobot-family arms to Menagerie — incorporate those notes here if relevant.

- [ ] **Step 1: Read Menagerie's trs_so_arm100.xml**

Use the Read tool on `hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/trs_so_arm100.xml`. Identify:
- The `<default>` block with joint armature/damping defaults
- Each joint's inertial properties (via parent body's `<inertial>` element)
- The actuator section with kp/kv/forcerange defaults
- Gripper implementation details (tendon, equality, mimic joints)

- [ ] **Step 2: Read ElRobot's URDF to cross-reference**

Use the Read tool on `hardware/elrobot/simulation/elrobot_follower.urdf`. For each of the 8 joints (M1-M8), note:
- Parent/child link names
- Axis of rotation
- Joint limits (from URDF `<limit>`)
- Existing inertial values (from URDF `<inertial>`)
- Mesh file references

- [ ] **Step 3: Write the comparison table**

Create `docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md`:

```markdown
# MVP-2 Menagerie → ElRobot Parameter Comparison Table

| | |
|---|---|
| **日期** | 2026-04-11 (或执行时实际日期) |
| **状态** | Phase 2 research spike 产物 |
| **Menagerie source** | mujoco_menagerie/trs_so_arm100 @ commit {SHA from VENDOR.md} |
| **Target** | hardware/elrobot/simulation/elrobot_follower.xml |

## 拓扑对照

### Menagerie SO-ARM100（5 revolute + 1 gripper，**共 6 actuators，非 5-6**）

| Menagerie joint | 类型 | armature | damping | frictionloss | 备注 |
|---|---|---|---|---|---|
| {填入 j1 - 通常是 shoulder_pan/shoulder_lift/elbow/wrist_flex/wrist_roll 某一个} | hinge | {value} | {value} | {value} | {ElRobot 对应？} |
| {填入 j2} | hinge | {value} | {value} | {value} | {ElRobot 对应？} |
| {填入 j3} | hinge | {value} | {value} | {value} | {ElRobot 对应？} |
| {填入 j4} | hinge | {value} | {value} | {value} | {ElRobot 对应？} |
| {填入 j5} | hinge | {value} | {value} | {value} | {ElRobot 对应？} |
| {填入 gripper} | {hinge/slide} | {value} | {value} | {value} | {ElRobot 对应？} |

### ElRobot (7 revolute + 1 gripper = 8 actuators)

| Joint | ElRobot URDF link | Menagerie analog | armature | damping | frictionloss | 来源 |
|---|---|---|---|---|---|---|
| M1 Shoulder Pitch | {link 1} | {likely shoulder_lift} | {copy from analog} | {copy} | {copy} | menagerie direct |
| M2 Shoulder Roll | {link 2} | **无对应** | {estimate from M1} | {estimate} | {estimate} | nearest-neighbor (M1) |
| M3 Shoulder Yaw | {link 3} | {likely shoulder_pan, or 无对应 if Menagerie's pan is pitch-axis} | ... | ... | ... | menagerie direct OR nearest-neighbor |
| M4 Elbow | {link 4} | {elbow} | ... | ... | ... | menagerie direct |
| M5 Wrist Roll | {link 5} | {wrist_roll if present; else nearest wrist joint} | ... | ... | ... | menagerie direct OR nearest-neighbor |
| M6 Wrist Pitch | {link 6} | {wrist_flex} | ... | ... | ... | menagerie direct |
| M7 Wrist Yaw | {link 7} | **无对应** | {estimate from M6} | {estimate} | {estimate} | nearest-neighbor (M6) |
| M8 Gripper | {link 8} | {gripper} | ... | ... | ... | menagerie direct |

**Expected independent joints: 3** (M2 Shoulder Roll + one of M3/M5 + M7 Wrist Yaw, depending on Menagerie's actual 5-joint chain — confirm by axis comparison).

## 参数继承策略

1. **`<option>`**: 直接继承 Menagerie (timestep, integrator, solver, iterations, tolerance)
2. **`<default>` classes**:
   - Menagerie 有 classes: {list}
   - ElRobot 继承所有 classes 不修改
3. **Actuator kp/kv/forcerange**: Menagerie 默认值作为 ElRobot 的 baseline. 若 Phase 2 smoke test 过不了 Floor 4 step response, 允许 per-joint 微调
4. **Gripper mimic**: Menagerie 使用 {tendon / weld / 其它}. ElRobot 保留 MVP-1 的 `<tendon><fixed>` + `<equality><tendon>` 实现（P0 不可破）

## 独有关节的估值依据（3 个关节，非 2）

| ElRobot 关节 | 估值来源 | 备注 |
|---|---|---|
| {ElRobot-only #1, e.g. M2 Shoulder Roll} | {nearest Menagerie shoulder joint} | {同为 shoulder 组，惯量量级相近} |
| {ElRobot-only #2, e.g. M3 Shoulder Yaw if Menagerie only has pan/lift} | {nearest shoulder joint} | {axis 最接近的} |
| {ElRobot-only #3, e.g. M7 Wrist Yaw} | {nearest wrist joint} | {同为 wrist 组，惯量极小} |

(Fill in after confirming Menagerie's actual joint list in Step 1. The 3 independent joints may differ from the guess above — go by axis alignment, not joint name.)

## Risk notes

- Menagerie 的 {具体 class / 具体参数} 如果和 ElRobot 不兼容, 记录 deviation 并说明 why
- 若 Menagerie 的 kp 过低导致 ElRobot 的重关节过不了 Floor 4, 允许调高 kp 但 armature 必须保持 Menagerie 值
```

- [ ] **Step 4: Fill in all `{填入}` placeholders with actual data from Step 1 and Step 2**

This is the **manual research effort** — read both files carefully and produce a concrete table. No test automation can do this.

- [ ] **Step 5: Commit the comparison table**

```bash
git add docs/superpowers/specs/2026-04-11-mvp2-menagerie-comparison-table.md
git commit -m "spec: MVP-2 Menagerie → ElRobot parameter comparison table"
```

---

### Task 5.2: Hand-write `elrobot_follower.xml`

**Files:**
- Create: `hardware/elrobot/simulation/elrobot_follower.xml`

**Rationale:** The hand-written MJCF is the core deliverable of Phase 2. It must:
1. Use Menagerie-inherited `<option>` and `<default>` blocks
2. Have 8 `<position>` actuators for M1-M8
3. Construct the ElRobot body tree from URDF kinematics
4. Reference `../assets/*.stl` for visual meshes
5. Use primitive collision geometry (boxes/capsules) to avoid MVP-1's self-collision issues
6. Preserve the gripper tendon-mimic structure that Chunk 1's P0 tests require

- [ ] **Step 1: Start from Menagerie's `<option>` + `<default>` as template**

Copy Menagerie's `trs_so_arm100.xml` `<option>` and `<default>` blocks into the new file. Preserve the class hierarchy (`arm_link`, `visual`, `collision`, etc. — whatever Menagerie uses).

- [ ] **Step 2: Construct the body tree from ElRobot URDF**

For each URDF `<joint>` (M1-M7 revolute + M8 gripper + 2 mimic), emit a matching `<body>` + `<joint>` + `<geom>` triple in the MJCF. Use URDF's pos/axis/range exactly. Use ElRobot's own STL files via `../assets/*.stl` relative paths.

Example structure:

```xml
<worldbody>
  <body name="base_link" pos="0 0 0">
    <geom type="mesh" mesh="base_link" class="visual"/>
    <geom type="box" size="0.05 0.05 0.03" pos="0 0 0.015" class="collision"/>
    <body name="link_01" pos="{from URDF}" quat="{from URDF}">
      <joint name="rev_motor_01" type="hinge" axis="{from URDF}" range="{from URDF}"/>
      <geom type="mesh" mesh="Joint_01_1" class="visual"/>
      <geom type="capsule" ... class="collision"/>
      <body name="link_02" ...>
        <!-- continue down the chain -->
      </body>
    </body>
  </body>
</worldbody>
```

For M8 gripper: replicate MVP-1's tendon + equality structure (which P0 tests depend on). The MVP-1 `elrobot_follower.xml` (generated by gen.py, about to be deleted) is a fine reference for the gripper block — grep it before deleting:

```bash
grep -A 30 '<tendon>\|<equality>' hardware/elrobot/simulation/worlds/elrobot_follower.xml > /tmp/mvp1_gripper_block.txt
```

Use `/tmp/mvp1_gripper_block.txt` as the gripper section of the new MJCF (with any path/name fixups).

- [ ] **Step 3: Apply Menagerie-inherited `armature` / `damping` to joints**

Using the comparison table from Task 5.1, add per-joint `armature=` and `damping=` attributes. Joints with Menagerie analogs get the analog values; the **3 ElRobot-unique joints** (see Task 5.1's "独有关节的估值依据" section — likely M2 Shoulder Roll + one of M3/M5 + M7 Wrist Yaw) get nearest-neighbor estimates.

Example:

```xml
<joint name="rev_motor_01" ... armature="0.015" damping="0.3"/>
```

(Exact numbers depend on Task 5.1's findings.)

- [ ] **Step 4: Emit the actuator section**

For each of 8 actuators, emit a `<position>` element with kp/kv from the Menagerie comparison table and ctrlrange/forcerange from ElRobot's URDF limits:

```xml
<actuator>
  <position name="act_motor_01" joint="rev_motor_01"
            kp="{menagerie kp}" kv="{menagerie kv}"
            ctrlrange="{urdf range}" forcerange="{-urdf effort} {urdf effort}"/>
  ...
  <position name="act_motor_08" joint="rev_motor_08"
            kp="10" kv="0.3"
            ctrlrange="0 2.2028" forcerange="-2.94 2.94"/>
</actuator>
```

- [ ] **Step 5: Use primitive collision geometry, not full mesh**

For every `<geom class="collision">` or equivalent, use a primitive shape (box, sphere, capsule) approximating the link. Do NOT use `type="mesh"` for collision geoms — this is what avoided Menagerie's self-collision issues and is spec §7.3 item 3's explicit strategy.

- [ ] **Step 6: Load test**

```bash
python -m mujoco.viewer hardware/elrobot/simulation/elrobot_follower.xml
```

Expected: the ElRobot arm renders in the viewer. Manual drag test: rotate a joint, verify it moves smoothly. Close viewer.

If the MJCF has compile errors: debug by reading the MuJoCo error message. Common issues:
- Mesh path wrong → adjust `../assets/` relative path
- Joint axis not normalized → URDF might need re-export
- Inertia singular (body has zero mass) → add a minimal `<inertial>` block to the offending body

- [ ] **Step 7: Commit**

```bash
git add hardware/elrobot/simulation/elrobot_follower.xml
git commit -m "mvp2: hand-write elrobot_follower.xml for Phase 2"
```

---

### Task 5.3: Write `elrobot_follower.scene.yaml`

**Files:**
- Create: `hardware/elrobot/simulation/elrobot_follower.scene.yaml`

- [ ] **Step 1: Write the scene yaml**

Create `hardware/elrobot/simulation/elrobot_follower.scene.yaml`:

```yaml
# MVP-2 Phase 2 — ElRobot scene config.
#
# Replaces MVP-1's worlds/elrobot_follower.world.yaml which was both
# gen.py manifest AND scene config. This file is pure scene config —
# load the hand-written MJCF and annotate the gripper.

world_name: elrobot_follower
robot_id: elrobot_follower
mjcf_path: ./elrobot_follower.xml

# No scene_overrides — elrobot_follower.xml's <option> is authoritative.
# No scene_extras — the MJCF has its own lighting/floor setup.

actuator_annotations:
  - mjcf_actuator: act_motor_08
    actuator_id: rev_motor_08
    display_name: Gripper
    capability:
      kind: GRIPPER_PARALLEL
      limit_min: 0.0
      limit_max: 1.0
      effort_limit: 2.94
      velocity_limit: 4.71
      normalized_range: [0.0, 1.0]
    gripper:
      primary_joint_range_rad: [0.0, 2.2028]
      mimic_joints:
        - {joint: rev_motor_08_1, multiplier: -0.0115}
        - {joint: rev_motor_08_2, multiplier: 0.0115}
```

- [ ] **Step 2: Smoke test loader**

```bash
PYTHONPATH=software/sim-server python3 -c "
from pathlib import Path
from norma_sim.world.model import MuJoCoWorld
world = MuJoCoWorld.from_manifest_path(Path('hardware/elrobot/simulation/elrobot_follower.scene.yaml'))
print(f'nu={world.model.nu}')
print(f'neq={world.model.neq}')
print(f'ntendon={world.model.ntendon}')
gripper = world.actuator_by_mjcf_name('act_motor_08')
assert gripper is not None
assert gripper.capability.kind == 'GRIPPER_PARALLEL'
print('ElRobot scene yaml loads OK')
"
```

Expected: `nu=8`, `neq=2`, `ntendon=2`, "ElRobot scene yaml loads OK".

- [ ] **Step 3: Commit**

```bash
git add hardware/elrobot/simulation/elrobot_follower.scene.yaml
git commit -m "mvp2: add elrobot_follower.scene.yaml pointing at hand-written MJCF"
```

---

### Task 5.4: Run ElRobot-flavored tests that were skipped during Chunks 2-4

**Files:** None (verification; tests are already in place from Chunk 3)

**Rationale:** Chunks 2-4 left several tests marked `pytest.skip` because `elrobot_scene_yaml` fixture didn't exist. Now that it does, run them.

- [ ] **Step 1: Run previously-skipped ElRobot tests**

```bash
PYTHONPATH=software/sim-server python3 -m pytest \
    software/sim-server/tests/world/test_model.py::test_mujoco_world_loads_elrobot_mjcf \
    software/sim-server/tests/world/test_model.py::test_mujoco_world_elrobot_actuator_lookups \
    software/sim-server/tests/world/test_model.py::test_mujoco_world_elrobot_joint_qposadr_lookups \
    software/sim-server/tests/world/test_model.py::test_mujoco_world_elrobot_actuator_by_mjcf_name \
    software/sim-server/tests/world/test_snapshot.py::test_snapshot_initial_state_elrobot \
    software/sim-server/tests/world/test_snapshot.py::test_snapshot_tracks_ctrl_goal_elrobot \
    software/sim-server/tests/world/test_snapshot.py::test_snapshot_gripper_reports_normalized_elrobot \
    software/sim-server/tests/world/test_mimic_gripper.py \
    software/sim-server/tests/world/test_descriptor_build.py::test_build_world_descriptor_happy_elrobot \
    software/sim-server/tests/world/test_descriptor_build.py::test_build_world_descriptor_actuator_kinds_elrobot \
    software/sim-server/tests/world/test_descriptor_build.py::test_build_world_descriptor_encodes_elrobot \
    -v
```

Expected: all tests PASS. **P0 gripper mimic tests** (`test_mimic_gripper.py`) must pass — if they don't, Task 5.2's MJCF gripper block is wrong and must be fixed before proceeding.

- [ ] **Step 2: Run the walking skeleton test to confirm Menagerie still works**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/integration/test_menagerie_walking_skeleton.py -v
```

Expected: all 6 tests still green. If they regressed, we broke something in Chunk 5 that affects norma_sim.world (shouldn't have touched any of it, but double-check).

- [ ] **Step 3: No commit — pure verification**

---

### Task 5.5: Delete `gen.py`, `worlds/` directory, and update `Makefile` + `station-sim.yaml`

**Files:**
- Delete: `hardware/elrobot/simulation/worlds/gen.py`
- Delete: `hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml`
- Delete: `hardware/elrobot/simulation/worlds/elrobot_follower.xml`
- Delete: `hardware/elrobot/simulation/worlds/README.md` (if exists)
- Delete: `hardware/elrobot/simulation/worlds/` directory
- Modify: `Makefile` (remove `regen-mjcf` target + references)
- Modify: `software/station/bin/station/station-sim.yaml` (update scene yaml path)

**Rationale:** The MVP-1 artifacts are no longer needed (MJCF is hand-written, no generation pipeline). Clean them up. Also repoint the MVP-1 station-sim.yaml to the new scene yaml location.

- [ ] **Step 1: Delete `worlds/` contents**

```bash
ls hardware/elrobot/simulation/worlds/
```

Expected: `gen.py`, `elrobot_follower.world.yaml`, `elrobot_follower.xml`, possibly `README.md`.

```bash
git rm hardware/elrobot/simulation/worlds/gen.py
git rm hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml
git rm hardware/elrobot/simulation/worlds/elrobot_follower.xml
[if README exists] git rm hardware/elrobot/simulation/worlds/README.md
```

Then remove the now-empty directory (git removes it automatically when all files are gone):

```bash
ls hardware/elrobot/simulation/worlds/ 2>&1
```

Expected: `No such file or directory` (git does not track empty directories).

- [ ] **Step 2: Update `Makefile`**

```bash
grep -n 'regen-mjcf\|worlds/gen.py\|worlds/' Makefile
```

Expected hits:
- A `regen-mjcf:` target (delete it)
- Any references to `worlds/gen.py` or `worlds/elrobot_follower.xml` in other targets (delete)
- The `.PHONY: regen-mjcf` line if present (delete)

Apply the deletions with the Edit tool. Verify clean:

```bash
grep -n 'regen-mjcf\|worlds/gen.py' Makefile
```

Expected: no matches.

- [ ] **Step 3: Update `station-sim.yaml`**

```bash
grep -n 'world_manifest\|scene_yaml\|world.yaml' software/station/bin/station/station-sim.yaml
```

Find the line that references the old scene yaml path. Update it:

```yaml
# before:
world_manifest: hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml
# after:
world_manifest: hardware/elrobot/simulation/elrobot_follower.scene.yaml
```

(Exact field name depends on station_iface schema — it might be `scene_yaml` or `manifest` or similar; match whatever the existing yaml uses.)

- [ ] **Step 4: Smoke test — station loads the updated config**

```bash
./target/debug/station -c software/station/bin/station/station-sim.yaml --validate-only 2>&1 || echo "no --validate-only, skipping"
```

Alternative if `--validate-only` doesn't exist: briefly start station and kill it:

```bash
timeout 5 ./target/debug/station -c software/station/bin/station/station-sim.yaml --web 0.0.0.0:8889 2>&1 | head -20
```

Expected: station starts, subprocess spawns, no errors about missing scene yaml. Kill with Ctrl+C after verifying it's running.

- [ ] **Step 5: Run `make sim-test` to verify regressions**

```bash
make sim-test 2>&1 | tail -15
```

Expected: all tests still green. The `worlds/` deletion should not affect tests (conftest.py fixtures point at the new location).

- [ ] **Step 6: Commit deletions + config updates as one atomic change**

```bash
git add -u hardware/elrobot/simulation/worlds/ Makefile software/station/bin/station/station-sim.yaml
git commit -m "mvp2: delete gen.py + worlds/ directory, repoint station-sim.yaml"
```

Commit body:

```
The URDF → MJCF pipeline is no longer needed in MVP-2. elrobot_follower.xml
is hand-written at hardware/elrobot/simulation/elrobot_follower.xml (see
Chunk 5 Task 5.2). worlds/gen.py and its outputs are deleted.

Station-sim.yaml is repointed to the new scene yaml location. The
Makefile loses its regen-mjcf target.
```

---

### Task 5.6: Chunk 5 gate

**Files:** None (verification)

- [ ] **Step 1: Run full test suite**

```bash
make sim-test 2>&1 | tail -20
```

Expected:
- Architecture invariants ✓
- All Rust tests pass (zero Rust touched)
- Python: Chunk 4 count + all previously-skipped ElRobot tests now pass
- Zero failures

- [ ] **Step 2: Verify `gen.py` is gone**

```bash
find hardware/elrobot/simulation -name 'gen.py' -o -name '*.world.yaml'
```

Expected: no matches.

- [ ] **Step 3: Verify Menagerie walking skeleton still green**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/integration/test_menagerie_walking_skeleton.py -v
```

Expected: 6 tests still PASS. Permanent regression fixture working.

- [ ] **Step 4: Chunk 5 completion summary**

1. ✅ Comparison table artifact committed (`2026-04-11-mvp2-menagerie-comparison-table.md`)
2. ✅ `elrobot_follower.xml` hand-written and loads via MuJoCo
3. ✅ `elrobot_follower.scene.yaml` loads via `MuJoCoWorld.from_manifest_path`
4. ✅ All previously-skipped ElRobot tests now pass (including P0 gripper mimic)
5. ✅ `gen.py` and `worlds/` directory deleted
6. ✅ `Makefile` + `station-sim.yaml` updated
7. ✅ Walking skeleton still green (permanent regression holds)
8. ✅ Architecture invariants + Rust tests unchanged

---

**Next:** Chunk 6 writes the `test_elrobot_acceptance.py` with the 6 Floor criteria from spec §3.1.

---

## Chunk 6: Phase 2 — Acceptance tests (Floor §3.1)

**Purpose:** Code-ify the 6 Floor acceptance criteria from spec §3.1 as automated tests. Each criterion becomes one or more pytest functions. The parametrized per-motor step response test (Floor 4) is the most rigorous — it fails loudly if any single motor doesn't respond within spec.

**Gate:** All 6 Floor criteria pass on the ElRobot MJCF from Chunk 5. The per-motor parametrize test shows 8 PASSED (one per motor). If any motor fails, iterate on ElRobot MJCF tuning (adjust kp/kv) up to the **5-iteration tuning budget** before escalating.

**Prerequisites:** Chunks 1-5 complete.

**Files touched:**
- Create: `software/sim-server/tests/integration/test_elrobot_acceptance.py`

---

### Task 6.1: Write the acceptance test file

**Files:**
- Create: `software/sim-server/tests/integration/test_elrobot_acceptance.py`

- [ ] **Step 1: Write the file**

Create `software/sim-server/tests/integration/test_elrobot_acceptance.py`:

```python
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
```

- [ ] **Step 2: Run the file**

```bash
PYTHONPATH=software/sim-server python3 -m pytest software/sim-server/tests/integration/test_elrobot_acceptance.py -v
```

Expected outcome **depends on Chunk 5's MJCF tuning**:

- **Best case**: all tests PASS on first run. Floors 1-3 are almost always satisfied by Menagerie parameters + clean collision primitives. Floor 4 (per-motor step response) is the fragile one — if Menagerie's kp/kv transfer cleanly, it passes too.

- **Common case**: 1-3 motors fail Floor 4 with "only reached X%". This is the **tuning iteration** (spec §7.5 / §10 Risk B). For each failing motor:
  1. Increase `kp` on that motor's `<position>` in `elrobot_follower.xml` by 25-50%.
  2. If the motor then overshoots, increase `kv` proportionally (typically kv = 0.1 × kp for critically damped).
  3. Re-run the failing test: `pytest ... -k test_elrobot_motor_step_response -v`.
  4. If after 5 iterations the motor still fails, **STOP** and escalate per spec Risk B. Options:
     - Revisit the Menagerie comparison table for that joint
     - Increase the joint's `armature` (adds effective inertia, smooths response)
     - Widen the Floor 4 tolerance (80% → 70%, 2s → 3s) as a **spec amendment** — must re-run spec review loop.

- **Unlikely case**: Floors 1/2/3 fail. This means Chunk 5's MJCF is structurally broken. Go back to Task 5.2 and fix the mesh/inertial/tendon issues before re-running Chunk 6.

- [ ] **Step 3: If all tests pass, commit**

```bash
git add software/sim-server/tests/integration/test_elrobot_acceptance.py
git commit -m "test: MVP-2 Phase 2 acceptance — 6 Floor criteria"
```

Commit body (adjust the "tuning" notes based on actual execution):

```
Codifies spec §3.1 Floor 1-6 criteria as automated tests:
- Floor 1: data.ncon == 0 at rest
- Floor 2: effective inertia floor (M[i,i] + armature >= 1e-4)
- Floor 3: 10000 random-ctrl steps, no NaN
- Floor 4: per-motor step response (parametrized, 8 tests)
- Floor 5: delegation stub (actual coverage in test_mimic_gripper.py)
- Floor 6: delegation stub (actual coverage in make sim-test)

All tests passing at commit time.
```

- [ ] **Step 4: If tuning was required, also commit the MJCF changes**

If you had to adjust `elrobot_follower.xml` kp/kv/armature values during Step 2's iteration loop, commit those as a separate commit before the test file:

```bash
git add hardware/elrobot/simulation/elrobot_follower.xml
git commit -m "mvp2: tune elrobot_follower.xml per Floor 4 response targets"
```

---

### Task 6.2: Chunk 6 gate

**Files:** None (verification)

- [ ] **Step 1: Run full test suite**

```bash
make sim-test 2>&1 | tail -20
```

Expected:
- Architecture invariants ✓
- Rust tests unchanged
- Python: everything from Chunks 2-5 + 13 new tests in `test_elrobot_acceptance.py` (3 Floor standalones + 8 parametrized Floor 4 + 2 delegation stubs)
- Zero failures

- [ ] **Step 2: Verify tuning budget wasn't exceeded**

If Chunk 6 required more than 5 tuning iterations on `elrobot_follower.xml`, that's a **spec amendment** situation — STOP, write an amendment note in the spec, re-run spec review, then continue.

- [ ] **Step 3: Chunk 6 completion summary**

1. ✅ `test_elrobot_acceptance.py` exists with 6 Floor criteria encoded
2. ✅ All 13 tests (3 floor standalone + 8 parametrized + 2 delegation) pass
3. ✅ `test_mimic_gripper.py` P0 tests still pass (Floor 5)
4. ✅ `make sim-test` zero failures (Floor 6)
5. ✅ Rust + arch invariants unchanged
6. ✅ If tuning was required, iteration budget not exceeded

**Do NOT proceed to Chunk 7 if any Floor criterion failed.**

---

**Next:** Chunk 7 updates the manual checklist, memory docs, and runs the final DoD check.

---

## Chunk 7: Phase 2 — Manual smoke test, docs, and MVP-2 wrap-up

**Purpose:** Run the manual browser smoke test (Ceiling acceptance criteria 7 + 8 from spec §3.2), update documentation (sim-server README manual checklist + sim_starting_point memory), and verify all 10 DoD items from spec §3.4 are ticked.

**Gate:** All 10 DoD items ticked. `sim_starting_point.md` reflects MVP-2 completion. Plan completed.

**Prerequisites:** Chunks 1-6 complete.

**Files touched:**
- Modify: `software/sim-server/README.md` (add MVP-2 Phase 2 manual checklist)
- Modify: `~/.claude/projects/-home-yuan-proj-norma-core/memory/sim_starting_point.md` (mark MVP-2 complete)

---

### Task 7.1: Manual browser smoke test for ElRobot (Ceiling §3.2)

**Files:** None (manual verification)

- [ ] **Step 1: Start station with ElRobot config**

```bash
./target/debug/station -c software/station/bin/station/station-sim.yaml --web 0.0.0.0:8889 2>&1 | tail -10
```

Expected: station starts, subprocess spawns with `elrobot_follower.scene.yaml`, web server listens on 8889.

- [ ] **Step 2: Open http://localhost:8889 and verify 8 motors visible**

In a browser, navigate to `http://localhost:8889`. Expected:
- No "connect a robot" empty state
- ST3215 viewer shows 8 motors (M1-M8) on the sim bus

- [ ] **Step 3: Drag each motor slider and verify smooth response**

For each of M1 through M7:
- Set control source to "Web"
- Drag the slider slowly from current position toward one extreme of its range
- Expected: 3D view (if visible) shows the joint rotating smoothly
- Expected: no oscillation, no visible jitter, no popping
- Release the slider and verify the motor holds position (doesn't droop — gravity has been validated via the MuJoCo viewer in Task 1.3)

Pay special attention to **M1 (Shoulder Pitch)** — this was the MVP-1 regression. If M1 is now responsive and smooth, Phase 2 has delivered on the physics debt.

- [ ] **Step 4: Drag M8 (Gripper) slider**

Drag the gripper slider 0 → 1 → 0. Expected:
- Primary joint and both mimic joints open/close
- No NaN artifacts (jaws don't disappear or fly apart)
- Multi-joint visual synchronization via tendon equality

- [ ] **Step 5: Multi-motor simultaneous test**

Drag M1 + M4 + M8 at the same time (e.g., open gripper while rotating shoulder while bending elbow). Expected:
- All 3 motors respond independently
- No interference between motors
- Arm motion looks natural (not jittery or physics-broken)

- [ ] **Step 6: Kill + restart test**

Press Ctrl+C to stop the station process. Restart:

```bash
./target/debug/station -c software/station/bin/station/station-sim.yaml --web 0.0.0.0:8889
```

Expected:
- Clean startup (no "address already in use")
- Arm returns to home pose (initial qpos)

- [ ] **Step 7: MuJoCo viewer side-by-side with Menagerie (Ceiling item 8)**

In a separate terminal:

```bash
python -m mujoco.viewer hardware/elrobot/simulation/elrobot_follower.xml
```

In another terminal:

```bash
python -m mujoco.viewer hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml
```

Drag joints in both viewers. Expected:
- ElRobot's response quality ≈ Menagerie SO-ARM100 quality
- No obvious "ours is worse" artifacts (no extra jitter, no weird motions, no obviously-wrong physics)

If the ElRobot viewer looks materially worse than Menagerie's: record the specific difference (e.g., "M1 oscillates when released in ElRobot but not in Menagerie"). This is **advisory** at Phase 2 gate — Floor §3.1 is the hard gate. If Ceiling §3.2 item 8 fails, record as a follow-up investigation but don't block MVP-2 completion.

- [ ] **Step 8: Record results**

In `/tmp/phase2_smoke_results.txt`:
- Date + time
- Which motors passed Step 3 (1-8 checklist)
- Whether gripper (Step 4) worked
- Whether multi-motor (Step 5) worked
- Whether kill+restart (Step 6) worked cleanly
- Visual comparison to Menagerie (Step 7) — match / ElRobot worse / ElRobot better
- Overall Ceiling §3.2 verdict: PASS / PASS with caveats / FAIL

---

### Task 7.2: Update `sim-server/README.md` manual checklist

**Files:**
- Modify: `software/sim-server/README.md`

**Rationale:** The MVP-1 README has a scenario A manual smoke test checklist. Replace it with the MVP-2 version that covers the ElRobot 8-motor expectations AND the Menagerie walking skeleton regression check.

- [ ] **Step 1: Read current README to find the checklist section**

```bash
grep -n 'checklist\|smoke test\|scenario A\|MVP-1' software/sim-server/README.md
```

- [ ] **Step 2: Replace the checklist section**

Find the "Manual browser smoke test" or similar section in the README. Replace with MVP-2 version:

```markdown
## Manual browser smoke test (MVP-2)

MVP-2 has two complementary smoke tests:

### Phase 1 baseline — Menagerie walking skeleton

This test validates that the station infrastructure works with any
MuJoCo-valid MJCF (hypothesis A: "infra is robot-agnostic"). Run
periodically after any change to station / sim-runtime / bridge code.

```bash
./target/debug/station -c software/station/bin/station/station-sim-menagerie.yaml --web 0.0.0.0:8889
```

- [ ] Browser shows Menagerie SO-ARM100's N motors (5-6)
- [ ] Dragging any slider smoothly rotates the corresponding joint
- [ ] `test_menagerie_walking_skeleton.py` pytest passes

### Phase 2 — ElRobot full 8-motor demo

This is the MVP-2 exit criterion. Validates that ElRobot's sim
env is usable for future policy training (spec §3.2 Ceiling).

```bash
./target/debug/station -c software/station/bin/station/station-sim.yaml --web 0.0.0.0:8889
```

- [ ] Browser connects, shows 8 motors (M1-M8) populated
- [ ] Drag M1 (Shoulder Pitch) slowly through full range
  - smooth response, no oscillation, no jitter
  - arm holds position when slider released (no droop)
- [ ] Repeat for M2 (Shoulder Roll), M3 (Shoulder Yaw), M4 (Elbow)
- [ ] Repeat for M5 (Wrist Roll), M6 (Wrist Pitch), M7 (Wrist Yaw)
- [ ] Drag M8 (Gripper) slider 0 → 1 → 0
  - mimic jaws open/close in sync
  - no NaN artifacts
- [ ] Multi-motor test: drag M1 + M4 + M8 simultaneously, no interference
- [ ] Kill station, restart, verify arm returns to home pose

### Side-by-side visual comparison (advisory)

```bash
python -m mujoco.viewer hardware/elrobot/simulation/elrobot_follower.xml
python -m mujoco.viewer hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml
```

- [ ] ElRobot response quality ≈ Menagerie SO-ARM100 quality
  (no obvious "ours is worse" artifacts)
```

(Preserve any other README sections unrelated to the smoke test.)

- [ ] **Step 3: Commit**

```bash
git add software/sim-server/README.md
git commit -m "docs(sim-server): update manual smoke checklist for MVP-2"
```

---

### Task 7.3: Update `sim_starting_point.md` memory for MVP-2 completion

**Files:**
- Modify: `~/.claude/projects/-home-yuan-proj-norma-core/memory/sim_starting_point.md`

**Rationale:** The memory doc currently says MVP-1 is complete and MVP-2 is pending. Update the relevant sections to reflect MVP-2 completion (assuming all Chunks 1-7 pass).

- [ ] **Step 1: Read current memory state**

Use the Read tool on `~/.claude/projects/-home-yuan-proj-norma-core/memory/sim_starting_point.md`.

- [ ] **Step 2: Apply updates**

Update the following sections:

1. **Top-of-file "交付状态" block**: Change from "MVP-1 已完成" to "MVP-1 + MVP-2 已完成". Add MVP-2 merge commit info.

2. **"执行结果一览" table**: Add MVP-2 row with test counts + chunk list.

3. **"Smoke test 真实结果" section**: Add an MVP-2 subsection noting that the 4-factor debt (forcerange / zero damping / self-collision / gripper near-zero inertia) is resolved by the Menagerie fork. Record specifically:
   - M1 now responsive (armature from Menagerie)
   - self-collision gone (primitive collision geometry from Menagerie pattern)
   - gripper primary joint stable (Menagerie inertial values + armature)
   - No gravity compensation needed (Menagerie approach works)

4. **"MVP-2 起点" section**: Rename to "MVP-2 完成 + MVP-3 起点" and update the Chunk 1-2 draft into a "Phase 0/1/2 executed" summary. Add MVP-3 outlook (env wrapper, policy training) as the next phase.

5. **"时间线" section**: Add 2026-04-11 + following dates entries for MVP-2 Chunk execution.

6. **"How to apply" section**: Add MVP-2 guidance:
   - "改 elrobot_follower.xml 前：先跑 test_menagerie_walking_skeleton.py 确认 infra 基线"
   - "ElRobot 的 Floor §3.1 acceptance 在 test_elrobot_acceptance.py"
   - "Menagerie 更新的合并流程：see vendor/menagerie/VENDOR.md"

- [ ] **Step 3: Save (no git commit — memory is outside the repo)**

Memory files live in `~/.claude/` and are not under git control. Just save the file via the Write tool.

---

### Task 7.4: Final MVP-2 DoD verification (spec §3.4)

**Files:** None (verification)

**Rationale:** Spec §3.4 lists 10 Definition-of-Done items for MVP-2. Verify each and record.

- [ ] **Step 1: Go through spec §3.4 exit criteria**

For each of the 10 items:

1. Phase 0 reconnaissance gate passed (Chunk 1 Task 1.1-1.3)
2. Phase 1 walking skeleton green (Chunk 4 Task 4.4-4.5)
3. Phase 2 ElRobot 8 motor operational (Chunk 7 Task 7.1)
4. Floor §3.1 (6 criteria) all automated tests pass (Chunk 6 Task 6.1-6.2)
5. Ceiling §3.2 (2 criteria) manual verified (Chunk 7 Task 7.1)
6. 143 tests (or more after migration) green (Chunk 6 Task 6.2 gate)
7. `make check-arch-invariants` passes (Chunk 6 Task 6.2)
8. Rust clippy zero new warnings (run `cargo clippy -p sim-runtime -p st3215-wire -p st3215-compat-bridge -p station_iface --all-targets` and confirm)
9. spec + plan + comparison table committed to main (Chunks 5 Task 5.1 + plan commits)
10. `sim_starting_point.md` reflects MVP-2 completion (Task 7.3)

For each, mark ✅ or ❌ and list any unmet items.

- [ ] **Step 2: Run the final verification commands**

```bash
# Tests
make sim-test 2>&1 | tail -5

# Clippy
cargo clippy -p sim-runtime -p st3215-wire -p st3215-compat-bridge -p station_iface --all-targets 2>&1 | grep -c '^warning:' || echo "0 warnings"

# Arch invariants
make check-arch-invariants

# Git log shows all MVP-2 commits
git log --oneline $(git merge-base HEAD main 2>/dev/null || git log --format=%H | tail -1)..HEAD | head -50
```

Record counts in `/tmp/mvp2_dod_summary.txt`.

- [ ] **Step 3: If any DoD item fails, STOP and fix**

If items 1-10 are not all ✅, MVP-2 is not complete. Return to the failing chunk and iterate.

- [ ] **Step 4: Commit a final "MVP-2 complete" note commit**

Only after all 10 DoD items pass:

```bash
# An empty commit marking the milestone is acceptable, but it's more useful
# to pair it with a small doc update (e.g., bumping a VERSION file if any).
# If no files need changing, skip this step — the git log itself is the record.

git log --oneline -20 > /tmp/mvp2_final_log.txt
```

Read `/tmp/mvp2_final_log.txt` and confirm the chain of commits from Chunks 1-7 is coherent.

---

### Task 7.5: MVP-2 completion summary

**Files:** None (documentation)

At the end of Chunk 7:

1. ✅ All 7 chunks executed to their gate
2. ✅ Manual Phase 2 smoke test passed (Task 7.1)
3. ✅ `sim-server/README.md` has MVP-2 checklist (Task 7.2)
4. ✅ `sim_starting_point.md` memory reflects MVP-2 done (Task 7.3)
5. ✅ Spec §3.4 DoD 10 items all ✅ (Task 7.4)
6. ✅ Final `make sim-test` + clippy clean
7. ✅ M1 physics debt from MVP-1 is resolved — Menagerie parameters + armature were the correct fix, gravity compensation is NOT required
8. ✅ `test_menagerie_walking_skeleton.py` remains green as permanent regression
9. ✅ URDF → MJCF pipeline is permanently retired (gen.py deleted)
10. ✅ ElRobot sim is ready for MVP-3 (env wrapper / policy training)

**MVP-2 is complete.** Subagent-driven-development or executing-plans handoff: report completion, offer to invoke `superpowers:finishing-a-development-branch` if this execution was run on a worktree.

