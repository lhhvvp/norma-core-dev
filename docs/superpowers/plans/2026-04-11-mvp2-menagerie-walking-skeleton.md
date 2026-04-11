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
- Number of `<joint>` elements (expected: 5-6 revolute + gripper)
- Presence of `<default>` block with `<joint armature="..." damping="..."/>` (REQUIRED for spec viability)
- Actuator type (expected: `<position>` for revolute + something for gripper)
- Gripper implementation: `<tendon>` + `<equality>`? `<weld>`? Or simple prismatic?
- Mesh path convention: relative to `assets/`?
- `<option>` block with timestep/gravity/integrator

Document findings in a scratch note. This info is needed for Chunk 4 (ElRobot MJCF construction).

**If no `armature` attribute is present anywhere in the default block**: STOP. The whole MVP-2 fork strategy relies on Menagerie having tuned armature values. Report to user and re-examine spec.

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

**Gate for Task 1.1:** All 6 steps pass. If any fail, STOP and report. Do not proceed to task 1.2.

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
