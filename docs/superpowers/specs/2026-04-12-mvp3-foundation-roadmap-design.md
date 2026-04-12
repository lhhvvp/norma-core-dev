# MVP-3 Engine Package Completion Roadmap — Design Spec

> **Type**: Lean roadmap spec (per brainstorming Approach A). Defines the chunk
> decomposition, dependencies, and boundaries of MVP-3 Engine Package Completion. Each chunk
> gets its own brainstorming → spec → plan → execute cycle when its turn comes.
> This spec is **not** a per-chunk implementation spec.

> **Naming history**: Originally drafted as "MVP-3 Foundation Roadmap" (commits
> `fde872c`/`93fcf15`/`a851354`). Renamed at codex review iteration 4 because
> codex correctly pointed out that A3 (`scene.xml` viewer wrapper) is upstream/
> ergonomics work, not foundation infrastructure. The 3-chunk content is
> unchanged; the framing now honestly says "package completion" (engine-tier
> robot package becomes self-contained, visually verifiable, and Norma-decoupled)
> rather than pretending all 3 chunks are foundation.

**Status**: draft, awaiting spec-document-reviewer + codex iter-2 + user review
**Date**: 2026-04-12
**Predecessor**: `docs/superpowers/specs/2026-04-12-mvp3-first-class-mjcf-design.md` (Chunk 0 spec)
**Predecessor commit**: `6ef605b` on `main` (Chunk 0 atomic restructure landed)
**Successor**: per-chunk specs (TBD, one per chunk after Chunk 1 brainstorming starts)

---

## 1. Goal & Success Criterion

**Milestone**: **MVP-3 Engine Package Completion** — finish what Chunk 0 started. Chunk 0
promoted `hardware/elrobot/simulation/` to a three-tier first-class structure
but only built the *skeleton* of the engine-tier robot package. MVP-3 Engine Package Completion
fills in the *contents* so the package is genuinely self-contained (Chunk 1),
visually verifiable through a Menagerie-style entry point (Chunk 2 — viewer
ergonomics, not foundation), and test-decoupled from `norma_sim` (Chunk 3).
Chunks 1 and 3 are foundation work; Chunk 2 is upstream/visualization
ergonomics that is naturally bundled because the package metadata files
(README/CHANGELOG/VERSION/robot.yaml) are already being touched in this
milestone.

### Starting state (HEAD `6ef605b`)

`mujoco/elrobot_follower/` exists but is partial:

- ✅ Skeleton present (`README.md`, `CHANGELOG.md`, `VERSION` 0.1.0, `LICENSE`,
  `robot.yaml`, `measurements/`, `tests/`)
- ❌ MJCF references `meshdir="../../assets"` — assets live outside the package
- ❌ `elrobot_follower.urdf` lives at `simulation/` not in the package
- ❌ No `scene.xml` wrapper — `mujoco.viewer` on `elrobot_follower.xml` shows
  the arm without floor or lights
- ❌ `test_elrobot_acceptance.py` (13 tests) lives in
  `software/sim-server/tests/integration/`, imports `norma_sim`, cannot run
  from a fresh checkout without `PYTHONPATH=software/sim-server`

### Success criterion (MVP-3 Engine Package Completion is "done" when)

1. `python3 -m pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -v`
   passes **without** `PYTHONPATH=software/sim-server` and includes the 13
   acceptance tests (currently in sim-server)
2. `python3 -m mujoco.viewer hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml`
   exists as a runnable wrapper with floor + lights (manual GUI verification
   optional in headless dev environments)
3. The package is **fully self-contained**:
   `cp -r mujoco/elrobot_follower /tmp/test && cd /tmp/test && pytest tests/`
   passes (every dependency lives inside the package directory)
4. `make sim-test` is fully green with no regressions (assertion: `passed ≥
   chunk_0_baseline_passed + 2`; the +2 comes from Chunk 2's
   `test_scene_loadable.py` and Chunk 3's net delta of +1, see Section 2
   inventory and Appendix A; do NOT hardcode any absolute count)
5. `mujoco/elrobot_follower/robot.yaml`'s `upstream.prerequisites` list moves
   from 1/5 done (Chunk 0) to **3/5 done** (Chunk 0 + MVP-3 Engine Package Completion):
   - ✅ `LICENSE file present (done at 0.1.0)` — done at Chunk 0
   - ✅ `Passing tests executable in isolation` — **done at MVP-3**
   - ✅ `scene.xml wrapper with lights/floor` — **done at MVP-3**
   - ❌ `sysid_complete == true` — needs hardware (deferred MVP-4+)
   - ❌ `Full CITATION.cff metadata` — deferred MVP-4+

### Why this scope (vs other MVP-3 framings considered)

**Four** other framings were considered and rejected during brainstorming
(the original three from session Q2 + one surfaced by codex iter-1 review):

- **Upstream-ready package (A1+A2+A3+A4 partial+A7 partial)**: requires hardware
  (sysID) and CITATION metadata. Deferred MVP-4.
- **Policy-training-ready (A5 LeRobot + A6 MJX)**: introduces JAX + LeRobot
  ecosystem dependencies. YAGNI for MVP-3.
- **Real-world-validated (A7 sysID-driven)**: needs hardware. Deferred MVP-4.
- **Self-contained-package, no viewer wrapper (A1+A2 only, drop A3)**: codex
  iter-1 surfaced this — it is the strictest "foundation only" framing because
  A3 is viewer/upstream ergonomics, not infrastructure. Rejected because: (a)
  the user explicitly chose "Foundation 全做 = A1+A2+A3" during brainstorming
  Q2; (b) Chunk 2 is small (~5 file ops, 1 new file + metadata bumps) and
  bundling it amortizes the README/CHANGELOG/VERSION/robot.yaml touch cost
  that all 3 chunks share; (c) `scene.xml` mirroring Menagerie convention is
  the cheapest meaningful step toward upstream parity, and Chunk 0's
  CHANGELOG already documented "no scene.xml wrapper" as a known limitation
  to be tracked. **The honest concession to codex's point**: this milestone
  is renamed "Engine Package Completion" rather than "Foundation" so the
  framing matches the actual content.

The chosen framing — Engine Package Completion = A2 (assets/urdf into
package, foundation) + A3 (scene.xml wrapper, ergonomics bundled) + A1 (test
decoupling, foundation) — is the natural continuation of Chunk 0's
organizational refactor and is the dominant prerequisite for any of the
other framings. After MVP-3 Engine Package Completion lands, all four MVP-4
directions remain open.

---

## 2. Chunk Inventory

| | **Chunk 1** | **Chunk 2** | **Chunk 3** |
|---|---|---|---|
| **Codename** | A2 — Assets+URDF Move | A3 — Scene Wrapper | A1 — Test Decoupling |
| **Core action** | Move `assets/` (19 STL) + `elrobot_follower.urdf` into the package; simplify `meshdir`; update all dependent paths | Add Menagerie-style `scene.xml` (lights + floor + `<include>`) | Rewrite `test_elrobot_acceptance.py` (13 tests) as pure-mujoco; move 13 into package; **keep 1 sentinel test in `software/sim-server/tests/integration/`** that exercises the `scene.yaml → load_manifest → MuJoCoWorld` pipeline (see Section 5 for rationale) |
| **Size** | Large (~25-35 file ops) | Small (1-2 new files) | Medium (1 large rewrite + 1 sentinel + cleanup) |
| **Risk** | High (most file movement) | Low (pure additive) | Medium-high (behavioral rewrite + coverage equivalence audit) |
| **VERSION bump** | 0.1.0 → 0.2.0 (minor) | 0.2.0 → 0.2.1 (patch) | 0.2.1 → 0.2.2 (patch — decided per codex consult) |
| **Commit topology** | 1 atomic commit | 1 atomic commit | **2 commits (split per codex consult)**: (1) add new pure-mujoco acceptance suite + sentinel, run green; (2) delete old sim-server file, clean fixtures/README/CHANGELOG/VERSION/robot.yaml |
| **Test count delta (engine-tier in repo)** | +0 | +1 (scene loadable smoke) | +13 (acceptance suite moved into package) |
| **Test count delta (sim-server alone)** | +0 | +0 | -13 (acceptance suite removed) +1 (manifest-pipeline sentinel added) = -12 net |
| **Test count delta (`make sim-test` total)** | +0 | +1 | -12 + 13 = +1 |
| **Post-chunk `make sim-test`** | baseline + 0 | baseline + 1 | baseline + 2 |
| **Estimated plan length** | 1000-1400 lines | 300-500 lines | 700-1000 lines |
| **Prerequisite** | Chunk 0 (`6ef605b`) | Chunk 1 landed | Chunk 1 landed (hard); Chunk 2 landed (soft — see Section 6; the prior "soft + count-hard" framing was an artifact of writing absolute test counts and disappears under baseline-relative deltas) |

**Why baseline-relative deltas (codex iter-1 finding)**: absolute counts like
"90 passed" depend on (a) current dev env (no `mujoco.mjx` installed → 1 test
skipped) and (b) no concurrent test additions on `main`. Both can change.
Plans MUST verify deltas against the actual baseline measured at chunk start
(`make sim-test` immediately after pulling, before any chunk work) — not
against the absolute number written here. The "baseline" is whatever
`make sim-test` reports the moment Phase A pre-flight runs.

**Total MVP-3 Engine Package Completion**: 4 atomic commits (Chunk 1: 1, Chunk 2: 1, Chunk 3: 2),
~40-50 file ops, ~2000-3000 plan lines combined. Smaller than MVP-2 (4216
lines / 7 chunks) because the work is mostly organizational — Chunks 1 and 2
are pure file movement and additive content; **Chunk 3 is a behavioral test
rewrite, not pure organization** (codex iter-1 correction — earlier draft
incorrectly claimed all work was "purely organizational").

---

## 3. Chunk 1 — A2: Assets + URDF Move

### Scope

Move `hardware/elrobot/simulation/assets/` (19 STL meshes) and
`hardware/elrobot/simulation/elrobot_follower.urdf` **into**
`hardware/elrobot/simulation/mujoco/elrobot_follower/`. Simplify the MJCF's
`meshdir="../../assets"` to `meshdir="assets"`. Update every dependent path
(test fixtures, sim-server fixtures, README references, conftest paths).
Bump VERSION to 0.2.0 (minor — structural package layout change). This
fulfills Chunk 0's "future chunk will move assets" promise.

### Prerequisites

- HEAD = `6ef605b` (Chunk 0) or newer
- `make sim-test` baseline measured immediately at Phase A (capture as
  `BASELINE_PASSED` and `BASELINE_SKIPPED` — these are the reference for all
  delta assertions in this chunk; do NOT hardcode "90 passed" in plan)
- `make check-arch-invariants` green
- `git status` clean (except untracked `MUJOCO_LOG.TXT`, `station_data/`)

### File operations (high-level)

**Move (`git mv`)**:
- `hardware/elrobot/simulation/assets/` → `hardware/elrobot/simulation/mujoco/elrobot_follower/assets/`
  (19 STL files individually, not bulk directory rename — `git mv` per file
  gives clearest history)
- `hardware/elrobot/simulation/elrobot_follower.urdf` →
  `hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.urdf`

**Edit (in-place after move)**:
- `mujoco/elrobot_follower/elrobot_follower.xml`: `meshdir="../../assets"` → `meshdir="assets"`
- `mujoco/elrobot_follower/tests/test_urdf_parity.py`: `urdf_path` fixture
  resolution from `parent.parent.parent.parent` (4 levels) to `parent.parent`
  (2 levels)
- `mujoco/elrobot_follower/README.md`: update "Structure" section + relationship
  notes
- `mujoco/elrobot_follower/CHANGELOG.md`: add `[0.2.0] — 2026-04-12` entry
- `mujoco/elrobot_follower/VERSION`: `0.1.0` → `0.2.0`
- `mujoco/elrobot_follower/robot.yaml`: bump `version.current`; update prereq
  state

**Path updates in dependent files** (must be enumerated by `grep -rn` in plan
Phase A — see Section 7 (α) lesson):
- Possible: `software/sim-server/tests/conftest.py` (if any URDF fixture exists)
- Possible: `software/sim-server/tests/integration/test_elrobot_acceptance.py`
  (if URDF is referenced directly — Chunk 3 will rewrite this file anyway, but
  Chunk 1 must not leave it in a broken intermediate state)
- Possible: any `hardware/elrobot/simulation/manifests/norma/*.scene.yaml`
  references
- Anything else `grep -rn 'simulation/assets\|simulation/elrobot_follower\.urdf'
  software/ hardware/ Makefile docs/` finds (excluding `docs/superpowers/`
  historical references and `vendor/menagerie/VENDOR.md`)

### Folded-in Chunk 0 余债 (see Section 7)

- Item 1: `tests/test_urdf_parity.py:50-65` unused `elrobot_mjcf_path` parameter
  — fix by adding an `assert model.nu == 8` cross-check (upgrade lint smell to
  belt-and-suspenders)
- Item 2: `tests/test_urdf_parity.py:53` "7 revolute + 1 gripper primary"
  docstring drift — reword to "8 revolute (7 arm + 1 gripper primary)"
- Item 3: `CHANGELOG.md:96` "MVP-3 Chunk 0 commit: TBD" — backfill with `6ef605b`

### Success criteria (all assertions are baseline-relative deltas)

1. `python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml'); print(m.nu, m.neq)"`
   → `nu=8 neq=2`
2. `make sim-test` → **`BASELINE_PASSED` passed, `BASELINE_SKIPPED` skipped**
   (delta = +0; no test added, no test removed; if measured count drifts
   from baseline → investigate)
3. `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -v`
   (no PYTHONPATH) → 4 passed + 1 skipped (mjx) — these counts are absolute
   because the engine-tier package's own test count is fully owned by this
   spec (no external main commits add tests here)
4. `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py -v`
   → 2 **PASSED** (counted explicitly via `grep -c PASSED`, not just "no
   failures"). **Chunk 1 also upgrades the URDF fixture from
   `pytest.skip("URDF not found")` to `raise FileNotFoundError` (or
   `assert urdf_path.exists()`) because URDF is mandatory content of the
   package after this chunk** — see Section 9 risk #6 reframe
5. **Self-containment**: `cp -r hardware/elrobot/simulation/mujoco/elrobot_follower /tmp/elrobot-test && cd /tmp/elrobot-test && python3 -m pytest tests/ -v`
   → 4 passed + 1 skipped (mjx) — **first chunk where this command is
   meaningful** (Chunk 0 baseline cp -r produces 0 passed / 2 failed because
   `meshdir="../../assets"` cannot resolve in `/tmp`)
6. Phase G.8 grep: `grep -rn 'hardware/elrobot/simulation/assets\|hardware/elrobot/simulation/elrobot_follower\.urdf' software/ hardware/ Makefile docs/`
   (with the same exclusions as Chunk 0) → empty
7. `git status` clean
8. `make check-arch-invariants` green

### Risks (Chunk-1-specific)

1. **Hidden URDF references** (high): URDF may be referenced from sim-server
   integration tests, calibration scripts, or `manifests/norma/*.scene.yaml`.
   Mitigation: Chunk 1 plan **must** start with a `grep -rn` exhaustive scan
   in Phase A to enumerate every reference into Phase F's edit list. This is
   the first practical application of the Section 7 (α) "grep first" lesson.
2. **STL bulk move**: 19 individual `git mv` operations. Risk of mid-loop
   error leaving partial state. Mitigation: enumerate per-file `git mv`
   commands explicitly in plan (matching Chunk 0's per-file pattern, not a
   loop), verify each rename via `git status` afterward.
3. **`meshdir` is NOT a compiler-level edge case** (codex iter-1 reframe):
   MuJoCo resolves `meshdir` relative to the *containing MJCF file*, so
   `meshdir="assets"` and `meshdir="../../assets"` use identical resolution
   logic — there is no compiler difference. The actual risks are:
   (a) introducing a *second* `<compiler meshdir=...>` somewhere
   (e.g., in the future `scene.xml` of Chunk 2) that overrides the main
   MJCF's; (b) `<include>` chains that change the effective base directory
   silently. Chunk 1 addresses (a) by **NOT writing any `<compiler>` block in
   any new file** and (b) by verifying via actual compile (`mujoco.MjModel.from_xml_path`)
   not just text inspection.
4. **`test_urdf_parity` skip-vs-fail trap → upgrade to fail**: the URDF
   fixture currently uses `pytest.skip("URDF not found")` not `pytest.fail`.
   This was correct for Chunk 0 (URDF was outside the package, optional). After
   Chunk 1 it is **wrong**: URDF is now mandatory content of the package, so a
   missing URDF must be a hard failure, not a silent skip. **Chunk 1's plan
   includes upgrading this fixture from `pytest.skip` to a hard error**
   (either `raise FileNotFoundError` or `assert urdf_path.exists(), ...`).
   This eliminates the off-by-one silent-skip class of bug going forward.
   Codex iter-1 explicitly recommended this upgrade.

### Boundary

- ❌ Scene wrapper (`scene.xml`) — Chunk 2
- ❌ Rewrite `test_elrobot_acceptance.py` — Chunk 3
- ❌ Any physics parameter changes (`<default>`, `<contact>`, `<actuator>`)
- ❌ Any Rust changes
- ❌ `vendor/menagerie/` changes
- ❌ Changes outside the path-migration scope

### VERSION bump policy

`0.1.0` → `0.2.0` (minor). Reason: structural change to package layout —
consumers referencing mesh paths must update their references. Per
CHANGELOG.md's "minor for structural" rule.

---

## 4. Chunk 2 — A3: Scene Wrapper

### Scope

Add `hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml`: a
Menagerie-style MJCF wrapper containing `<visual>`, `<statistic>`, `<asset>`
(skybox + checker groundplane), `<worldbody>` (light + floor), and
`<include file="elrobot_follower.xml"/>`. Running
`python3 -m mujoco.viewer .../scene.xml` opens an ElRobot visualization with
floor and lights — fulfilling Chunk 0's "Known limitations: no scene.xml
wrapper" deferral.

### Prerequisites

- Chunk 1 (`A2 — Assets+URDF Move`) landed (**soft prerequisite** — Chunk 2
  *content* doesn't depend on Chunk 1, but the cp -r self-containment success
  criterion #5 below requires assets to already be in the package; running
  Chunk 2 before Chunk 1 would mean re-running visual validation after Chunk 1)
- Package is self-contained at this point (assets + URDF in package, courtesy
  of Chunk 1)
- `make sim-test` baseline measured immediately at Phase A (capture as
  `BASELINE_PASSED` and `BASELINE_SKIPPED` — these reference Chunk 1's
  end-state, used for delta assertions)

### File operations

**Create**:
- `mujoco/elrobot_follower/scene.xml` — new file, ~30-50 lines, Menagerie style.
  **Must NOT contain a `<compiler>` block** — that would override the main
  MJCF's `meshdir` setting via `<include>` namespace merge (codex iter-1
  reframed risk #3)
- `mujoco/elrobot_follower/tests/test_scene_loadable.py` — 1 new smoke test
  verifying `mujoco.MjModel.from_xml_path('scene.xml')` compiles. Assertions:
  (a) `m.nu == 8` (actuator count matches main MJCF — cross-check that
  `<include>` namespace merging worked); (b)
  `mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor") >= 0` (the floor
  geom exists by name); (c) `mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, "<light_name>") >= 0`
  for whatever light name we settle on. **Do NOT use `m.ngeom == <count>`**
  — codex iter-1 flagged this as fragile (codex 7-pt). Name-based
  assertions are stable across future geometry additions.

**Edit**:
- `mujoco/elrobot_follower/README.md`: add `scene.xml` to Structure section;
  add a "How to view" snippet (`python3 -m mujoco.viewer .../scene.xml`)
- `mujoco/elrobot_follower/CHANGELOG.md`: add `[0.2.1] — 2026-04-12` entry
- `mujoco/elrobot_follower/VERSION`: `0.2.0` → `0.2.1`
- `mujoco/elrobot_follower/robot.yaml`: bump `version.current`; mark
  `scene.xml wrapper` prereq as done
- **Polish (codex iter-1 surfaced)**:
  `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml:11-12`
  comment currently reads `# No scene_extras — the MJCF has its own lighting/floor setup.`
  This is a stale comment from MVP-2 Chunk 5 — `elrobot_follower.xml` does
  NOT have lighting/floor (verified by reading the MJCF). Chunk 2 fixes the
  comment to reference the new `scene.xml` wrapper instead, OR removes the
  misleading sentence entirely. Single-line edit, naturally folded into Chunk 2

**No moves, no deletions, no other path updates** — Chunk 2 is mostly
additive (1 new MJCF + 1 new test) plus the comment-polish edit above.

### `scene.xml` content design

Reference: `vendor/menagerie/trs_so_arm100/scene.xml` (already in repo, the
canonical Menagerie convention). Borrow the `<visual>`, `<statistic>`, and
`<asset>` blocks (builtin textures, no binary asset files). Worldbody declares
its own light and floor; the main MJCF gets included via `<include>`.

Approximate skeleton (final form determined in Chunk 2 brainstorming):

```xml
<mujoco model="elrobot_follower scene">
  <include file="elrobot_follower.xml"/>
  <statistic center="0 0 0.1" extent="0.5"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" .../>
    <texture type="2d" name="groundplane" builtin="checker" .../>
    <material name="groundplane" texture="groundplane" .../>
  </asset>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
```

### Success criteria (baseline-relative deltas)

1. `python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml'); print(f'nu={m.nu}'); import sys; sys.exit(0 if m.nu == 8 and mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor') >= 0 else 1)"`
   → exits 0 (`nu == 8` matching `elrobot_follower.xml` AND `floor` geom
   exists by name — name-based, not count-based per codex iter-1 reframe)
2. `pytest .../tests/test_scene_loadable.py -v` → 1 PASSED
3. `make sim-test` → **`BASELINE_PASSED + 1` passed, `BASELINE_SKIPPED` skipped**
   (delta = +1 from `test_scene_loadable.py`; baseline is Chunk 1 end-state)
4. `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -v`
   (no PYTHONPATH) → 5 passed + 1 skipped (engine-tier package's own tests
   are absolute; +1 vs Chunk 1 end-state)
5. **Self-containment** (`cp -r /tmp` pattern from Chunk 1) → 5 passed + 1 skipped
6. Optional GUI gate: `python3 -m mujoco.viewer .../scene.xml` opens visualization
   (skipped in headless dev environments — see Open Decision U3)
7. `git status` clean; `make check-arch-invariants` green

### Risks (Chunk-2-specific)

1. **`<include>` namespace merging + `<compiler>` collision**: MuJoCo's
   `<include>` is textual merge, not import. Two failure modes:
   (a) `scene.xml` declaring its own `<default class="elrobot">` or reusing
   an asset name → namespace conflicts;
   (b) `scene.xml` declaring its own `<compiler meshdir=...>` → silently
   overrides the main MJCF's meshdir during merge, causing mesh resolution
   failures (codex iter-1 reframe of original meshdir risk).
   Mitigation: `scene.xml` only declares `<visual>`, `<statistic>`,
   `<asset>` (with non-colliding names), `<worldbody>` (light + floor
   only), `<include>`. **No `<compiler>` block. No `<default>` block.**
2. **Borrowed Menagerie textures**: using builtin textures (`builtin="checker"`)
   avoids binary assets and licensing concerns. Don't add a real PNG.
3. **Smoke test must assert by name not by count** (codex iter-1): the smoke
   test must use `mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor") >= 0`
   not `m.ngeom == <expected_count>`. Count-based assertions are fragile to
   future geometry additions. The plan author MUST use name-based assertions
   for any element added by `scene.xml` (floor, light, groundplane material).

### Boundary

- ❌ Rewrite `test_elrobot_acceptance.py` — Chunk 3
- ❌ Camera presets beyond scene.xml defaults
- ❌ Headless rendering / image capture / mp4
- ❌ Any physics changes
- ❌ Adding mesh-based geoms (the scene.xml is purely visual scaffolding)

### VERSION bump policy

`0.2.0` → `0.2.1` (patch). Reason: purely additive; no API or physics change.
Consumers don't need to do anything. Optional new entry point. (Open
Decision U1 considered `0.3.0` minor as a milestone marker but resolved
to patch per codex iter-1 — semver should signal consumer-visible changes,
not milestone progress.)

---

## 5. Chunk 3 — A1: Test Decoupling

### Scope

Rewrite `software/sim-server/tests/integration/test_elrobot_acceptance.py`
(13 physics-acceptance tests) to use **raw `mujoco.MjModel` / `mujoco.MjData`**
APIs directly instead of going through `MuJoCoWorld.from_manifest_path(...)`.
Move the rewritten suite to
`hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_elrobot_acceptance.py`.

**Coverage equivalence is at the physics layer, NOT at the manifest layer.**
The original tests entered through `scene.yaml → load_manifest → MuJoCoWorld`
which exercises (a) scene-yaml parsing, (b) `actuator_annotations` consistency
validation, (c) `mjcf_path` resolution, (d) mjcf↔manifest actuator name
binding. The pure-mujoco rewrite enters through `mujoco.MjModel.from_xml_path(...)`
which **skips all manifest-layer work entirely**. To prevent silent coverage
loss (codex iter-1 critical finding), Chunk 3 ALSO creates **1 sentinel test
in `software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py`**
that exercises the full manifest pipeline:

```python
def test_elrobot_manifest_pipeline_sentinel(elrobot_scene_yaml):
    """Smoke: scene.yaml → load_manifest → MuJoCoWorld pipeline still works
    end-to-end for the elrobot package. Catches manifest-layer regressions
    that the engine-tier acceptance suite (now in mujoco/elrobot_follower/tests/)
    cannot see — specifically: scene.yaml parsing, world_name binding,
    actuator_annotations consistency check, and the GRIPPER_PARALLEL
    capability assignment for act_motor_08."""
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    mujoco.mj_step(world.model, world.data)

    # Manifest parsing actually happened (not just MJCF load)
    assert world.manifest.world_name == "elrobot_follower"

    # MJCF compile + lookup cache built correctly
    assert world.model.nu == 8

    # actuator_annotations were applied — without these explicit annotations,
    # load_manifest auto-synthesizes act_motor_08 as a plain
    # REVOLUTE_POSITION (manifest.py:187), so this is the assertion that
    # fails if the gripper annotation is silently dropped from the
    # scene.yaml or from load_manifest's annotation merge.
    gripper = world.actuator_by_mjcf_name("act_motor_08")
    assert gripper is not None
    assert gripper.capability.kind == "GRIPPER_PARALLEL"
    assert gripper.gripper is not None  # gripper-specific metadata block
```

The 13 physics tests move; the manifest-layer coverage is preserved by the
sentinel. This is the **only honest way** to do the rewrite. Pre-codex draft
claimed "semantic equivalence" — that claim was wrong because the test
*entry points* are not equivalent.

This is the MVP-3 Engine Package Completion capstone — afterwards
`mujoco/elrobot_follower/` is a complete, fresh-checkout-runnable robot
package, AND the sim-server still validates the manifest pipeline.

### Prerequisites

- Chunk 1 (`A2 — Assets+URDF Move`) landed (**hard prerequisite** — the
  cp -r /tmp self-containment success criterion below requires assets+URDF
  inside the package; without Chunk 1, that test fails)
- Chunk 2 (`A3 — Scene Wrapper`) landed (**soft prerequisite** — Chunk 3's
  rewrite doesn't reference scene.xml at all; the prior draft claimed a
  "count-hard" dependency but codex iter-1 correctly pointed out that this
  was a self-imposed dependency from writing absolute test counts. With
  baseline-relative deltas, this dependency disappears entirely. Chunk 2 is
  recommended-before-Chunk-3 only because the package "feels complete" with
  scene.xml in place during Chunk 3's manifest-vs-engine coverage audit)
- `make sim-test` baseline measured immediately at Phase A (capture as
  `BASELINE_PASSED` and `BASELINE_SKIPPED` — these reference Chunk 2's
  end-state, used for delta assertions in this chunk)
- Package is self-contained (assets + URDF + scene.xml all in place)

### File operations (split into 2 commits per codex iter-1 U2)

**Commit 1: Add new pure-mujoco acceptance suite + manifest sentinel, both green**

- Create `mujoco/elrobot_follower/tests/test_elrobot_acceptance.py` — new
  file containing the 13 rewritten physics tests using raw mujoco APIs.
  Run the suite green before committing.
- Create `software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py`
  — new file with 1 test that exercises the full
  `scene.yaml → load_manifest → MuJoCoWorld → mj_step` pipeline (see Scope
  for the test body sketch). Run green before committing.
- Possibly add a shared fixture in
  `mujoco/elrobot_follower/tests/conftest.py` (e.g., `elrobot_model` or
  `elrobot_data_at_rest`) to share boilerplate across the 13 tests if
  rewrite-time inspection finds duplication. Discretionary.
- The original `software/sim-server/tests/integration/test_elrobot_acceptance.py`
  is **untouched** in commit 1. After commit 1, both old and new acceptance
  suites exist and pass — total `make sim-test` shows
  `BASELINE_PASSED + 14` (`+13` new acceptance + `+1` sentinel).

**Commit 2: Delete the old sim-server file + clean up dead fixtures + bump metadata**

- Delete `software/sim-server/tests/integration/test_elrobot_acceptance.py`
  (the old norma_sim-coupled version)
- `software/sim-server/tests/conftest.py`: check if `elrobot_mjcf_path` /
  `elrobot_scene_yaml` fixtures have remaining consumers via
  `grep -rn 'elrobot_mjcf_path\|elrobot_scene_yaml' software/sim-server/tests/`.
  Keep them only if other tests use them (the new sentinel uses
  `elrobot_scene_yaml`, so that fixture stays). Delete only the dead ones.
- `mujoco/elrobot_follower/CHANGELOG.md`: add `[0.2.2]` entry covering both
  the rewrite + the move
- `mujoco/elrobot_follower/VERSION`: `0.2.1` → `0.2.2`
- `mujoco/elrobot_follower/robot.yaml`: bump `version.current`; mark
  `Passing tests executable in isolation` prereq as done
- `mujoco/elrobot_follower/README.md`: rewrite "Relationship to NormaCore"
  section — engine-tier acceptance tests live in package; sim-server keeps
  the manifest-pipeline sentinel and Norma-specific integration tests
- Phase G.8 grep: `grep -rn 'test_elrobot_acceptance' software/ hardware/ Makefile docs/`
  (with same exclusions as Chunk 0) → only matches the **new** package
  location, not the deleted sim-server location

**Why split** (codex iter-1 U2 reasoning): commit 1 is a pure additive
change (easy review); commit 2 is a pure deletion (easy review). The split
makes it auditable that the 13 tests' assertions are equivalent because the
reviewer can run BOTH the old and new suites side-by-side at HEAD-of-commit-1.
A single atomic commit would conflate "new tests added" with "old tests
deleted" in one diff, making the equivalence audit much harder.

### Folded-in Chunk 0 余债

- Item 4: `tests/test_mimic_gripper.py:1` `★ P0` unicode glyph → `[P0]` ASCII
  (see Open Decision U5 for alternative — defer to upstream prep)

### Rewrite strategy (corrected per codex iter-1 — original mapping table had fictional APIs)

The actual current `MuJoCoWorld` API surface (verified by reading
`software/sim-server/norma_sim/world/model.py:20-79`):

```python
class MuJoCoWorld:
    def __init__(self, manifest: WorldManifest) -> None:
        self.manifest = manifest
        self.model = mujoco.MjModel.from_xml_path(str(manifest.mjcf_path))
        self.data = mujoco.MjData(self.model)
        self.lock = threading.Lock()
        self._build_lookups()

    @classmethod
    def from_manifest_path(cls, manifest_path) -> "MuJoCoWorld": ...

    def actuator_id_for(self, mjcf_actuator: str) -> Optional[int]: ...
    def joint_qposadr_for(self, mjcf_joint: str) -> Optional[int]: ...
    def actuator_by_mjcf_name(self, mjcf_actuator: str) -> Optional[ActuatorManifest]: ...

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)
```

That is the **complete** public API surface used by `test_elrobot_acceptance.py`.
The `__init__` is thin: load model, build MjData, build lookup caches. No
implicit physics setup, no ctrlrange validation, no qpos preset. The
"implicit setup" risk in the prior draft was misdirected — the real
implicit-setup work is in `load_manifest()` (`software/sim-server/norma_sim/world/manifest.py:94+`),
which the rewrite avoids entirely (and which the new sentinel still
exercises).

**Real mapping table** (verified against actual test usage in
`software/sim-server/tests/integration/test_elrobot_acceptance.py:39-100`):

| Original (norma_sim entry) | Rewrite (raw mujoco) |
|---|---|
| `from norma_sim.world.model import MuJoCoWorld` | `import mujoco` |
| `world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)` | `model = mujoco.MjModel.from_xml_path(mjcf_path); data = mujoco.MjData(model)` |
| `world.model` | `model` |
| `world.data` | `data` |
| `world.data.qpos` / `world.data.qvel` / `world.data.ctrl[:]` / `world.data.ncon` / `world.data.qM` | identical (`world.data` is just a wrapper around the same `mujoco.MjData`; all field access is unchanged after dropping the `world.` prefix) |
| `world.step()` | `mujoco.mj_step(model, data)` |
| `mujoco.mj_forward(world.model, world.data)` | `mujoco.mj_forward(model, data)` |
| `mujoco.mj_fullM(world.model, M, world.data.qM)` | `mujoco.mj_fullM(model, M, data.qM)` |
| `world.model.actuator_ctrlrange[:, 0]` / `world.model.dof_armature[i]` / `world.model.nv` / etc. | identical (`world.model` is just a wrapper around the same `mujoco.MjModel`; all field access is unchanged) |

**Critical observation** (codex iter-1): the 13 existing tests are already
~95% raw-mujoco-compatible. They never call `actuator_id_for()`, never call
`actuator_by_mjcf_name()`, never use `world.lock`, and never use the
manifest object. The rewrite is mechanically just "drop the `world.`
wrapper and replace `MuJoCoWorld.from_manifest_path(scene_yaml)` with
direct `MjModel.from_xml_path(mjcf_path) + MjData(model)`". This is much
smaller than the prior draft suggested.

**Naming convention**: the engine-tier tests use **`act_motor_XX` (mjcf
names) exclusively**, never `rev_motor_XX` (Norma client_id). The 13
existing tests do NOT use client_id at all (they index `model.actuator_*`
arrays positionally), so this is automatic.

**Fictional APIs that appeared in the previous draft** (do NOT exist; do
NOT use): `world.step(dt=...)`, `world.set_ctrl(ctrl_dict)`, `world.qpos`,
`world.qvel`, `world.actuator_by_client_id(...)`. These were errors in the
original draft surfaced by codex iter-1 fact-checking against
`norma_sim/world/model.py`.

### Success criteria (baseline-relative deltas, measured at end of commit 2)

1. **Physics-layer coverage equivalence**: 13 test functions exist in the
   new package location; each one's assertions are semantically equivalent
   to the original physics behavior (manual side-by-side review during
   spec-compliance review — at HEAD-of-commit-1, both old and new suites
   exist and can be diff-compared directly)
2. **Manifest-layer coverage preserved by sentinel**: the new
   `software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py`
   exists and passes; it exercises the full
   `scene.yaml → load_manifest → MuJoCoWorld → mj_step` pipeline
3. `pytest .../mujoco/elrobot_follower/tests/test_elrobot_acceptance.py -v`
   (no PYTHONPATH) → 13 passed
4. `grep -c 'norma_sim' .../mujoco/elrobot_follower/tests/test_elrobot_acceptance.py`
   → 0
5. `make sim-test` → **`BASELINE_PASSED + 1` passed, `BASELINE_SKIPPED` skipped**
   (delta math: -13 from removing old sim-server acceptance, +13 from new
   package acceptance, +1 from new sentinel = net **+1**; baseline is
   Chunk 2 end-state)
6. **Full self-containment**:
   `cp -r mujoco/elrobot_follower /tmp/elrobot-test && cd /tmp/elrobot-test && pytest tests/ -v`
   → 18 passed + 1 skipped (4 from Chunk 0 mimic_gripper + urdf_parity, 1
   from Chunk 2 scene_loadable, 13 from Chunk 3 acceptance, 1 mjx skip)
7. sim-server cleanup: old
   `software/sim-server/tests/integration/test_elrobot_acceptance.py` no
   longer exists; new
   `software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py`
   does exist; `pytest software/sim-server/tests/ -q` shows
   **(prior sim-server baseline) − 13 + 1 = (prior sim-server baseline) − 12**
   (engine-tier moved out, sentinel added in)
8. `make check-arch-invariants` green
9. Phase G.8 grep:
   `grep -rn 'test_elrobot_acceptance' software/ hardware/ Makefile docs/`
   (with the same exclusions as Chunk 0 — `docs/superpowers/.*2026-04-1[012]`
   and `vendor/menagerie/VENDOR.md`) → only matches the **new** package
   path, not the deleted sim-server path or any other stale reference
10. `git status` clean

### Risks (Chunk-3-specific, reframed per codex iter-1)

1. **Manifest-layer coverage drop** (critical): the original tests enter
   through `scene.yaml → load_manifest → MuJoCoWorld`, exercising
   manifest parsing + actuator_annotations validation + mjcf_path
   resolution. The pure-mujoco rewrite skips all of that. Mitigation:
   the **new sentinel test in `software/sim-server/tests/integration/`**
   covers the manifest pipeline end-to-end. Spec-compliance review must
   verify the sentinel exists and exercises the full pipeline (not just
   imports). The prior draft missed this entirely and claimed "semantic
   equivalence" — that claim was wrong because the entry points are not
   equivalent.
2. **Physics-layer coverage reduction** (critical): even at the physics
   layer, if a `norma_sim`-specific helper has no raw-mujoco equivalent,
   the rewrite may silently drop the assertion. Mitigation:
   spec-compliance review must do side-by-side reading of original vs
   new for every test function at HEAD-of-commit-1 (the split commit
   topology lets reviewer have both files in tree simultaneously) and
   verify assertion count + assertion strength equivalence per test.
   Codex iter-1 noted that 13 tests are already ~95% raw-mujoco-compatible,
   so this risk is lower than feared but still requires the audit.
3. **Stale references after delete**: scripts/CI YAML/Makefile may
   reference the old test path. Mitigation: Phase G.8 grep pattern in
   commit 2's verification.
4. **Dead fixture cleanup**: must `grep` before deleting `elrobot_*`
   fixtures from sim-server conftest. Note that the new sentinel uses
   `elrobot_scene_yaml`, so that fixture must be **kept**. Only fixtures
   with zero remaining consumers may be deleted.
5. **Verbosity blowup**: raw mujoco is slightly more verbose than the
   `world.` wrapper, but not by a meaningful amount. Codex iter-1 verified
   that the existing 13 tests already use `world.model` / `world.data` /
   `world.step()` patterns, so the rewrite is mostly mechanical
   `world.` deletion. Estimated 13 tests × 30 lines = ~400 lines original;
   rewrite likely 400-500 lines (not 800-1200 as the prior draft feared).
6. **`__init__` is not the risk source** (codex iter-1 correction): the
   prior draft told the implementer to "read `MuJoCoWorld.__init__` and
   list implicit setup". `__init__` is thin (load model, build MjData,
   build lookups). The real implicit work is in `load_manifest()` at
   `software/sim-server/norma_sim/world/manifest.py:94+`, which the
   rewrite intentionally avoids and the sentinel preserves. Plan should
   redirect the "read source" step toward `load_manifest` if it wants to
   document what the sentinel is protecting against, OR drop the step
   entirely since the sentinel makes the manifest-layer coverage explicit.

### Boundary

- ❌ Any physics parameter changes
- ❌ New acceptance test functionality (rewrite-only; new coverage waits for
  later chunks)
- ❌ Changes to other sim-server tests (test_full_loop, test_menagerie_walking_skeleton,
  test_model — leave alone)
- ❌ Changes to `norma_sim` library (any norma_sim bug discovered during
  rewrite is logged to memory for MVP-4, not fixed in Chunk 3)
- ❌ Deleting sim-server fixtures that have remaining consumers
- ❌ MJCF / scene.xml changes
- ❌ `robot.yaml` schema redesign (Item 5 — defer MVP-4)

### VERSION bump policy

`0.2.1` → `0.2.2` (patch). Reason: internal test infrastructure change; no
API, no physics, no consumer impact. **Decided per codex iter-1**: do NOT
use semver to mark milestone — use it to signal consumer-visible change
levels. There is no consumer-visible change here.

---

## 6. Cross-chunk Dependencies & Ordering Rationale

### Dependency graph

```
Chunk 0 (✅ 6ef605b)
   │
   ├─→ Chunk 1 (A2: assets+urdf)         [hard: unlocks self-containment]
   │       │
   │       ├─→ Chunk 2 (A3: scene.xml)    [soft: technically reversible]
   │       │       │
   │       │       └─→ Chunk 3 (A1: test) [soft]
   │       │
   │       └────────→ Chunk 3 (A1: test)  [hard: self-containment requires assets in package]
   │
   └────────────────→ Chunk 3 (A1: test)  [no direct dependency]
```

**Definitions** (used throughout this section and Sections 3-5 prerequisites):
- **hard prerequisite** = a chunk's success criterion fails without it; the
  chunk cannot prove "done" until the prereq lands
- **soft prerequisite** = the chunk can technically execute without it, but
  reverse-ordering produces an intermediate state that delays a
  cross-chunk invariant (typically the cp -r self-containment audit) and
  forces re-validation later

The **only true hard dependency** is Chunk 1 → Chunk 3 (in the content
sense — Chunk 3's cp -r self-containment requires assets+URDF in package).
Chunk 2 in the middle is logical-order optimization, not technical
necessity. The Chunk 2 → Chunk 3 edge is fully **soft** when success
criteria are written as baseline-relative deltas — the prior draft labeled
it "soft + hard combination" because it wrote absolute test counts (`18+1s`)
that artificially bound Chunk 3's gate to Chunk 2's smoke test being
present. With baseline-relative deltas (codex iter-1 fix), this artifact
disappears.

We chose A2 → A3 → A1 because:

- A2 first unlocks package self-containment, the precondition for A3 and A1
- A3 second is a cheap sanity check — `mujoco.viewer scene.xml` after A2
  visually validates the move
- A1 last is the largest behavioral change; it benefits from running on a
  fully-stabilized package

**A2 → A1 → A3 is also viable** (codex iter-1 explicitly noted): it puts
both foundation-class chunks first (A2 + A1) and saves the ergonomics
chunk (A3) for last as a quick sweetener. Trade-off vs A2 → A3 → A1: A1
is done without the visual-sanity tool (`mujoco.viewer scene.xml`)
available, so any Chunk 1 mesh-resolution surprise is harder to spot
during the test rewrite phase. We retain A2 → A3 → A1 for that reason,
but explicitly acknowledge A2 → A1 → A3 as the codex-recommended
"foundation-purity" alternative.

**A2+A3 merge alternative considered and rejected**: codex iter-1 floated
merging Chunks 1 and 2 into a single "package self-containment + visual
wrapper" mega-chunk, with A1 alone as Chunk 2. Rejected because: (a) A2
is already a large chunk (~25-35 file ops, on par with Chunk 0); bundling
A3 risks pushing it past the comfortable atomic-commit size; (b) splitting
the metadata bumps (VERSION/CHANGELOG/robot.yaml) per chunk is cheap and
gives finer-grained rollback; (c) the user explicitly chose "3 chunks
total" during brainstorming Q4. A2+A3 merge would produce 2 chunks total,
contradicting that choice.

### Why reverse orderings are worse

| Reverse order | Cost |
|---|---|
| A3 → A1 → A2 | scene.xml in `meshdir="../../assets"` state; A1 self-containment can't validate; double work after A2 |
| A1 → A2 → A3 | A1 rewrite happens with assets/urdf still outside package; path resolution requires patching after A2 |
| A1 → A3 → A2 | Same as above + scene.xml also patched twice |
| A2 → A1 → A3 | Workable, but A1 lacks `mujoco.viewer` sanity check tool until after A3 |
| A3 → A2 → A1 | Smallest-first builds confidence but scene.xml needs visual revalidation after A2 |

A2 → A3 → A1 is the dominant choice. All others incur "patch twice" or
"sanity gap" cost.

### Why not parallel execution (multiple sessions)

Per `multi_session_workflow` memory, MVP-3 is **not suitable for parallel
sessions**:
- The Chunk 1 → Chunk 3 hard dependency forces those two to be sequential;
  Chunk 2 sits between them only by convention (it is content-soft from
  both sides). Two of the three chunks are therefore strictly serialized,
  and parallelizing the third would only save Chunk 2's work in isolation
- Chunks are small enough that single-session context is sufficient
- Multi-session coordination overhead > time saved
- Cross-chunk learning (each chunk's lessons inform the next) is easier in a
  single session

Single-session, sequential execution.

### Rollback strategy (codex iter-1 hardened)

- **Pre-commit rollback** (chunk in progress, nothing committed yet): same
  as Chunk 0 — `git restore --staged . && git restore . && git clean -fd <new_dirs>`
  resets a single chunk to the previous commit. Safe and idempotent.
- **Post-commit rollback** (chunk committed, regression discovered):
  **`git revert <chunk_commit_sha>`**, NOT `git reset --hard`. Revert
  creates a forward-moving "undo commit" that preserves history; reset
  rewrites history. Even though `main` is never pushed (per `git_topology`
  memory) so reset is technically safe, **revert is the better hygiene
  habit** because it documents the rollback in the commit log. Codex
  iter-1 explicitly recommended this change. If multiple chunks need
  rolling back, revert them in reverse order (most recent first).
- **Plan-fix-first**: if a chunk reveals a structural plan flaw, stop and
  update the plan/spec before continuing — do NOT hack-patch into the next
  chunk
- **Spec amendment trigger**: if a chunk's brainstorming/execution reveals
  a roadmap-level concern, follow Appendix B's amendment procedure rather
  than silently mutating the spec

---

## 7. Chunk 0 余债 Placement

Chunk 0 left 5 minor code-review items + 1 meta-level lesson. They are
distributed across MVP-3 chunks rather than left as permanent debt.

### Distribution table

| # | Item | Location | Type | Assigned to | Reason |
|---|---|---|---|---|---|
| 1 | unused `elrobot_mjcf_path` parameter | `tests/test_urdf_parity.py:50-65` | lint smell | **Chunk 1** | A2 already touches this file for path resolution; upgrade unused param into a meaningful `assert model.nu == 8` cross-check |
| 2 | "7 revolute + 1 gripper primary" docstring drift | `tests/test_urdf_parity.py:53` | docstring fix | **Chunk 1** | Same file as #1 |
| 3 | `CHANGELOG.md:96` "MVP-3 Chunk 0 commit: TBD" | `CHANGELOG.md:96` | string replace | **Chunk 1** | A2 touches CHANGELOG.md for `[0.2.0]` entry; backfill `6ef605b` simultaneously |
| 4 | `★ P0` unicode glyph | `tests/test_mimic_gripper.py:1` | ASCII replace | **Chunk 3** | A1 owns final tests/ cleanup; Chunks 1/2 don't naturally touch this file (see Open Decision U5 for defer alternative) |
| 5 | `robot.yaml` mimic parallel-array → object-pair schema | `robot.yaml:27-32` | schema redesign | **Defer to MVP-4** | reviewer said "wait for first consumer"; no consumer in MVP-3 |

### Per-chunk impact

- **Chunk 1 plan**: +3 micro steps (~15 plan lines). Estimated length 1000-1400
  remains valid (within buffer)
- **Chunk 2 plan**: +0 (no debt items in scope)
- **Chunk 3 plan**: +1 micro step (~5 plan lines). Estimated length 700-1000
  remains valid

### Meta-level debt: plan template improvement

**Problem**: Chunk 0's plan Phase F file list was incomplete — missed 4 stale
path references (`test_menagerie_walking_skeleton.py:34`, `software/sim-server/README.md:115/142/144`,
`measurements/menagerie_diff.md:8`, `station-sim-menagerie.yaml:13`). The
implementer caught these in Phase G.8 grep and folded the fixes into the same
atomic commit, but the plan should not have shipped with the gaps.

**Root cause**: plan author did not run `grep -rn '<old_path>' software/
hardware/ Makefile docs/` before writing Phase F, so dependent files were
enumerated by recall instead of by exhaustive search.

**Strategy** (chosen: α + γ; β rejected):

- **(α) "grep first" mandatory in chunk plans (chosen)**: every chunk plan in
  MVP-3 (especially path-migration chunks like Chunk 1 and Chunk 3) must
  include in Phase A a `grep -rn` exhaustive scan whose results are folded
  into the Phase F file list. This is a per-plan pattern, not a system-level
  change.
- **(α') "baseline-first" mandatory in chunk plans (added per codex iter-1,
  scope clarified per codex iter-2)**: every chunk plan must capture
  `BASELINE_PASSED` and `BASELINE_SKIPPED` from `make sim-test` in Phase A
  pre-flight, and write all **`make sim-test` total-suite** test-count
  assertions and **`pytest software/sim-server/tests/`** total-suite
  test-count assertions as deltas relative to those baselines — never as
  absolute numbers. **Scope of the rule**: this applies only to *cross-repo
  totals* whose absolute count depends on dev env (mjx) or other concurrent
  commits on main. **Package-local absolute counts are allowed**: the
  engine-tier suite at
  `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/`
  and the cp -r /tmp self-containment count are both fully owned by this
  spec — no external commits add tests there, mjx skip is the only env
  variable, and the count for "1 mjx skip" is stable. Writing absolute
  counts like "4 passed + 1 skip(mjx)" for the package-local suite is
  fine because if mjx becomes installed, the assertion can be expressed
  conditionally as "4 passed + 1 skipped (mjx absent) OR 5 passed +
  0 skipped (mjx present)" in the plan.

  The original MVP-3 spec drafts wrote *cross-repo* absolute counts
  (`90 passed, 1 skipped` etc.), which would silently break if (a)
  `mujoco.mjx` got installed in dev env, (b) another commit on main added
  a test, or (c) spec lines drifted out of sync. Baseline-relative deltas
  for cross-repo totals eliminate that class of fragility while keeping
  package-local absolute counts (which are not fragile) readable.
- **(β) modify writing-plans skill template (rejected)**: would require
  editing files outside `norma-core` repo and would affect other projects.
  Out of scope.
- **(γ) write a lessons-learned doc (chosen)**: a short note captures the
  lesson for future plan authors. Suggested location:
  `docs/superpowers/notes/lessons-from-mvp3-chunk0.md`. Independent task,
  scheduled **after** this roadmap spec lands but **before** Chunk 1
  brainstorming begins. **This timing is decided here**, not in Open
  Decisions — it does not appear in U4.

### Out of MVP-3 scope (debt-related, not assigned)

- LICENSE byte-exactness vs apache.org canonical text — only matters under
  strict legal review
- MVP-2 DoD item 20 post-merge re-verify — MVP-2's debt
- MVP-1 `usbvideo-compat-bridge` late acceptance test — MVP-1's debt
- Chunk 0's "23 vs predicted 21 file count" estimation drift — explained,
  not actually debt

---

## 8. Out of Scope / Deferred to MVP-4

### Deferred to MVP-4 (planned next milestone)

| Item | Reason | When |
|---|---|---|
| **A4: `CITATION.cff`, `pyproject.toml`, `docs/upstream-to-menagerie.md`** | Upstream contribution prep; needs sysID-driven baseline first | After A7 sysID lands or before formal upstream PR |
| **A5: LeRobot EnvHub + `gymnasium.Env` wrapper** | Introduces LeRobot v0.5+ + gymnasium ecosystem; new runtime abstraction | When ElRobot is the target of RL/IL training |
| **A6: MJX CI gate** | Adds JAX dependency (~500MB to CI image); Chunk 0's `test_mjx_compat.py` placeholder is sufficient until then | When differentiable / batched-rollout sim is needed |
| **A7: Real-hardware sysID + `sysid_complete: true`** | Hardware-dependent | When the user has sysID equipment + protocol |
| **PR to `mujoco_menagerie`** | Requires A1 + A2 + A3 + A4 + A7 all done | MVP-5 or later |

### Deferred indefinitely ("when needed")

| Item | Reason |
|---|---|
| `robot.yaml` mimic schema redesign | Wait for first consumer (likely LeRobot integration) |
| LICENSE byte-exact comparison vs apache.org | Only matters under strict legal review |
| MVP-2 DoD item 20 post-merge re-verify | MVP-2's debt; backporting now adds no value |
| MVP-1 `usbvideo-compat-bridge` late acceptance test | MVP-1's debt; MVP-3 doesn't touch the bridge layer |

### Explicitly forbidden in every chunk (boundary recap)

- ❌ **Any physics parameter changes** — `<default>`, `<contact>`, `<actuator>`,
  `<tendon>` blocks in `elrobot_follower.xml` are frozen. **Zero physics drift**
- ❌ **Any Rust changes** — sim-runtime, station, all crates frozen
- ❌ **`norma_sim` API changes** — Chunk 3 may discover `norma_sim` issues but
  must not fix them in MVP-3
- ❌ **New acceptance test functionality** — only path/wrapper smoke tests
  are allowed as new tests; the Floor §3.1 acceptance gate is frozen at MVP-2
- ❌ **`vendor/menagerie/` changes** — vendored upstream, separate update flow
  in `vendor/menagerie/VENDOR.md`
- ❌ **Worktree / branch operations** — same as Chunk 0: direct commit on
  `main`, no worktree, no branch, no push, no PR (per `git_topology` memory)

### Tolerance carve-out (decided per codex iter-1, narrowed)

The "zero physics drift" rule has a **narrow** carve-out for tolerance
adjustment in Chunk 3's test rewrite, but **not a blanket pass**:

- **Equivalent or stricter tolerances**: allowed without ceremony. If the
  rewrite uses `assert pos < 0.0005` where the original used `assert pos < 0.001`,
  no documentation needed.
- **Looser tolerances**: REQUIRES (a) explicit listing of old value vs new
  value in the chunk plan; (b) written rationale (e.g., "raw mujoco timing
  differs from MuJoCoWorld.step() by 1 nanosecond accumulated bias over
  10000 steps"); (c) **side-by-side evidence** — run both old and new
  version of the assertion at HEAD-of-commit-1 (where both files exist)
  and document that the old value was on the edge of the original
  tolerance, not comfortably inside. Without all three, looser tolerance
  is treated as physics drift and rejected at code review.

This rule prevents the rewrite from silently weakening assertions to make
flaky tests pass. Codex iter-1 explicitly recommended this narrowing.

### Deciding when deferred items unlock

**MVP-3 Engine Package Completion completion** = Chunk 3's two split commits
(commit 1 = additive new acceptance + sentinel; commit 2 = deletion +
cleanup) both landed + double review passed for both + memory updated.
After that:

1. User picks the next milestone (MVP-4 upstream / MVP-4 training / MVP-4
   sysID / other)
2. Chosen direction starts a new brainstorming → spec → plan cycle
3. Some deferred items unlock; rest stay deferred

**Do not change MVP-3 scope mid-execution.** Any "since I'm here, I'll
also..." temptation is recorded in memory for MVP-4 brainstorming.

---

## 9. Open Decisions & Risks

### Resolved decisions (decided during codex iter-1 review, no longer open)

| # | Question | Decision | Reason |
|---|---|---|---|
| **U1** | Chunk 3 VERSION bump | `0.2.2` patch (chosen) | Codex iter-1: don't use semver to mark milestone; use it to signal consumer-visible change. Chunk 3 has zero consumer-visible change. |
| **U2** | Chunk 3 atomic vs split commit | **Split into 2 commits** (chosen) | Codex iter-1 recommended split. Commit 1 = additive (new package acceptance + sentinel both green). Commit 2 = deletion + cleanup. Split makes the assertion-equivalence audit much easier because reviewer can see both old + new at HEAD-of-commit-1. |
| **U6** | "Zero physics drift" tolerance carve-out | **Narrowed** (chosen) | Equivalent-or-stricter allowed without ceremony; looser requires plan documentation + rationale + side-by-side evidence. Section 8 has the full rule. |

### Unresolved decisions (still to be resolved during user spec review)

| # | Question | Default | Alternatives | Impact |
|---|---|---|---|---|
| **U3** | Chunk 2 manual viewer GUI gate | Optional ("GUI environment, otherwise skipped") | Mandatory; or remove entirely | Depends on user dev environment (WSL2 + WSLg has GUI) |
| **U4** | Section 7 meta-debt strategy | (α) + (α') chunk-level grep-first + baseline-relative deltas + (γ) lessons-learned doc (γ timing already decided in Section 7: after spec lands, before Chunk 1 brainstorming) | (β) modify writing-plans skill template (rejected — out of repo scope) | Strategy choice; γ timing not subject to U4 |
| **U5** | Item 4 (`★` glyph) placement | Chunk 3 fold-in | Defer entirely to MVP-4 / MVP-5 (upstream prep only) | Cosmetic |
| **U7** | git tag strategy | No tag (Chunk 0 and MVP-2 both used naked `main` commits without tags) | Tag `mvp3-engine-package-completion` at completion; or per-chunk tags | History readability vs ceremony |

### Known risks (priority sorted, reframed per codex iter-1)

**Critical (chunk-blocking if not mitigated)**

1. **Hidden URDF references in Chunk 1**: URDF may be referenced from unexpected
   files. Mitigation: Section 7 (α) "grep first" applied in Chunk 1 plan
   Phase A.
2. **Manifest-layer coverage drop in Chunk 3 rewrite**: rewriting the
   acceptance suite from `MuJoCoWorld.from_manifest_path(scene_yaml)` entry
   to `MjModel.from_xml_path(mjcf_path)` entry silently abandons the
   manifest-pipeline coverage (scene.yaml parsing, actuator_annotations
   validation, mjcf_path resolution). Mitigation: **new sentinel test** in
   `software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py`
   exercises the full pipeline. Spec-compliance review must verify the
   sentinel exists and exercises the full pipeline (not just imports).
3. **Physics-layer assertion equivalence in Chunk 3 rewrite**: even at the
   physics layer, the rewrite may silently weaken assertions. Mitigation:
   the split commit topology (commit 1 = add new + sentinel; commit 2 =
   delete old) lets spec-compliance review compare old and new
   side-by-side at HEAD-of-commit-1. Tolerance carve-out (Section 8) has
   strict rules for any loosening.
4. **Test count fragility**: writing absolute test counts in plans creates
   false failures when dev env changes (e.g., mjx gets installed) or
   when other commits add tests. Mitigation: Section 7 (α') mandates
   baseline-relative deltas in every chunk plan. Captured during Phase A
   pre-flight; never hardcoded.

**High (chunk-internal, recoverable)**

5. **STL bulk move correctness in Chunk 1**: 19 individual `git mv`. Mitigation:
   per-file enumerated `git mv` (matching Chunk 0 pattern), verify each
   rename via `git status`.
6. **`test_urdf_parity` upgrade from skip to fail (Chunk 1)**: after Chunk 1
   moves URDF into the package, the fixture must hard-fail on missing URDF,
   not silently skip. Codex iter-1 explicitly recommended this upgrade —
   it eliminates the off-by-one silent-skip class of bug. Mitigation: Chunk 1
   plan includes the fixture upgrade as a step; verification gate explicitly
   counts `PASSED` lines via `grep -c PASSED`.
7. **`<include>` + `<compiler>` collision in Chunk 2**: `scene.xml` must
   not declare its own `<compiler>` (would override main MJCF's meshdir
   via namespace merge) or `<default>` (would collide with main MJCF's).
   Codex iter-1 reframed the original "meshdir relative resolution" risk —
   `meshdir="assets"` and `meshdir="../../assets"` have identical compiler
   resolution; the real risk is double-declaration via `<include>`.
8. **scene.xml smoke test must use name-based assertions** (Chunk 2):
   `m.ngeom == <count>` is fragile to future geometry additions.
   `mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor") >= 0` is
   stable. Codex iter-1 explicitly flagged this.

**Medium (cross-chunk coordination)**

9. **VERSION / CHANGELOG / robot.yaml three-way sync**: every chunk plan must
   verify all three are consistent before commit.
10. **Per-chunk plan completeness**: Section 7 (α) "grep first" + (α')
    "baseline first" must be applied to every chunk plan, not just the
    first one.

**Low (acceptable noise)**

11. **Chunk 3 verbosity blowup is smaller than feared** (codex iter-1):
    13 tests are already ~95% raw-mujoco-compatible; the rewrite is mostly
    mechanical `world.` deletion. Estimated 400-500 lines, not 800-1200.
12. **Total session execution time**: 3 chunks ≈ 2-4 hours single session.
    `/compact` if context exceeds 150K.

### Risk ownership

| Who catches | Risks (renumbered post-codex-iter-1) |
|---|---|
| Brainstorming spec review (current phase) | U3-U7, defer list completeness, ordering rationale, milestone framing |
| Codex iter-1+ external review | #2, #3, #4, #6, #7, #8 (cross-spec/code consistency that internal spec-doc-reviewer can't see) |
| Per-chunk plan review | #1, #5, #9, #10 |
| Per-chunk implementer self-review | #2, #6, #11 |
| Per-chunk spec-compliance review | #2, #3, #9 |
| Per-chunk code-quality review | #11 |
| User (cross-chunk) | #12 (context monitoring) |

### Carry-forward (not handled by this spec)

- Plan-implicit-widening root cause = missing grep step. Already covered by
  Section 7 (α)+(γ).
- MVP-2 Chunk 5 had the same root cause (different plan, same problem),
  validating that (α) is necessary, not a Chunk 0 one-off.

---

## Appendix A: Test count chain (baseline-relative deltas)

**Codex iter-1 hardening**: this table now records test count **deltas**
rather than absolutes. The earlier version embedded absolute counts (`90
passed, 1 skipped` etc.) which depended on (a) the dev env having no
`mujoco.mjx` installed, and (b) no concurrent test additions on `main`.
Both can change. Plans MUST capture `BASELINE_PASSED` and `BASELINE_SKIPPED`
from `make sim-test` at Phase A pre-flight, and assert deltas relative to
those baselines.

The reference baselines for each chunk:
- **Chunk 1 baseline** = `make sim-test` immediately after Chunk 0 (`6ef605b`).
  At time of writing this spec, this happens to be `90 passed, 1 skipped`,
  but plans must re-measure rather than hardcode.
- **Chunk 2 baseline** = `make sim-test` after Chunk 1 lands.
- **Chunk 3 baseline** = `make sim-test` after Chunk 2 lands.

| Stage | `make sim-test` delta vs prior chunk | sim-server alone delta | engine-tier alone (in repo) absolute | cp -r /tmp self-containment absolute |
|---|---|---|---|---|
| **Chunk 0 baseline (HEAD `6ef605b`)** | (set baseline) | (set baseline) | 4 passed + 1 skip(mjx) | **N/A** — package not self-contained yet (assets + urdf still outside; cp -r currently produces 0 passed / 2 failed / 3 skipped because `meshdir="../../assets"` cannot resolve in `/tmp`) |
| **Chunk 1 (A2: assets+urdf)** | +0 (organizational only) | +0 | 4 passed + 1 skip(mjx) | 4 passed + 1 skip(mjx) — **first chunk where cp -r is meaningful** (Chunk 1 moves assets+urdf into the package) |
| **Chunk 2 (A3: scene.xml)** | +1 (`test_scene_loadable.py`) | +0 | 5 passed + 1 skip(mjx) | 5 passed + 1 skip(mjx) |
| **Chunk 3 (A1: test decoupling), at end of commit 2** | +1 (sim-server -13 acceptance + 1 sentinel = -12; engine-tier +13; net +1) | -12 | 18 passed + 1 skip(mjx) | **18 passed + 1 skip(mjx)** |
| **Chunk 3 intermediate, end of commit 1** (transient) | +14 (both old and new acceptance present + sentinel) | +1 (sentinel added; old still present) | 18 passed + 1 skip(mjx) | (transient, not used for verification) |

Notes:
- "+1 skip(mjx)" / "+1s" means `test_mjx_compat.py` placeholder skipped
  because mjx is not installed in dev env. If mjx IS installed, it
  becomes "+0 skip" and the skipped column drops by 1 — chunk plan
  assertions on `BASELINE_SKIPPED` automatically adapt because they're
  baseline-relative
- "engine-tier alone (in repo)" = `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/`
  run from the repo root (without PYTHONPATH); resolves `meshdir`/URDF
  fixture paths via the repo's directory layout. These counts are
  absolute because the engine-tier package's own test count is fully
  owned by this spec (no external main commits add tests there).
- "cp -r /tmp self-containment" = same suite executed after `cp -r mujoco/elrobot_follower /tmp/`;
  only meaningful once the package owns its own assets + URDF (Chunk 1
  onwards). Counts are absolute for the same reason.
- **The Chunk 3 split commit topology**: at HEAD-of-commit-1 (additive
  only), both old sim-server `test_elrobot_acceptance.py` and the new
  package-located one exist + the sentinel — the +14 transient delta
  enables side-by-side audit of assertion equivalence. At HEAD-of-commit-2
  (after deletion + cleanup), the delta resolves to +1 net.
- The Chunk-0 cp -r failing state is exactly what motivates MVP-3
  Engine Package Completion: Section 1 success criterion #3
  ("fully self-contained") goes from N/A → passing across the 3 chunks
- **Why deltas instead of absolutes** (codex iter-1 finding): writing
  absolute counts in the spec creates false failures when (a) dev env
  changes (mjx installed), (b) other commits add tests on main, or (c)
  spec lines drift out of sync with each other. Delta math is robust to
  all three.

## Appendix B: Spec amendment hook

If a chunk's brainstorming surfaces a roadmap-level concern that
invalidates a Section 3/4/5 success criterion, prerequisite, or boundary
statement, or a Section 6 dependency claim — **append a `## Spec
amendments` section to this file before starting the affected chunk's
plan**, similar to the amendment chain in
`2026-04-12-mvp3-first-class-mjcf-design.md` (the Chunk 0 spec). Do not
silently overwrite existing content. Each amendment entry should include
date, what was found, what changed, and why.

**When to amend vs when to handle in normal review**:

- **Amend** when a finding invalidates a *success criterion*, a
  *prerequisite*, a *boundary statement*, or a *dependency edge*
- **Resolve via normal per-chunk brainstorming** when a finding only
  refines an unresolved Open Decision (U1-U7) default — those are
  expected to be decided during chunk-level work, not amendments
- **Just note in the chunk plan** when a finding is purely
  chunk-internal (e.g., "Chunk 1 plan needs an extra micro-step to fix
  STL filename casing") — the chunk plan owns its own details

The intent is to make Section 8's "do not change MVP-3 scope
mid-execution" enforceable: amendments are visible, traceable, and
required to be discussed during the per-chunk brainstorming, not patched
into a chunk plan.

## Appendix C: Spec metadata

- **Author**: brainstorming session 2026-04-12
- **Brainstorming process**: 4 clarifying questions (Q1 scope, Q2 end state,
  Q3 A6 in/out, Q4 chunk ordering) + Approach A (lean roadmap) chosen +
  3 spec-document-reviewer iterations + 1 codex iter-1 review (which
  surfaced 4 critical fixes: milestone framing, Chunk 3 mapping/sentinel,
  baseline-relative deltas, rollback hardening + URDF fixture upgrade)
- **Predecessor commit**: `6ef605b` (Chunk 0)
- **Estimated MVP-3 Engine Package Completion total work**: 3 chunks
  totaling 4 atomic commits (Chunk 1: 1, Chunk 2: 1, **Chunk 3: 2 split
  per codex iter-1**), ~40-50 file ops, ~2000-3000 plan lines combined,
  2-4 hours single-session execution
- **End-state version**: `mujoco/elrobot_follower/VERSION` 0.1.0 → **0.2.2**
  (decided per codex iter-1: patch, not minor — Chunk 3 has zero
  consumer-visible change so milestone marker via semver was rejected)
- **End-state `make sim-test`**: baseline + 2 (delta-based; absolute count
  depends on dev env mjx availability and other main commits)
- **End-state `upstream.prerequisites`**: 3/5 done (was 1/5 at Chunk 0)
- **Codex consult session**: `019d7726-6dcf-7fe2-8887-35ee3b9c2568`
  (continued from Chunk 0 brainstorming; iter-1 dispatched after spec
  was written, found 4 critical issues + 7 advisory items, applied to
  this commit)

*End of spec.*
