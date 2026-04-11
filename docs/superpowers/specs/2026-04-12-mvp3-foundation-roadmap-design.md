# MVP-3 Foundation Roadmap — Design Spec

> **Type**: Lean roadmap spec (per brainstorming Approach A). Defines the chunk
> decomposition, dependencies, and boundaries of MVP-3 Foundation. Each chunk
> gets its own brainstorming → spec → plan → execute cycle when its turn comes.
> This spec is **not** a per-chunk implementation spec.

**Status**: draft, awaiting spec-document-reviewer + user review
**Date**: 2026-04-12
**Predecessor**: `docs/superpowers/specs/2026-04-12-mvp3-first-class-mjcf-design.md` (Chunk 0 spec)
**Predecessor commit**: `6ef605b` on `main` (Chunk 0 atomic restructure landed)
**Successor**: per-chunk specs (TBD, one per chunk after Chunk 1 brainstorming starts)

---

## 1. Goal & Success Criterion

**Milestone**: **MVP-3 Foundation** — finish what Chunk 0 started. Chunk 0
promoted `hardware/elrobot/simulation/` to a three-tier first-class structure
but only built the *skeleton* of the engine-tier robot package. MVP-3 Foundation
fills in the *contents* so the package is genuinely self-contained, visually
verifiable, and test-decoupled from `norma_sim`.

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

### Success criterion (MVP-3 Foundation is "done" when)

1. `python3 -m pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -v`
   passes **without** `PYTHONPATH=software/sim-server` and includes the 13
   acceptance tests (currently in sim-server)
2. `python3 -m mujoco.viewer hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml`
   exists as a runnable wrapper with floor + lights (manual GUI verification
   optional in headless dev environments)
3. The package is **fully self-contained**:
   `cp -r mujoco/elrobot_follower /tmp/test && cd /tmp/test && pytest tests/`
   passes (every dependency lives inside the package directory)
4. `make sim-test` is fully green with no regressions (≥90 passed)
5. `mujoco/elrobot_follower/robot.yaml`'s `upstream.prerequisites` list moves
   from 1/5 done (Chunk 0) to **3/5 done** (Chunk 0 + MVP-3 Foundation):
   - ✅ `LICENSE file present (done at 0.1.0)` — done at Chunk 0
   - ✅ `Passing tests executable in isolation` — **done at MVP-3**
   - ✅ `scene.xml wrapper with lights/floor` — **done at MVP-3**
   - ❌ `sysid_complete == true` — needs hardware (deferred MVP-4+)
   - ❌ `Full CITATION.cff metadata` — deferred MVP-4+

### Why this scope (vs other MVP-3 framings considered)

Three other framings were considered and rejected during brainstorming:

- **Upstream-ready package (A1+A2+A3+A4 partial+A7 partial)**: requires hardware
  (sysID) and CITATION metadata. Deferred MVP-4.
- **Policy-training-ready (A5 LeRobot + A6 MJX)**: introduces JAX + LeRobot
  ecosystem dependencies. YAGNI for MVP-3.
- **Real-world-validated (A7 sysID-driven)**: needs hardware. Deferred MVP-4.

The chosen framing — "Foundation only" — is the natural continuation of
Chunk 0's organizational refactor and is the dominant prerequisite for any of
the other framings. After MVP-3 Foundation lands, all four MVP-4 directions
remain open.

---

## 2. Chunk Inventory

| | **Chunk 1** | **Chunk 2** | **Chunk 3** |
|---|---|---|---|
| **Codename** | A2 — Assets+URDF Move | A3 — Scene Wrapper | A1 — Test Decoupling |
| **Core action** | Move `assets/` (19 STL) + `elrobot_follower.urdf` into the package; simplify `meshdir`; update all dependent paths | Add Menagerie-style `scene.xml` (lights + floor + `<include>`) | Rewrite `test_elrobot_acceptance.py` (13 tests) as pure-mujoco; move into package |
| **Size** | Large (~25-35 file ops) | Small (1-2 new files) | Medium (1 large rewrite + cleanup) |
| **Risk** | High (most file movement) | Low (pure additive) | Medium-high (behavioral rewrite) |
| **VERSION bump** | 0.1.0 → 0.2.0 (minor) | 0.2.0 → 0.2.1 (patch) | 0.2.1 → 0.2.2 (patch) — see Open Decision U1 |
| **Atomic commit** | 1 | 1 | 1 (see Open Decision U2) |
| **Test count delta** | 0 | +1-2 (scene loadable smoke) | 0 (sim-server -13, engine-tier +13) |
| **Post-chunk `make sim-test`** | 90 passed, 1 skipped | 91-92 passed, 1 skipped | 91-92 passed, 1 skipped |
| **Estimated plan length** | 1000-1400 lines | 300-500 lines | 700-1000 lines |
| **Prerequisite** | Chunk 0 (`6ef605b`) | Chunk 1 landed | Chunk 1 landed (hard); Chunk 2 landed (soft) |

**Total MVP-3 Foundation**: 3 atomic commits, ~50-70 file ops, ~2000-3000
plan lines combined. Smaller than MVP-2 (4216 lines / 7 chunks) because the
work is purely organizational — no new code logic.

---

## 3. Chunk 1 — A2: Assets + URDF Move

### Scope

Move `hardware/elrobot/simulation/assets/` (19 STL meshes) and
`hardware/elrobot/simulation/elrobot_follower.urdf` **into**
`hardware/elrobot/simulation/mujoco/elrobot_follower/`. Simplify the MJCF's
`meshdir="../../assets"` to `meshdir="assets"`. Update every dependent path
(test fixtures, sim-server fixtures, README references, conftest paths).
Bump VERSION to 0.2.0 (minor — structural package layout change). Tendons
are: this fulfills Chunk 0's "future chunk will move assets" promise.

### Prerequisites

- HEAD = `6ef605b` (Chunk 0) or newer
- `make sim-test` baseline 90 passed, 1 skipped (mjx)
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

### Success criteria

1. `python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml'); print(m.nu, m.neq)"`
   → `nu=8 neq=2`
2. `make sim-test` → 90 passed, 1 skipped
3. `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -v`
   (no PYTHONPATH) → 4 passed + 1 skipped (mjx)
4. `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_urdf_parity.py -v`
   → 2 **PASSED** (counted explicitly via grep, not just "no failures" — see
   Risk #6)
5. **Self-containment**: `cp -r hardware/elrobot/simulation/mujoco/elrobot_follower /tmp/elrobot-test && cd /tmp/elrobot-test && python3 -m pytest tests/ -v`
   → 4 passed + 1 skipped (mjx)
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
   error leaving partial state. Mitigation: write a bash loop in plan but
   verify each rename via `git status` afterward.
3. **`meshdir` relative resolution edge cases**: MuJoCo resolves `meshdir`
   relative to the MJCF file. After move, `meshdir="assets"` resolves to
   `mujoco/elrobot_follower/assets/`. Verify by actually compiling, not by
   path-string inspection alone.
4. **`test_urdf_parity` skip-vs-fail trap**: the URDF fixture currently uses
   `pytest.skip("URDF not found")` not `pytest.fail`. If the new path
   resolution layer count is off-by-one, the test silently skips rather than
   failing. Verification gate must explicitly count `PASSED` lines via
   `grep -c PASSED`, not just look at the summary.

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

- Chunk 1 (`A2 — Assets+URDF Move`) landed
- Package is self-contained at this point (assets + URDF in package)
- `make sim-test` baseline 90 passed, 1 skipped

### File operations

**Create**:
- `mujoco/elrobot_follower/scene.xml` — new file, ~30-50 lines, Menagerie style
- `mujoco/elrobot_follower/tests/test_scene_loadable.py` — 1 new smoke test
  verifying `mujoco.MjModel.from_xml_path('scene.xml')` compiles and asserting
  `nu == 8` (cross-check that `<include>` namespace merging worked)

**Edit**:
- `mujoco/elrobot_follower/README.md`: add `scene.xml` to Structure section;
  add a "How to view" snippet
- `mujoco/elrobot_follower/CHANGELOG.md`: add `[0.2.1] — 2026-04-12` entry
- `mujoco/elrobot_follower/VERSION`: `0.2.0` → `0.2.1`
- `mujoco/elrobot_follower/robot.yaml`: bump `version.current`; mark
  `scene.xml wrapper` prereq as done

**No moves, no deletions, no path updates** — purely additive.

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

### Success criteria

1. `python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml'); print(f'nu={m.nu} ngeom={m.ngeom}')"`
   → `nu=8` (matching `elrobot_follower.xml`) and `ngeom` is 1 greater (the
   floor)
2. `pytest .../tests/test_scene_loadable.py -v` → 1 PASSED
3. `make sim-test` → 91 passed, 1 skipped
4. `pytest hardware/elrobot/simulation/mujoco/elrobot_follower/tests/ -v`
   (no PYTHONPATH) → 5 passed + 1 skipped
5. **Self-containment** (cp -r /tmp pattern) still passes
6. Optional GUI gate: `python3 -m mujoco.viewer .../scene.xml` opens visualization
   (skipped in headless dev environments — see Open Decision U3)
7. `git status` clean; `make check-arch-invariants` green

### Risks (Chunk-2-specific)

1. **`<include>` namespace merging**: MuJoCo's `<include>` is textual merge,
   not import. If `scene.xml` declares its own `<default class="elrobot">` or
   reuses an asset name from the main MJCF, namespace conflicts result.
   Mitigation: scene.xml only declares `<visual>`, `<statistic>`, `<asset>`
   (with names that don't collide), `<worldbody>` (light + floor only),
   `<include>`. Nothing else.
2. **Borrowed Menagerie textures**: using builtin textures (`builtin="checker"`)
   avoids binary assets and licensing concerns. Don't add a real PNG.
3. **Smoke test depth**: `test_scene_loadable.py` must assert real invariants
   (`nu`, `ngeom`), not just `from_xml_path` no-error. Otherwise it's a
   useless import test.

### Boundary

- ❌ Rewrite `test_elrobot_acceptance.py` — Chunk 3
- ❌ Camera presets beyond scene.xml defaults
- ❌ Headless rendering / image capture / mp4
- ❌ Any physics changes
- ❌ Adding mesh-based geoms (the scene.xml is purely visual scaffolding)

### VERSION bump policy

`0.2.0` → `0.2.1` (patch). Reason: purely additive; no API or physics change.
Consumers don't need to do anything. Optional new entry point. See Open
Decision U1 for an alternative (`0.3.0` minor as foundation milestone marker).

---

## 5. Chunk 3 — A1: Test Decoupling

### Scope

Rewrite `software/sim-server/tests/integration/test_elrobot_acceptance.py`
(13 tests) as **pure mujoco** — remove all `norma_sim` imports and use raw
`mujoco.MjModel` / `mujoco.MjData` APIs directly. Move the rewritten file to
`hardware/elrobot/simulation/mujoco/elrobot_follower/tests/test_elrobot_acceptance.py`.
Assertion semantics must remain equivalent (no coverage weakening). This is
the MVP-3 Foundation capstone — afterwards `mujoco/elrobot_follower/` is a
complete, fresh-checkout-runnable robot package.

### Prerequisites

- Chunk 2 (`A3 — Scene Wrapper`) landed
- `make sim-test` baseline 91 passed, 1 skipped
- Package is self-contained (assets + URDF + scene.xml all in place)

### File operations

**Rewrite + move**: rewrite content of
`software/sim-server/tests/integration/test_elrobot_acceptance.py` first,
then `git mv` to
`mujoco/elrobot_follower/tests/test_elrobot_acceptance.py`. The rewrite is
substantial enough that git's rename detection (default 50% similarity) will
likely show as delete + add, not rename. Acceptable — commit message documents
the move explicitly.

**Edit**:
- `mujoco/elrobot_follower/tests/conftest.py`: possibly add an `elrobot_data`
  or `elrobot_model_at_rest` fixture to share boilerplate across the 13 tests
- `software/sim-server/tests/conftest.py`: check if `elrobot_mjcf_path` /
  `elrobot_scene_yaml` fixtures still have consumers; if test_elrobot_acceptance
  was the only consumer, delete the dead fixtures. Otherwise leave them.
- `mujoco/elrobot_follower/CHANGELOG.md`: add `[0.2.2]` entry (or `[0.3.0]`
  if Open Decision U1 chooses minor)
- `mujoco/elrobot_follower/VERSION`: `0.2.1` → `0.2.2`
- `mujoco/elrobot_follower/robot.yaml`: bump `version.current`; mark
  `Passing tests executable in isolation` prereq as done
- `mujoco/elrobot_follower/README.md`: rewrite "Relationship to NormaCore"
  section — all engine-tier tests now in package, sim-server only owns
  Norma-specific integration tests

### Folded-in Chunk 0 余债

- Item 4: `tests/test_mimic_gripper.py:1` `★ P0` unicode glyph → `[P0]` ASCII
  (see Open Decision U5 for alternative — defer to upstream prep)

### Rewrite strategy

Mapping table from `norma_sim` API to raw mujoco API (final form determined in
Chunk 3 brainstorming):

| Original (`norma_sim`) | Rewrite (raw mujoco) |
|---|---|
| `from norma_sim.world.model import MuJoCoWorld` | `import mujoco` |
| `MuJoCoWorld.from_manifest_path(scene_yaml)` | `mujoco.MjModel.from_xml_path(mjcf_path)` |
| `world.model` | `model` (raw `MjModel`) |
| `world.data` | `data = mujoco.MjData(model)` |
| `world.actuator_by_client_id('rev_motor_XX').mjcf_id` | `mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'act_motor_XX')` |
| `world.actuator_by_mjcf_name(...).capability.kind` | direct `model.actuator_*` array reads |
| `world.step(dt=...)` | `mujoco.mj_step(model, data)` |
| `world.set_ctrl(ctrl_dict)` | `data.ctrl[i] = value` |
| `world.qpos / world.qvel` | `data.qpos / data.qvel` |

**Naming convention**: rewritten tests use **`act_motor_XX` (mjcf names)
exclusively**, never `rev_motor_XX` (Norma client_ids). Engine-tier tests
should be Norma-agnostic. Tests for the client_id mapping itself belong in
sim-server tests.

### Success criteria

1. **Coverage equivalence**: 13 test functions exist; each one's assertions are
   semantically equivalent to the original (manual side-by-side review during
   spec-compliance review)
2. `pytest .../tests/test_elrobot_acceptance.py -v` (no PYTHONPATH) → 13 passed
3. `grep -c 'norma_sim' .../tests/test_elrobot_acceptance.py` → 0
4. **Total test count conserved**: `make sim-test` → 91 passed, 1 skipped
   (sim-server -13, engine-tier +13, net 0; +0 from Chunk 2's scene smoke
   already in baseline)
5. **Full self-containment**:
   `cp -r mujoco/elrobot_follower /tmp/elrobot-test && cd /tmp/elrobot-test && pytest tests/ -v`
   → **17 passed + 1 skipped** (4 from Chunk 0 + 1 from Chunk 2 scene smoke +
   13 from Chunk 3 acceptance + 1 mjx skip)
6. sim-server cleanup: `software/sim-server/tests/integration/test_elrobot_acceptance.py`
   no longer exists; `pytest software/sim-server/tests/ -q` → 78 passed
   (91 - 13)
7. `make check-arch-invariants` green
8. Phase G.8 grep:
   `grep -rn 'test_elrobot_acceptance' software/ --include='*.py'` → no stale
   references
9. `git status` clean

### Risks (Chunk-3-specific)

1. **Behavioral drift** (critical): `MuJoCoWorld.__init__` may do implicit
   setup (ctrlrange validation, initial qpos, forcerange clamps) that the
   original tests rely on without realizing. The rewrite must mirror every
   such setup or the new tests pass on a different state than the originals.
   Mitigation: Chunk 3 plan **must** include a Phase 0 step "read
   `software/sim-server/norma_sim/world/model.py::MuJoCoWorld.__init__` and
   list every implicit setup as a checklist". Mirror each item into the new
   conftest fixture or test setup.
2. **Coverage reduction** (critical): if a `norma_sim` helper (e.g.,
   `world.assert_no_self_collision()`) has no 1:1 raw-mujoco equivalent, the
   rewrite may silently drop the assertion. Mitigation: spec-compliance
   review for Chunk 3 must do side-by-side reading of original vs new for
   every test function and verify assertion count + assertion strength
   equivalence.
3. **Stale references after delete**: scripts/CI YAML/Makefile may reference
   the old test path. Mitigation: Phase G.8 grep pattern.
4. **Dead fixture cleanup**: must `grep` before deleting fixtures from
   sim-server conftest to ensure no other consumer.
5. **Verbosity blowup**: raw mujoco is more verbose than `norma_sim` wrappers.
   Estimated 13 tests × 30 lines = 400 lines original; rewrite may be 800-1200
   lines. Mitigation: extract 1-2 shared fixtures in conftest. Acceptable
   either way — verbosity is not a quality issue if coverage is equivalent.

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
API, no physics, no consumer impact. See Open Decision U1 for `0.3.0` minor
alternative as a "MVP-3 Foundation done" milestone marker.

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
   │       │       └─→ Chunk 3 (A1: test) [soft + hard combination]
   │       │
   │       └────────→ Chunk 3 (A1: test)  [hard: self-containment requires assets in package]
   │
   └────────────────→ Chunk 3 (A1: test)  [no direct dependency]
```

The **only true hard dependency** is Chunk 1 → Chunk 3. Chunk 2 in the
middle is logical-order optimization, not technical necessity. We chose
A2 → A3 → A1 because:

- A2 first unlocks package self-containment, the precondition for A3 and A1
- A3 second is a cheap sanity check — `mujoco.viewer scene.xml` after A2
  visually validates the move
- A1 last is the largest behavioral change; it benefits from running on a
  fully-stabilized package

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
- 3 chunks form a hard sequential chain (A2 → A3 → A1)
- Chunks are small enough that single-session context is sufficient
- Multi-session coordination overhead > time saved
- Cross-chunk learning (each chunk's lessons inform the next) is easier in a
  single session

Single-session, sequential execution.

### Rollback strategy

- **Chunk-internal rollback**: same as Chunk 0 — `git restore --staged . &&
  git restore . && git clean -fd <new_dirs>` resets a single chunk to the
  previous commit
- **Cross-chunk rollback**: `git reset --hard <chunk_N_minus_1_commit_sha>`.
  No force-push needed (per `git_topology` memory: main never pushed)
- **Plan-fix-first**: if a chunk reveals a structural plan flaw, stop and
  update the plan/spec before continuing — do NOT hack-patch into the next
  chunk

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
- **(β) modify writing-plans skill template (rejected)**: would require
  editing files outside `norma-core` repo and would affect other projects.
  Out of scope.
- **(γ) write a lessons-learned doc (chosen)**: a short note captures the
  lesson for future plan authors. Suggested location:
  `docs/superpowers/notes/lessons-from-mvp3-chunk0.md`. Independent task,
  scheduled **after** this roadmap spec lands but **before** Chunk 1
  brainstorming begins.

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

### Tolerance carve-out (see Open Decision U6)

The "zero physics drift" rule does NOT include test tolerance adjustment: if
a Chunk 3 rewrite encounters numerical noise that requires loosening a
tolerance value (e.g., `assert pos < 0.001` → `assert pos < 0.005`), this is
test infrastructure work, not physics drift. Document the change in the
chunk plan with rationale; no spec amendment needed.

### Deciding when deferred items unlock

**MVP-3 Foundation completion** = Chunk 3 atomic commit landed + double review
passed + memory updated. After that:

1. User picks the next milestone (MVP-4 upstream / MVP-4 training / MVP-4
   sysID / other)
2. Chosen direction starts a new brainstorming → spec → plan cycle
3. Some deferred items unlock; rest stay deferred

**Do not change MVP-3 scope mid-execution.** Any "since I'm here, I'll
also..." temptation is recorded in memory for MVP-4 brainstorming.

---

## 9. Open Decisions & Risks

### Unresolved decisions (to be resolved during user spec review)

| # | Question | Default | Alternatives | Impact |
|---|---|---|---|---|
| **U1** | Chunk 3 VERSION bump | `0.2.2` patch | `0.3.0` minor (mark "MVP-3 Foundation done") | Cosmetic; just affects VERSION display |
| **U2** | Chunk 3 atomic vs split commit | Atomic (1 commit) | Split into "rewrite in place" + "move + cleanup" | Affects readability vs atomicity |
| **U3** | Chunk 2 manual viewer GUI gate | Optional ("GUI environment, otherwise skipped") | Mandatory; or remove entirely | Depends on user dev environment (WSL2 + WSLg has GUI) |
| **U4** | Section 7 meta-debt strategy | (α) chunk-level grep-first + (γ) lessons-learned doc | (β) modify writing-plans skill template | (γ) timing — before or after Chunk 1 brainstorming |
| **U5** | Item 4 (`★` glyph) placement | Chunk 3 fold-in | Defer entirely to MVP-4 / MVP-5 (upstream prep only) | Cosmetic |
| **U6** | "Zero physics drift" tolerance carve-out | Allow tolerance adjustment as test infrastructure | Disallow (treat tolerance as physics) | Affects Chunk 3 rewrite latitude |
| **U7** | git tag strategy | No tag (consistent with Chunk 0 / MVP-2) | Tag `mvp3-foundation` at completion; or per-chunk tags | History readability vs ceremony |

### Known risks (priority sorted)

**Critical (chunk-blocking if not mitigated)**

1. **Hidden URDF references in Chunk 1**: URDF may be referenced from unexpected
   files. Mitigation: Section 7 (α) "grep first" applied in Chunk 1 plan
   Phase A.
2. **Behavioral drift in Chunk 3 rewrite**: `MuJoCoWorld.__init__` implicit
   setup may not be mirrored. Mitigation: Chunk 3 plan Phase 0 reads
   `MuJoCoWorld.__init__` source and produces an explicit setup checklist.
3. **Coverage reduction in Chunk 3 rewrite**: silently dropped assertions.
   Mitigation: spec-compliance review does side-by-side original-vs-new
   reading per test function.

**High (chunk-internal, recoverable)**

4. **STL bulk move correctness in Chunk 1**: 19 individual `git mv`. Mitigation:
   per-file verification via `git status`.
5. **`meshdir` relative resolution edge cases in Chunk 1**: verify by actual
   compile, not by string inspection.
6. **`test_urdf_parity` skip-vs-fail trap in Chunk 1**: explicit `grep -c
   PASSED` count, not just summary.
7. **`<include>` namespace merging in Chunk 2**: scene.xml only declares
   non-colliding elements.

**Medium (cross-chunk coordination)**

8. **VERSION / CHANGELOG / robot.yaml three-way sync**: every chunk plan must
   verify all three are consistent before commit.
9. **Per-chunk plan completeness**: Section 7 (α) "grep first" must be applied
   to every chunk plan, not just the first one.

**Low (acceptable noise)**

10. **Chunk 3 verbosity blowup**: raw mujoco verbose. Acceptable; mitigate
    via shared conftest fixtures.
11. **Total session execution time**: 3 chunks ≈ 2-4 hours single session.
    `/compact` if context exceeds 150K.

### Risk ownership

| Who catches | Risks |
|---|---|
| Brainstorming spec review (current phase) | U1-U7, defer list completeness, ordering rationale |
| Per-chunk plan review | #1, #4, #6, #7, #8, #9 |
| Per-chunk implementer self-review | #2, #5, #10 |
| Per-chunk spec-compliance review | #3, #8 |
| Per-chunk code-quality review | #10 |
| User (cross-chunk) | #11 (context monitoring) |

### Carry-forward (not handled by this spec)

- Plan-implicit-widening root cause = missing grep step. Already covered by
  Section 7 (α)+(γ).
- MVP-2 Chunk 5 had the same root cause (different plan, same problem),
  validating that (α) is necessary, not a Chunk 0 one-off.

---

## Appendix: Spec metadata

- **Author**: brainstorming session 2026-04-12
- **Brainstorming process**: 4 clarifying questions (Q1 scope, Q2 end state,
  Q3 A6 in/out, Q4 chunk ordering) + Approach A (lean roadmap) chosen
- **Predecessor commit**: `6ef605b` (Chunk 0)
- **Estimated MVP-3 Foundation total work**: 3 chunks, 50-70 file ops,
  2000-3000 plan lines combined, 2-4 hours single-session execution
- **End-state version**: `mujoco/elrobot_follower/VERSION` 0.1.0 → 0.2.2 (or
  0.3.0 per Open Decision U1)
- **End-state `make sim-test`**: 91 passed, 1 skipped (mjx)
- **End-state `upstream.prerequisites`**: 3/5 done (was 1/5 at Chunk 0)

*End of spec.*
