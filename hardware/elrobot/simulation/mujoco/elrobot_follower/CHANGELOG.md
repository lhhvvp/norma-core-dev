# ElRobot Physics Model CHANGELOG

Follows a subset of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versioning is semver, independent of the `software/` crates.

## [0.2.3] — 2026-04-12

### Added

- `CITATION.cff` — CFF 1.2.0 repository-only citation metadata. Upstream
  prerequisite 3/5 now satisfied. No `preferred-citation` block yet (no
  associated paper); add one when a paper is published.
- `elrobot_follower.png` — 640x480 render thumbnail matching Menagerie's
  visual convention (elevated front-left view, home pose, floor visible).
  Generated headlessly via `tools/render_thumbnail.py`.
- `tools/render_thumbnail.py` — deterministic headless render script using
  `mujoco.Renderer` + Pillow. Regenerates the thumbnail after physics or
  geometry changes: `python3 tools/render_thumbnail.py` from package root.
- `docs/upstream-to-menagerie.md` (in NormaCore docs, not this package) —
  contribution guide with prerequisite checklist, file mapping table, and
  step-by-step submission procedure for mujoco_menagerie.

### Changed

- `robot.yaml`: `version.current` 0.2.2 -> 0.2.3; mark `Full CITATION.cff
  metadata` prerequisite as done. Upstream prerequisites now **4/5 done**
  (only `sysid_complete == true` remains).
- `README.md`: updated file tree to include `CITATION.cff`,
  `elrobot_follower.png`, and `tools/` directory.

### Physics gate results (at this version)

- No physics changes; all gates identical to v0.2.2.
- Engine-tier package tests: 20 passed + 1 skipped (mjx if absent).

### Integration context

- MVP-4 A4 commit: (this commit)
- Spec: `docs/superpowers/specs/2026-04-12-mvp4-a4-upstream-prep-design.md`

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
- `tests/test_mimic_gripper.py:1`: unicode star -> `[P0]` ASCII (Chunk 0
  remainder Item 4).
- `README.md`: rewrote "Relationship to NormaCore" section — engine-tier
  acceptance tests now live in this package; sim-server retains only the
  manifest-pipeline sentinel and Norma-specific integration tests.

### Removed

- `software/sim-server/tests/integration/test_elrobot_acceptance.py`
  (from sim-server, not this package) — the original norma_sim-coupled
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

- NormaCore main HEAD before this version: `08475e8` on main (2026-04-12,
  plan-review-fixes commit immediately preceding Chunk 1)
- MVP-3 Engine Package Completion Chunk 1 commit: `51ecccd` on main (2026-04-12)
- Roadmap spec: `docs/superpowers/specs/2026-04-12-mvp3-foundation-roadmap-design.md`

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
  which stays in sim-server because it depends on `norma_sim` (removed
  in 0.2.2 — physics tests moved to engine package, manifest-layer
  coverage preserved by sentinel).
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
- Assets (`*.stl`) still live outside this package at v0.1.0 (under the
  simulation tier's top-level `assets/` directory). MJCF uses
  `meshdir="../../assets"`. **Resolved in v0.2.0 (MVP-3 Chunk 1)** — see
  [0.2.0] entry above.
- No `scene.xml` wrapper with lights/floor. Running
  `python -m mujoco.viewer hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.xml`
  will show the arm without a floor. A future chunk will add a Menagerie-
  style `scene.xml` for visual parity.
- `CITATION.cff` is not present. Required for upstream contribution; a
  future chunk will add it once the real-hardware sysID corpus lands.

### Integration context

- NormaCore MVP-2 merge commit: `93c1597` on `main` (2026-04-12)
- MVP-3 Chunk 0 commit: 6ef605b on main (2026-04-12)
- Chunk 0 spec: `docs/superpowers/specs/2026-04-12-mvp3-first-class-mjcf-design.md`
