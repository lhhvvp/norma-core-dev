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
├── elrobot_follower.urdf    ← URDF kinematic source of truth (used by test_urdf_parity)
├── scene.xml                ← Menagerie-style wrapper with floor + lights (use with mujoco.viewer)
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
    ├── conftest.py              ← shared fixtures (elrobot_mjcf_path, elrobot_sim)
    ├── test_elrobot_acceptance.py ← physics acceptance (Floors 1-6, pure mujoco)
    ├── test_mimic_gripper.py    ← P0 gripper mimic regression
    ├── test_scene_loadable.py   ← scene.xml smoke gate
    ├── test_urdf_parity.py      ← URDF↔MJCF consistency gate
    └── test_mjx_compat.py       ← MJX smoke test (placeholder)
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

This package contains the complete engine-tier test suite for the ElRobot
follower arm, including physics-acceptance tests (Floors 1-6 from MVP-2
spec S3.1). All tests use raw `mujoco` APIs and run without `norma_sim`
on PYTHONPATH.

The Norma application layer still maintains:
- `hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml`
  — the Norma-specific runtime wrapper that maps MJCF actuator names
  (`act_motor_*`) to client-facing IDs (`rev_motor_*`) with capability
  annotations.
- `software/sim-server/tests/integration/test_elrobot_manifest_sentinel.py`
  — a single sentinel test exercising the full
  `scene.yaml -> load_manifest -> MuJoCoWorld -> mj_step` pipeline.
- `software/sim-server/tests/integration/test_full_loop.py` and other
  Norma-specific integration tests that depend on `norma_sim`.

## Upstream contribution

This package is structured to eventually be contributed to
`mujoco_menagerie` as a sibling of `trs_so_arm100/`. Prerequisites are
tracked in `robot.yaml`'s `upstream.prerequisites` field. Summary: need
LICENSE (done), real-hardware sysID data, CITATION.cff, a scene.xml
wrapper matching Menagerie convention, and tests that run without a
NormaCore checkout.
