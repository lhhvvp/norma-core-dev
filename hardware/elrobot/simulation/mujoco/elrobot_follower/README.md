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

The STL mesh assets live inside this package at `assets/` (moved in MVP-3
Chunk 1, commit `<TBD-mvp3-chunk1>`). The MJCF's `meshdir="assets"`
resolves to them. The package is now self-contained: it can be copied to
any location (e.g. `/tmp/elrobot-test`) and `pytest tests/` runs cleanly
without needing the rest of the NormaCore checkout on disk.

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
