# Menagerie Vendor Import

This directory contains files vendored verbatim from
[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie).

## Source

- **Repository:** https://github.com/google-deepmind/mujoco_menagerie
- **Commit SHA:** `c771fb04055d805f20db0eab6cb20b67555887d0`
- **Import date:** 2026-04-11
- **License:** Apache License 2.0 (see `LICENSE` in this directory)

## Vendored Content

- `trs_so_arm100/` — The Robot Studio SO-ARM100 MJCF model, copied unmodified.
  Version 1.3 "Standard Open Arm-100 5DOF" (last tuning 2025-06-09).
  Used as physics parameter reference for the ElRobot MVP-2 sim (see
  `docs/superpowers/specs/2026-04-11-mvp2-menagerie-walking-skeleton-design.md`).
  - Main MJCF: `trs_so_arm100/so_arm100.xml` (5 revolute + 1 gripper = 6 actuators)
  - Scene wrapper: `trs_so_arm100/scene.xml` (adds lighting + groundplane)
  - Assets: 18 STL meshes in `trs_so_arm100/assets/`
  - Requires MuJoCo >= 3.1.6

## Modifications

**None.** All files under `trs_so_arm100/` are byte-identical to the Menagerie
source at the commit above. If you need to modify MJCF content for ElRobot's
8-joint adaptation, create files at `hardware/elrobot/simulation/elrobot_follower.xml`
instead (see Chunk 5 of the MVP-2 plan).

## Update Procedure

To refresh vendored content from a newer Menagerie commit:

1. `git clone https://github.com/google-deepmind/mujoco_menagerie /tmp/menagerie`
2. Verify `trs_so_arm100/` still exists and license is still Apache 2.0
3. `rm -rf hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100`
4. `cp -r /tmp/menagerie/trs_so_arm100 hardware/elrobot/simulation/vendor/menagerie/`
5. `cp /tmp/menagerie/LICENSE hardware/elrobot/simulation/vendor/menagerie/LICENSE`
6. Update this VENDOR.md with the new commit SHA and date
7. Run the Phase 1 walking skeleton tests (`pytest software/sim-server/tests/integration/test_menagerie_walking_skeleton.py`) to verify nothing broke
