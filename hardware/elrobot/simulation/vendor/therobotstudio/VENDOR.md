# TheRobotStudio SO-ARM100 Vendor Import

This directory contains files vendored verbatim from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100).

## Source

- **Repository:** https://github.com/TheRobotStudio/SO-ARM100
- **Subdirectory:** `Simulation/SO101/`
- **Commit SHA:** `fda892cba81032c46c40976a48c9ceadbf40a9ca`
- **Commit date:** 2026-02-26
- **Import date:** 2026-04-12
- **License:** Apache License 2.0 (see `LICENSE` in this directory)

## Vendored Content

- `SO101/` — MuJoCo (MJCF) and URDF simulation files for the SO-101 robot
  arm (6-DOF, successor to SO-100). Includes:
  - `so101_new_calib.xml` / `so101_old_calib.xml` — two calibration variants
  - `so101_new_calib.urdf` / `so101_old_calib.urdf` — corresponding URDFs
  - `joints_properties.xml` — **real STS3215 servo parameters** (kp=17.8,
    armature=0.028, damping=0.60, frictionloss=0.052, backlash=0.5 deg).
    Adapted from the Open Duck Mini project.
  - `scene.xml` — MuJoCo scene wrapper
  - `assets/` — 26 mesh files (.stl + .part)

## Why This Vendor Exists (alongside Menagerie)

| Aspect | Menagerie (`vendor/menagerie/`) | TheRobotStudio (`vendor/therobotstudio/`) |
|---|---|---|
| Model | trs_so_arm100 (SO-100, 5DOF+gripper) | SO101 (SO-101, 6DOF) |
| Motor params | Experience-based (kp=50, armature=0.1) | STS3215 measured (kp=17.8, armature=0.028) |
| Backlash | Not modeled | Modeled (0.5 deg class) |
| Calibration | Single | New + old (dual) |
| Purpose in NormaCore | Walking skeleton regression baseline | Real servo parameter reference |

The Menagerie vendor remains the regression baseline for `test_menagerie_walking_skeleton.py`.
This vendor provides more accurate motor dynamics for future SO-100 first-class package work.

## Modifications

**None.** All files under `SO101/` are byte-identical to the source at
the commit above.

## Known Limitations

- **Gripper cannot close:** Same issue as Menagerie — the gripper is modeled
  as a single revolute joint (range -0.174 ~ 1.745 rad) without the real
  parallel-jaw gear/linkage mechanism. The jaw pads never make contact at
  any joint angle. The upstream README acknowledges this: "In LeRobot, the
  gripper is represented as a linear joint, though this mapping is not yet
  reflected in the current URDF and MuJoCo files."
- **Backlash class defined but not applied:** `joints_properties.xml` defines
  a `backlash` class with 0.5 deg range, but no joint in the MJCF uses it.

## Update Procedure

1. `git clone --depth 1 https://github.com/TheRobotStudio/SO-ARM100.git /tmp/so-arm100`
2. `rm -rf hardware/elrobot/simulation/vendor/therobotstudio/SO101`
3. `cp -r /tmp/so-arm100/Simulation/SO101 hardware/elrobot/simulation/vendor/therobotstudio/SO101`
4. `cp /tmp/so-arm100/LICENSE hardware/elrobot/simulation/vendor/therobotstudio/LICENSE`
5. Update this VENDOR.md with the new commit SHA and date
6. Run `make sim-test` to verify nothing broke
7. `rm -rf /tmp/so-arm100`
