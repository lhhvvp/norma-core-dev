# Measurements

This directory holds physics-parameter provenance for the ElRobot
follower arm. It is a **sysID-ready slot**, not just documentation.

## Current contents

- `menagerie_diff.md` — Menagerie `trs_so_arm100`→ElRobot parameter
  adaptation record. Maps each Menagerie joint/actuator to its ElRobot
  analog, documents nearest-neighbor estimates for joints with no direct
  analog, and lists the four MVP-2 amendment policies (`forcerange` from
  URDF effort, `dampratio` not `kv`, explicit `ctrlrange` not
  `inheritrange`, `Gripper_Gear_v1_1` inertia floor).

## Future contents (post-MVP-3 Chunk 0)

When real-hardware sysID lands, this directory will also contain:

```
measurements/
├── raw/              ← CSV logs from the sysID rigs
├── fit/              ← fitted parameter values + fit residuals
├── plots/            ← visualization of fit quality per joint
├── sysid/            ← procedure scripts + notes
└── menagerie_diff.md ← kept as historical context
```

The `robot.yaml` `physics_baseline.sysid_complete` flag flips to `true`
when `fit/` contains a full per-joint dataset replacing Menagerie
inheritance.

## Workflow

1. Measure. (Separate chunk; procedure documented in `sysid/README.md`.)
2. Fit. (Separate chunk; fit script emits `fit/<joint_name>.json`.)
3. Update `../elrobot_follower.xml` with the fitted values.
4. Flip `robot.yaml` `sysid_complete: true`.
5. Bump `../VERSION` to `0.2.0` (minor — structural physics change).
6. Add a `../CHANGELOG.md` entry.
