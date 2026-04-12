# Upstream Contribution to MuJoCo Menagerie

This document describes the procedure for contributing the ElRobot follower
arm MuJoCo model to [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie).

## Prerequisite Checklist

Tracked in `hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml`
under `upstream.prerequisites`.

| # | Prerequisite | Status | Notes |
|---|---|---|---|
| 1 | LICENSE file present | Done (v0.1.0) | Apache-2.0 |
| 2 | sysid_complete == true | **Pending** | Needs real-hardware measurement corpus in `measurements/sysid/` |
| 3 | Full CITATION.cff metadata | Done (v0.2.3) | Repository-only citation; add `preferred-citation` when paper exists |
| 4 | scene.xml wrapper (Menagerie convention) | Done (v0.2.1) | Lights + floor + skybox |
| 5 | Tests executable in isolation | Done (v0.2.2) | 20 passed, 1 skipped (MJX), zero norma_sim imports |

**Status: 4/5 done.** Only prerequisite 2 (real-hardware sysID) remains.
Do NOT open the upstream PR until sysID data lands and
`robot.yaml` `physics_baseline.sysid_complete` flips to `true`.

## File Mapping

When preparing the upstream PR, include only the files that match
Menagerie's robot package convention. NormaCore-specific files stay local.

| File | Include in upstream PR? | Reason |
|---|---|---|
| `elrobot_follower.xml` | Yes | Core MJCF model |
| `scene.xml` | Yes | Menagerie-convention scene wrapper |
| `elrobot_follower.png` | Yes | Render thumbnail (Menagerie convention) |
| `README.md` | Yes (rewritten) | Must be adapted for Menagerie audience |
| `CHANGELOG.md` | Yes | Version history |
| `LICENSE` | Yes | Apache-2.0 |
| `CITATION.cff` | Yes | Citation metadata |
| `assets/` | Yes | STL mesh files |
| `robot.yaml` | No | NormaCore-specific machine-readable identity |
| `VERSION` | No | NormaCore-specific semver tracking |
| `elrobot_follower.urdf` | No | Kinematic reference; Menagerie is MJCF-only |
| `tests/` | No | NormaCore engine-tier tests; Menagerie has its own test infra |
| `measurements/` | No | NormaCore sysID provenance data |
| `tools/` | No | NormaCore development tools (render script) |

## Submission Steps

### 1. Prepare the submission directory

```bash
# Create a clean staging area
mkdir -p /tmp/menagerie-submission/elrobot_follower

# Copy upstream-eligible files
cd hardware/elrobot/simulation/mujoco/elrobot_follower
cp elrobot_follower.xml scene.xml elrobot_follower.png \
   CHANGELOG.md LICENSE CITATION.cff \
   /tmp/menagerie-submission/elrobot_follower/
cp -r assets/ /tmp/menagerie-submission/elrobot_follower/
```

### 2. Adapt README.md

The package README contains NormaCore-specific sections ("Relationship to
NormaCore", "How to modify" with NormaCore-specific test commands). For
upstream, rewrite the README to:

- Describe the robot (8 actuators, 7 revolute + 1 tendon-mimic-parallel
  gripper, derived from `trs_so_arm100`)
- Include URDF-to-MJCF derivation notes (adapted from
  `measurements/menagerie_diff.md`)
- Reference `scene.xml` for visualization
- Remove all NormaCore-specific content

Use existing Menagerie READMEs (e.g., `trs_so_arm100/README.md`) as the
style template.

### 3. Fork and submit

```bash
# Fork google-deepmind/mujoco_menagerie on GitHub
# Clone your fork
git clone https://github.com/<your-fork>/mujoco_menagerie.git
cd mujoco_menagerie

# Copy the prepared directory
cp -r /tmp/menagerie-submission/elrobot_follower .

# Commit and push
git checkout -b add-elrobot-follower
git add elrobot_follower/
git commit -m "Add ElRobot follower arm model"
git push origin add-elrobot-follower

# Open PR on GitHub
```

PR description should include:
- Robot name and DOF count (8 actuated, 10 total including mimic)
- Physics baseline provenance (Menagerie `trs_so_arm100` v1.3)
- sysID status (complete/in-progress)
- Render thumbnail preview

### 4. Post-submission

After the Menagerie PR is merged:

1. Update `robot.yaml`:
   ```yaml
   upstream:
     candidate: mujoco_menagerie
     engaged: true
     uri: https://github.com/google-deepmind/mujoco_menagerie/tree/main/elrobot_follower
   ```
2. Bump VERSION and CHANGELOG to note the upstream landing
3. Consider keeping NormaCore as the development branch and periodically
   syncing with upstream (NormaCore has tests, measurements, and tools
   that upstream does not carry)
