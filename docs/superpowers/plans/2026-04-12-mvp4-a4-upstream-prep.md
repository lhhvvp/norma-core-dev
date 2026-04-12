# MVP-4 A4: Upstream Prep Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring the ElRobot MuJoCo package to 4/5 upstream prerequisites by adding CITATION.cff, a Menagerie-convention render thumbnail, and an upstream contribution guide.

**Architecture:** Single atomic commit — all changes are metadata/documentation with no behavioral impact. The render script uses `mujoco.Renderer` (verified available at mujoco 3.6.0) + Pillow for headless PNG generation.

**Tech Stack:** Python, mujoco 3.6.0 (`mujoco.Renderer`, `MjvCamera`), Pillow (`PIL.Image`).

**Spec:** `docs/superpowers/specs/2026-04-12-mvp4-a4-upstream-prep-design.md`

**Baselines (unchanged through this plan — no test/physics changes):**
- `make sim-test`: 94 passed, 1 skipped
- Package tests: 20 passed, 1 skipped
- VERSION: 0.2.2

**Package root (abbreviated as `PKG/` in this plan):**
`hardware/elrobot/simulation/mujoco/elrobot_follower/`

---

## Chunk 1: All Deliverables (Single Commit)

### Task 1: Create CITATION.cff

**Files:**
- Create: `PKG/CITATION.cff`

- [ ] **Step 1: Write CITATION.cff**

```yaml
cff-version: 1.2.0
message: "If you use this robot model, please cite it as below."
type: software
title: "ElRobot Follower Arm — MuJoCo Physics Model"
version: 0.2.3
license: Apache-2.0
date-released: "2026-04-12"
authors:
  - given-names: Yuan
```

- [ ] **Step 2: Validate structure**

Run: `python3 -c "import yaml; d = yaml.safe_load(open('hardware/elrobot/simulation/mujoco/elrobot_follower/CITATION.cff')); print(d['cff-version'], d['title'], d['version'])"`
Expected: `1.2.0 ElRobot Follower Arm — MuJoCo Physics Model 0.2.3`

---

### Task 2: Create render script

**Files:**
- Create: `PKG/tools/render_thumbnail.py`

The API (verified on mujoco 3.6.0):
- `mujoco.Renderer(model, height=480, width=640)` → renderer
- `mujoco.MjvCamera()` → camera with `lookat`, `distance`, `azimuth`, `elevation` fields
- `renderer.update_scene(data, camera=cam)` → set scene
- `renderer.render()` → `numpy.ndarray` shape `(480, 640, 3)` dtype `uint8`
- `PIL.Image.fromarray(pixels).save(path)` → PNG file

- [ ] **Step 1: Create `tools/` directory**

Run: `mkdir -p hardware/elrobot/simulation/mujoco/elrobot_follower/tools`

- [ ] **Step 2: Write the render script**

```python
#!/usr/bin/env python3
"""Render elrobot_follower.png thumbnail for Menagerie-convention packaging.

Usage (from the package root):
    python3 tools/render_thumbnail.py

Outputs elrobot_follower.png in the package root directory.
Requires: mujoco >= 3.0, Pillow.
"""
from pathlib import Path
import sys

try:
    import mujoco
except ImportError:
    sys.exit("mujoco not installed. Install with: pip install mujoco")

try:
    from PIL import Image
except ImportError:
    sys.exit("Pillow not installed. Install with: pip install Pillow")

# Resolve paths relative to this script's location (tools/ -> package root)
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
SCENE_XML = PACKAGE_ROOT / "scene.xml"
OUTPUT_PNG = PACKAGE_ROOT / "elrobot_follower.png"

# Image dimensions (standard Menagerie thumbnail size)
WIDTH = 640
HEIGHT = 480

# Camera: elevated front-left view, arm centered in frame at home pose.
# Tuned for the ElRobot follower arm with floor visible.
CAMERA_AZIMUTH = -120.0
CAMERA_ELEVATION = -25.0
CAMERA_DISTANCE = 1.0
CAMERA_LOOKAT = (0.0, 0.0, 0.2)


def main():
    if not SCENE_XML.exists():
        sys.exit(f"scene.xml not found at {SCENE_XML}")

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)

    cam = mujoco.MjvCamera()
    cam.azimuth = CAMERA_AZIMUTH
    cam.elevation = CAMERA_ELEVATION
    cam.distance = CAMERA_DISTANCE
    cam.lookat[:] = CAMERA_LOOKAT

    renderer.update_scene(data, camera=cam)
    pixels = renderer.render()

    img = Image.fromarray(pixels)
    img.save(str(OUTPUT_PNG))
    print(f"Saved {OUTPUT_PNG} ({WIDTH}x{HEIGHT})")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Run the render script**

Run: `cd hardware/elrobot/simulation/mujoco/elrobot_follower && python3 tools/render_thumbnail.py && cd /home/yuan/proj/norma-core`
Expected: `Saved .../elrobot_follower.png (640x480)`

- [ ] **Step 4: Verify PNG exists and is non-trivial**

Run: `ls -la hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.png`
Expected: file exists, size > 10KB (a real render, not a blank image)

---

### Task 3: Create contribution guide

**Files:**
- Create: `docs/upstream-to-menagerie.md`

- [ ] **Step 1: Write the contribution guide**

```markdown
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
```

- [ ] **Step 2: Verify the file exists**

Run: `test -f docs/upstream-to-menagerie.md && echo OK`
Expected: `OK`

---

### Task 4: Bump metadata

**Files:**
- Modify: `PKG/VERSION`
- Modify: `PKG/CHANGELOG.md`
- Modify: `PKG/robot.yaml`
- Modify: `PKG/README.md`

- [ ] **Step 1: Bump VERSION**

Change `0.2.2` to `0.2.3`.

- [ ] **Step 2: Add CHANGELOG [0.2.3] entry**

Insert a new version block between the header and `## [0.2.2]`. Replace:
```markdown
## [0.2.2] — 2026-04-12
```

With:
```markdown
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

- NormaCore main HEAD before this version: `0c74df3` on main
  (2026-04-12, Chunk 3 commit 2 immediately preceding A4)
- MVP-4 A4 commit: (this commit)
- Spec: `docs/superpowers/specs/2026-04-12-mvp4-a4-upstream-prep-design.md`

## [0.2.2] — 2026-04-12
```

- [ ] **Step 3: Update robot.yaml**

Change `version.current` from `"0.2.2"` to `"0.2.3"`.

Change the CITATION.cff prerequisite line from:
```yaml
    - Full CITATION.cff metadata
```
to:
```yaml
    - Full CITATION.cff metadata (done at 0.2.3)
```

- [ ] **Step 4: Update README.md file tree**

Replace:
```
└── tests/                   ← engine-level validation
    ├── conftest.py              ← shared fixtures (elrobot_mjcf_path, elrobot_sim)
```

With:
```
├── tools/                   ← development utilities
│   └── render_thumbnail.py  ← headless PNG render (mujoco.Renderer + Pillow)
└── tests/                   ← engine-level validation
    ├── conftest.py              ← shared fixtures (elrobot_mjcf_path, elrobot_sim)
```

Also add these two lines to the file listing (after `CHANGELOG.md` line, before `assets/`):

Insert after `├── CHANGELOG.md`:
```
├── CITATION.cff                ← CFF 1.2.0 citation metadata (upstream prereq)
├── elrobot_follower.png        ← render thumbnail (Menagerie convention)
```

---

### Task 5: Verify all success criteria + commit

- [ ] **Step 1: SC#1 — CITATION.cff valid**

Run: `python3 -c "import yaml; d = yaml.safe_load(open('hardware/elrobot/simulation/mujoco/elrobot_follower/CITATION.cff')); assert d['cff-version'] == '1.2.0'; assert d['version'] == '0.2.3'; print('CITATION.cff valid')"`
Expected: `CITATION.cff valid`

- [ ] **Step 2: SC#2 — PNG exists and non-zero**

Run: `test -s hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.png && echo "PNG OK"`
Expected: `PNG OK`

- [ ] **Step 3: SC#3 — Render script works**

Run: `cd hardware/elrobot/simulation/mujoco/elrobot_follower && python3 tools/render_thumbnail.py && cd /home/yuan/proj/norma-core`
Expected: `Saved .../elrobot_follower.png (640x480)`

- [ ] **Step 4: SC#4 — Contribution guide exists**

Run: `test -f docs/upstream-to-menagerie.md && echo OK`
Expected: `OK`

- [ ] **Step 5: SC#5 — VERSION is 0.2.3**

Run: `cat hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION`
Expected: `0.2.3`

- [ ] **Step 6: SC#6 — robot.yaml updated**

Run: `grep 'CITATION.cff' hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml`
Expected: contains `(done at 0.2.3)`

- [ ] **Step 7: SC#7 — make sim-test unchanged**

Run: `make sim-test 2>&1 | tail -3`
Expected: `94 passed, 1 skipped`

- [ ] **Step 8: SC#8 — Self-containment unchanged**

Run: `cp -r hardware/elrobot/simulation/mujoco/elrobot_follower /tmp/elrobot-test && cd /tmp/elrobot-test && python3 -m pytest tests/ -q 2>&1 | tail -3; cd /home/yuan/proj/norma-core; rm -rf /tmp/elrobot-test`
Expected: `20 passed, 1 skipped`

- [ ] **Step 9: SC#9 — git status**

Run: `git status`
Expected: only the files we're about to commit are new/modified.

- [ ] **Step 10: Commit**

```bash
git add \
  hardware/elrobot/simulation/mujoco/elrobot_follower/CITATION.cff \
  hardware/elrobot/simulation/mujoco/elrobot_follower/elrobot_follower.png \
  hardware/elrobot/simulation/mujoco/elrobot_follower/tools/render_thumbnail.py \
  hardware/elrobot/simulation/mujoco/elrobot_follower/VERSION \
  hardware/elrobot/simulation/mujoco/elrobot_follower/CHANGELOG.md \
  hardware/elrobot/simulation/mujoco/elrobot_follower/robot.yaml \
  hardware/elrobot/simulation/mujoco/elrobot_follower/README.md \
  docs/upstream-to-menagerie.md
git commit -m "$(cat <<'EOF'
mvp4-a4: upstream prep — CITATION.cff + thumbnail + contribution guide

Add CITATION.cff (CFF 1.2.0, repository-only citation).
Add elrobot_follower.png render thumbnail (640x480, Menagerie convention).
Add tools/render_thumbnail.py for deterministic headless regeneration.
Add docs/upstream-to-menagerie.md contribution guide with prerequisite
checklist, file mapping, and submission steps.

Bump VERSION 0.2.2 -> 0.2.3. Update CHANGELOG, robot.yaml (mark
CITATION.cff prereq done -> 4/5 upstream prerequisites met),
README (file tree).

make sim-test: 94 passed, 1 skipped (unchanged).
Only sysid_complete remains before upstream submission.

Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
EOF
)"
```
