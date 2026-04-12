# MVP-4 A4: Upstream Prep (Menagerie-Match) — Design Spec

## Goal

Bring the `mujoco/elrobot_follower/` package to 4/5 upstream prerequisites
by adding CITATION.cff, a Menagerie-convention render thumbnail, and a
contribution guide documenting the submission procedure. After A4, the only
remaining blocker is real-hardware sysID data (`sysid_complete == true`).

## Current State

Package: `hardware/elrobot/simulation/mujoco/elrobot_follower/` at v0.2.2.

Upstream prerequisites (`robot.yaml`):
- ✅ LICENSE file present (done at 0.1.0)
- ✅ scene.xml wrapper with lights/floor (done at 0.2.1)
- ✅ Passing tests executable in isolation (done at 0.2.2)
- ❌ Full CITATION.cff metadata — **this chunk**
- ❌ sysid_complete == true — needs hardware, out of scope

Reference: vendored Menagerie `trs_so_arm100` at
`hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/` shows the
standard Menagerie robot file pattern: `<robot>.xml`, `scene.xml`,
`README.md`, `CHANGELOG.md`, `LICENSE`, `assets/`, `<robot>.png`.

## Deliverables

### 1. CITATION.cff

**File:** `mujoco/elrobot_follower/CITATION.cff`

Repository-only citation (no associated paper). Follows CFF 1.2.0 schema.

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

Design decisions:
- No `repository` field — the NormaCore repo is local-only (per
  `git_topology` memory: "main 永不 push"). The field gets added when the
  package is contributed to Menagerie or published elsewhere.
- No `preferred-citation` block — no paper exists yet. Can be added later
  when a paper is published.
- `version` field tracks the `VERSION` file and must be bumped in lockstep.
- Minimal author entry (given-names only) — family-names, email, orcid can
  be added later if needed for upstream submission.

### 2. Render Thumbnail

**File:** `mujoco/elrobot_follower/elrobot_follower.png`

Menagerie robots include a `<robot>.png` thumbnail (e.g.,
`trs_so_arm100/so_arm100.png`). This is the visual identity of the robot
in the Menagerie catalog.

**Specifications:**
- Resolution: 640×480 pixels
- Camera: slightly elevated, front-left angle (matches Menagerie visual
  convention — robot centered in frame, floor visible, arm at home pose)
- Source: `scene.xml` (includes floor + lights)
- Format: PNG, RGB
- Generated headlessly via `mujoco.Renderer` — no display required

### 3. Render Script

**File:** `mujoco/elrobot_follower/tools/render_thumbnail.py`

Self-contained script that regenerates the thumbnail:

```bash
cd hardware/elrobot/simulation/mujoco/elrobot_follower
python3 tools/render_thumbnail.py
# → writes elrobot_follower.png to package root
```

**Implementation:**
- Uses `mujoco.MjModel.from_xml_path("scene.xml")` + `mujoco.Renderer`
- Camera parameters hardcoded for the ElRobot arm (azimuth, elevation,
  distance, lookat) — tuned once to produce a good composition
- No external dependencies beyond `mujoco` (uses `mujoco.Renderer`'s
  built-in pixel buffer → write raw PNG via Python stdlib or Pillow)
- Deterministic: same script always produces the same image (no random
  state, fixed camera, home pose)

**Why a script instead of a manual screenshot:**
- Reproducible after physics parameter changes
- Can run in CI
- Menagerie thumbnails are consistent renders; ad-hoc screenshots vary

**Why inside the package (`tools/`) instead of external:**
- Travels with the package — `cp -r` self-containment includes it
- The script references `scene.xml` by relative path, so it only works
  from within the package directory

### 4. Contribution Guide

**File:** `docs/upstream-to-menagerie.md`

NormaCore-level document (not inside the package) describing the procedure
for submitting the ElRobot model to mujoco_menagerie.

**Sections:**

**4a. Prerequisite Checklist** — mirrors `robot.yaml` `upstream.prerequisites`
with current status and what each item means. Updated whenever a prerequisite
is completed.

**4b. File Mapping** — which package files go upstream vs stay NormaCore-only:

| File | Upstream? | Reason |
|---|---|---|
| `elrobot_follower.xml` | ✅ | Core MJCF |
| `scene.xml` | ✅ | Menagerie convention |
| `elrobot_follower.png` | ✅ | Menagerie convention |
| `README.md` | ✅ | Rewritten for Menagerie audience (no NormaCore references) |
| `CHANGELOG.md` | ✅ | Version history |
| `LICENSE` | ✅ | Apache-2.0 |
| `CITATION.cff` | ✅ | Citation metadata |
| `assets/` | ✅ | STL meshes |
| `robot.yaml` | ❌ | NormaCore-specific machine-readable identity |
| `VERSION` | ❌ | NormaCore-specific semver tracking |
| `elrobot_follower.urdf` | ❌ | Kinematic reference, not needed by Menagerie |
| `tests/` | ❌ | NormaCore engine-tier tests; Menagerie has its own test infra |
| `measurements/` | ❌ | NormaCore sysID provenance |
| `tools/` | ❌ | NormaCore development tools |

**4c. Submission Steps:**
1. Fork `google-deepmind/mujoco_menagerie`
2. Create `elrobot_follower/` directory
3. Copy upstream-eligible files (per mapping table)
4. Adapt README.md for Menagerie audience (remove NormaCore-specific
   sections like "Relationship to NormaCore", "How to modify")
5. Open PR with description of the robot (DOF count, actuator layout,
   physics baseline provenance, sysID status)
6. Address reviewer feedback

**4d. Post-Submission:**
- Update `robot.yaml` `upstream.engaged: true`
- Add `upstream.uri` pointing to the Menagerie directory
- Keep NormaCore package in sync with upstream (pull Menagerie changes,
  push NormaCore-specific changes via separate PRs)

### 5. Metadata Updates

**VERSION:** `0.2.2` → `0.2.3` (patch — no API, no physics, no consumer
impact; metadata-only addition).

**CHANGELOG:** `[0.2.3]` entry documenting CITATION.cff, thumbnail, render
script additions.

**robot.yaml:**
- `version.current`: `"0.2.2"` → `"0.2.3"`
- `upstream.prerequisites` list after update:
  ```yaml
  prerequisites:
    - LICENSE file present (done at 0.1.0)
    - sysid_complete == true
    - Full CITATION.cff metadata (done at 0.2.3)
    - scene.xml wrapper with lights/floor (Menagerie convention; done at 0.2.1)
    - Passing tests executable in isolation (done at 0.2.2)
  ```
- Upstream prereq status becomes **4/5 done** (only `sysid_complete` remains)

**README.md:** Update file tree to include `CITATION.cff`,
`elrobot_follower.png`, and `tools/` directory.

## Boundary

- ❌ No physics changes
- ❌ No test changes (no new tests — CITATION.cff and PNG are metadata)
- ❌ No pyproject.toml (Menagerie doesn't use it)
- ❌ No upstream PR submission (sysID still blocks)
- ❌ No robot.yaml schema redesign (Chunk 0 item 5, deferred)
- ❌ No changes to sim-server or Norma application layer
- ❌ No README.md rewrite for Menagerie audience (that happens at
  submission time, per contribution guide Section 4c Step 4)

## Success Criteria

1. `cat mujoco/elrobot_follower/CITATION.cff` parses as valid CFF 1.2.0
   (verify with `cffconvert --validate` if available, or manual inspection)
2. `mujoco/elrobot_follower/elrobot_follower.png` exists and is a
   non-zero-size PNG file
3. `python3 mujoco/elrobot_follower/tools/render_thumbnail.py` regenerates
   the thumbnail without errors
4. `docs/upstream-to-menagerie.md` exists with all 4 sections
5. `cat mujoco/elrobot_follower/VERSION` → `0.2.3`
6. `robot.yaml` shows CITATION.cff prereq as done, version 0.2.3
7. `make sim-test` → `94 passed, 1 skipped` (unchanged — no test changes)
8. `cp -r mujoco/elrobot_follower /tmp/elrobot-test && cd /tmp/elrobot-test && pytest tests/ -v` → `20 passed, 1 skipped` (unchanged)
9. `git status` clean

## Commit Strategy

Single atomic commit (all changes are metadata/documentation — no behavioral
split needed, unlike Chunk 3's add-then-delete topology).

## Risks

1. **mujoco.Renderer availability**: `mujoco.Renderer` was introduced in
   mujoco 3.0. If the installed version is older, the render script will
   fail. Mitigation: check `mujoco.__version__` in the script and print a
   helpful error.
2. **Camera angle tuning**: the hardcoded camera parameters may not produce
   a good composition on the first try. Mitigation: the render script is
   designed to be re-run after parameter tweaks — iterate until the
   thumbnail looks good.
3. **PNG write dependency**: `mujoco.Renderer.render()` returns a numpy
   array (no built-in file-write method). Mitigation: use
   `PIL.Image.fromarray(pixels).save(...)` as the primary path (Pillow is
   widely available); fall back to `matplotlib.pyplot.imsave` if Pillow is
   absent.
