#!/usr/bin/env python3
"""
gen.py — derive MJCF from world.yaml manifest + URDF.

Pipeline (see docs/superpowers/specs/2026-04-10-simulation-integration-design.md §9.3):
  1. Load manifest yaml
  2. Load URDF via mujoco.MjModel.from_xml_path (MuJoCo auto-detects <robot> root)
  3. Dump canonical MJCF via mujoco.mj_saveLastXML to a temp baseline file
  4. Parse the baseline MJCF, modify/add option+compiler+equality+actuator+default+worldbody
  5. Embed sha256(urdf + manifest) as an XML comment in the output MJCF
  6. Self-check consistency

Usage:
  python3 hardware/elrobot/simulation/worlds/gen.py
  python3 hardware/elrobot/simulation/worlds/gen.py --manifest other.yaml

Exit codes:
  0 = success
  1 = manifest / URDF load error (bad input)
  2 = self-check failure (internal inconsistency)
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

# Enforce Python version explicitly — the plan assumes 3.11+ for PEP 585 generics and dataclass features
if sys.version_info < (3, 11):
    sys.exit(f"gen.py requires Python 3.11+; got {sys.version_info}")

import yaml


def main() -> int:
    ap = argparse.ArgumentParser(description="Derive MJCF from world.yaml manifest + URDF")
    ap.add_argument(
        "--manifest",
        type=Path,
        default=Path(__file__).parent / "elrobot_follower.world.yaml",
        help="Path to world.yaml manifest",
    )
    args = ap.parse_args()

    try:
        manifest = load_manifest(args.manifest)
    except Exception as e:
        print(f"ERROR: failed to load manifest {args.manifest}: {e}", file=sys.stderr)
        return 1

    manifest_dir = args.manifest.parent.resolve()
    urdf_path = (manifest_dir / manifest["urdf_source"]).resolve()
    mjcf_path = (manifest_dir / manifest["mjcf_output"]).resolve()

    if not urdf_path.is_file():
        print(f"ERROR: URDF not found at {urdf_path}", file=sys.stderr)
        return 1

    # Compute source hash (urdf bytes + manifest yaml bytes) BEFORE we touch
    # anything on disk. This is what the runtime (Chunk 5) verifies.
    urdf_bytes = urdf_path.read_bytes()
    manifest_bytes = args.manifest.read_bytes()
    source_hash = hashlib.sha256(urdf_bytes + manifest_bytes).hexdigest()

    # Build MJCF via MuJoCo-powered pipeline (Task 1.4 fills this in)
    try:
        mjcf_root = build_mjcf(manifest, urdf_path, manifest_dir)
    except NotImplementedError:
        # Skeleton stage — re-raise so the test in Task 1.3 Step 2 catches it
        raise
    except Exception as e:
        print(f"ERROR: build_mjcf failed: {e}", file=sys.stderr)
        return 1

    # Write MJCF with source_hash comment (Task 1.4 fills this in)
    write_mjcf(mjcf_path, mjcf_root, source_hash)

    # Self-check (Task 1.5 fills this in; Task 1.3 skeleton is a no-op)
    try:
        run_self_check(manifest, mjcf_path)
    except Exception as e:
        print(f"ERROR: self-check failed: {e}", file=sys.stderr)
        return 2

    print(f"gen.py OK: {mjcf_path}")
    print(f"  urdf: {urdf_path} ({len(urdf_bytes)} bytes)")
    print(f"  source_hash=sha256:{source_hash[:16]}...")
    return 0


def load_manifest(path: Path) -> dict[str, Any]:
    with path.open() as f:
        return yaml.safe_load(f)


def build_mjcf(manifest: dict[str, Any], urdf_path: Path, manifest_dir: Path) -> ET.Element:
    """Build MJCF XML tree from manifest + URDF.

    Two-phase pipeline:
      1. MuJoCo loads URDF, we save the canonical MJCF to a baseline file
      2. We parse that baseline and modify/add our extension sections

    Returns the root <mujoco> element. Caller is responsible for writing it.
    """
    import mujoco  # imported lazily so CLI --help doesn't require the dep
    import tempfile

    # Phase 0: preprocess URDF for MuJoCo consumption.
    # The ElRobot URDF has two issues that prevent direct MuJoCo load:
    #   (a) mesh filenames include an "assets/" prefix, but MuJoCo's URDF
    #       loader strips the path and uses only the basename + meshdir. We
    #       inject a <mujoco><compiler meshdir="<abs>/assets"/></mujoco>
    #       extension element (URDF allows arbitrary sub-elements).
    #   (b) several mimic-joint bodies (Gripper_Gear_v1_1, fingers) have
    #       zero inertia which MuJoCo rejects. We use `inertiafromgeom=true`
    #       to recompute inertia from the mesh geometry, and
    #       `balanceinertia=true` as a safety net for any remaining
    #       positive-definiteness issues.
    # This is a URDF data-quality workaround, not a deviation from the
    # design: the URDF is the source of truth for topology + joint limits;
    # mass/inertia get recomputed from the meshes (which is also what
    # urdfpy-style converters do by default).
    urdf_tree_preproc = ET.parse(urdf_path)
    urdf_root_preproc = urdf_tree_preproc.getroot()
    abs_assets_dir = str((urdf_path.parent / "assets").resolve())
    mj_ext = ET.Element("mujoco")
    ET.SubElement(
        mj_ext, "compiler",
        meshdir=abs_assets_dir,
        discardvisual="false",
        inertiafromgeom="true",
        balanceinertia="true",
    )
    urdf_root_preproc.insert(0, mj_ext)
    fd, tmp_urdf_path = tempfile.mkstemp(suffix=".urdf", prefix="elrobot_")
    os.close(fd)
    try:
        urdf_tree_preproc.write(tmp_urdf_path)

        # Phase 1: URDF (preprocessed) → canonical MJCF via MuJoCo
        model = mujoco.MjModel.from_xml_path(tmp_urdf_path)
        baseline_path = manifest_dir / "_baseline_from_urdf.xml"
        mujoco.mj_saveLastXML(str(baseline_path), model)
    finally:
        if os.path.exists(tmp_urdf_path):
            os.unlink(tmp_urdf_path)

    try:
        tree = ET.parse(baseline_path)
        root = tree.getroot()
    finally:
        # Clean up the temp baseline regardless of parse success
        if baseline_path.exists():
            baseline_path.unlink()

    # Set the model attribute on the root so later steps can identify it
    root.set("model", manifest["world_name"])

    # Phase 2a: modify compiler element (override defaults from baseline)
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)  # MJCF convention: compiler is the first child
    compiler.set("angle", "radian")
    # meshdir is expressed relative to the MJCF output file
    compiler.set(
        "meshdir",
        os.path.relpath(str(urdf_path.parent / "assets"), str(manifest_dir)),
    )
    compiler.set("autolimits", "true")
    compiler.set("discardvisual", "false")

    # Phase 2b: modify option element
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    scene = manifest["scene"]
    option.set("timestep", str(scene["timestep"]))
    option.set("iterations", str(scene["iterations"]))
    option.set("solver", scene["solver"])
    option.set("gravity", " ".join(str(g) for g in scene["gravity"]))
    option.set("integrator", scene["integrator"])

    # Phase 2c: equality constraints (replaces URDF <mimic>, which MuJoCo drops)
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    for robot in manifest["robots"]:
        for act in robot["actuators"]:
            if act["capability"]["kind"] != "GRIPPER_PARALLEL":
                continue
            primary_joint = act["urdf_joint"]
            for mimic in act["gripper"]["mimic_joints"]:
                ET.SubElement(equality, "joint", {
                    "joint1": mimic["joint"],
                    "joint2": primary_joint,
                    "polycoef": f"0 {mimic['multiplier']} 0 0 0",
                })

    # Phase 2d: actuators
    # Walk the original URDF (not the MJCF baseline) to fetch revolute limits
    # authoritatively.
    urdf_tree = ET.parse(urdf_path)
    urdf_joints = {j.get("name"): j for j in urdf_tree.getroot().findall("joint")}

    actuator_elem = root.find("actuator")
    if actuator_elem is None:
        actuator_elem = ET.SubElement(root, "actuator")

    for robot in manifest["robots"]:
        for act in robot["actuators"]:
            cap = act["capability"]
            gains = act["actuator_gains"]

            # Determine ctrlrange + forcerange
            if cap["kind"] == "GRIPPER_PARALLEL":
                # MuJoCo controls the primary joint in rad; 0..1 normalization
                # happens at norma_sim (Chunk 5) before writing data.ctrl.
                ctrl_lo, ctrl_hi = act["gripper"]["primary_joint_range_rad"]
                force = cap["effort_limit"]
            else:
                # REVOLUTE_POSITION: limits from URDF
                joint_name = act["urdf_joint"]
                if joint_name not in urdf_joints:
                    raise ValueError(
                        f"URDF has no joint '{joint_name}' referenced by "
                        f"manifest actuator '{act['actuator_id']}'"
                    )
                limit = urdf_joints[joint_name].find("limit")
                if limit is None:
                    raise ValueError(
                        f"URDF joint '{joint_name}' has no <limit> element"
                    )
                ctrl_lo = float(limit.get("lower", 0))
                ctrl_hi = float(limit.get("upper", 0))
                force = float(limit.get("effort", 0))

            ET.SubElement(actuator_elem, "position", {
                "name": act["mjcf_actuator"],
                "joint": act["urdf_joint"],
                "kp": str(gains["kp"]),
                "kv": str(gains["kv"]),
                "ctrlrange": f"{ctrl_lo} {ctrl_hi}",
                "forcerange": f"-{force} {force}",
            })

    # Phase 2e: default classes for geom contypes
    default = root.find("default")
    if default is None:
        default = ET.SubElement(root, "default")
    # Only add our classes if not already present (idempotence)
    if default.find("./default[@class='arm_link']") is None:
        arm_link = ET.SubElement(default, "default", {"class": "arm_link"})
        ET.SubElement(arm_link, "geom", {
            "contype": "1", "conaffinity": "1", "friction": "0.9 0.005 0.0001",
        })
    if default.find("./default[@class='gripper_finger']") is None:
        gripper_cls = ET.SubElement(default, "default", {"class": "gripper_finger"})
        ET.SubElement(gripper_cls, "geom", {
            "contype": "2", "conaffinity": "3", "friction": "1.5 0.05 0.001",
        })

    # Phase 2f: worldbody extras (lights + floor)
    # The baseline MJCF already has a <worldbody> with the robot bodies; we
    # append our extras to it without touching the existing children.
    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")

    existing_light_names = {
        l.get("name") for l in worldbody.findall("light")
    }
    for light in manifest["scene_extras"].get("lights", []):
        if light["name"] in existing_light_names:
            continue  # idempotent
        ET.SubElement(worldbody, "light", {
            "name": light["name"],
            "pos": " ".join(str(v) for v in light["pos"]),
            "dir": " ".join(str(v) for v in light["dir"]),
        })

    floor = manifest["scene_extras"].get("floor")
    if floor and worldbody.find("./geom[@name='floor']") is None:
        ET.SubElement(worldbody, "geom", {
            "name": "floor",
            "type": "plane",
            "size": " ".join(str(v) for v in floor["size"]),
            "material": floor["material"],
        })

    # Phase 2g: ensure referenced textures/materials exist in <asset>.
    # The floor references material="grid" — define it if not already
    # present (idempotent).
    if floor:
        asset = root.find("asset")
        if asset is None:
            asset = ET.SubElement(root, "asset")
        if asset.find(f"./material[@name='{floor['material']}']") is None:
            # Standard MuJoCo grid texture + material pairing
            ET.SubElement(asset, "texture", {
                "name": floor["material"],
                "type": "2d",
                "builtin": "checker",
                "rgb1": ".1 .2 .3",
                "rgb2": ".2 .3 .4",
                "width": "512",
                "height": "512",
                "mark": "cross",
                "markrgb": ".8 .8 .8",
            })
            ET.SubElement(asset, "material", {
                "name": floor["material"],
                "texture": floor["material"],
                "texrepeat": "1 1",
                "texuniform": "true",
                "reflectance": ".2",
            })

    return root  # Phase 2 edits happen in Steps 1b-1e


def write_mjcf(path: Path, mjcf_elem: ET.Element, source_hash: str) -> None:
    """Write MJCF with source_hash embedded as an XML comment inside <mujoco>.

    The runtime (Chunk 5's norma_sim) reads this comment on load and verifies
    sha256(urdf + world.yaml) matches; mismatch means gen.py needs re-running.
    """
    from xml.dom import minidom
    rough_bytes = ET.tostring(mjcf_elem, encoding="unicode")
    reparsed = minidom.parseString(rough_bytes)
    pretty = reparsed.toprettyxml(indent="  ", encoding=None)

    header_comment = (
        f"<!-- norma-sim: generated by gen.py\n"
        f"     source_hash=sha256:{source_hash}\n"
        f"     generator_version=1.0 -->\n"
    )
    # Inject the comment immediately after the opening <mujoco ...> tag line.
    lines = pretty.splitlines(keepends=True)
    out_lines: list[str] = []
    injected = False
    for line in lines:
        out_lines.append(line)
        if not injected and line.strip().startswith("<mujoco"):
            out_lines.append("  " + header_comment)
            injected = True
    with path.open("w") as f:
        f.writelines(out_lines)


def run_self_check(manifest: dict[str, Any], mjcf_path: Path) -> None:
    """Placeholder; Task 1.5 fleshes this out. Task 1.3 skeleton is a no-op."""
    pass


if __name__ == "__main__":
    sys.exit(main())
