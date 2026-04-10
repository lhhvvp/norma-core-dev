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

    # Phase 2c: mimic-joint coupling via <tendon><fixed> + <equality><tendon>.
    # (deviation from plan §1.4 Step 1c: the plan used <equality><joint
    # polycoef="..."/>, which MuJoCo's constraint solver does not enforce
    # reliably when the two joints are of different types — e.g. hinge
    # primary + prismatic mimic, as in this gripper — because the Jacobian
    # entries span disparate units (rad vs m). Tendon-based equality is
    # MuJoCo's canonical way to implement URDF <mimic> because a <fixed>
    # tendon reduces the problem to a single scalar length that the
    # equality solver can enforce strongly.)
    #
    # For each mimic j1 = multiplier * primary, we build:
    #   <tendon><fixed name="mimic_<j1>">
    #     <joint joint="<j1>" coef="1"/>
    #     <joint joint="<primary>" coef="-multiplier"/>
    #   </fixed></tendon>
    #   <equality><tendon tendon1="mimic_<j1>" solref="0.002 1"
    #                     solimp="0.99 0.999 0.0001"/></equality>
    # The tendon length = qpos[j1] - multiplier*qpos[primary]; constraining
    # it to 0 enforces qpos[j1] = multiplier * qpos[primary].
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    tendon = root.find("tendon")
    if tendon is None:
        tendon = ET.SubElement(root, "tendon")
    for robot in manifest["robots"]:
        for act in robot["actuators"]:
            if act["capability"]["kind"] != "GRIPPER_PARALLEL":
                continue
            primary_joint = act["urdf_joint"]
            for mimic in act["gripper"]["mimic_joints"]:
                tendon_name = f"mimic_{mimic['joint']}"
                fixed = ET.SubElement(tendon, "fixed", {"name": tendon_name})
                ET.SubElement(fixed, "joint", {
                    "joint": mimic["joint"], "coef": "1",
                })
                ET.SubElement(fixed, "joint", {
                    "joint": primary_joint,
                    "coef": str(-float(mimic["multiplier"])),
                })
                ET.SubElement(equality, "tendon", {
                    "tendon1": tendon_name,
                    "solref": "0.002 1",
                    "solimp": "0.99 0.999 0.0001 0.5 2",
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
    """Verify manifest and generated MJCF are consistent.

    Raises ValueError with a clear message on any mismatch. This runs
    after write_mjcf so self-check can inspect the serialized file.
    """
    mjcf_tree = ET.parse(mjcf_path)
    mjcf_root = mjcf_tree.getroot()

    # Check compiler.angle="radian"
    compiler = mjcf_root.find("compiler")
    if compiler is None or compiler.get("angle") != "radian":
        raise ValueError("MJCF compiler angle is not 'radian'")

    # Check timestep match
    option = mjcf_root.find("option")
    if option is None or float(option.get("timestep", 0)) != manifest["scene"]["timestep"]:
        raise ValueError(
            f"MJCF option.timestep {option.get('timestep') if option is not None else None} "
            f"does not match manifest scene.timestep {manifest['scene']['timestep']}"
        )

    # Collect actuator names in MJCF <actuator> section
    mjcf_actuators = {
        p.get("name") for p in mjcf_root.findall("./actuator/position")
    }

    # Collect joint names referenced in MJCF <actuator> (via joint="...")
    mjcf_actuator_joints = {
        p.get("joint") for p in mjcf_root.findall("./actuator/position")
    }

    # Mimic coupling is implemented via <tendon><fixed> + <equality><tendon>
    # (see build_mjcf Phase 2c). Collect tendon-based mimic mappings:
    # mimic_tendons[tendon_name] = {joint_name: coef}
    mimic_tendons: dict[str, dict[str, float]] = {}
    for fixed in mjcf_root.findall("./tendon/fixed"):
        name = fixed.get("name", "")
        if not name.startswith("mimic_"):
            continue
        coefs: dict[str, float] = {}
        for jn in fixed.findall("joint"):
            jname = jn.get("joint", "")
            if not jname:
                continue
            coefs[jname] = float(jn.get("coef", "0"))
        mimic_tendons[name] = coefs

    # Equality tendons referenced by <equality><tendon tendon1="..."/>
    mjcf_equality_tendons = {
        eq.get("tendon1") for eq in mjcf_root.findall("./equality/tendon")
        if eq.get("tendon1")
    }

    # Walk manifest and verify everything referenced exists in MJCF
    for robot in manifest["robots"]:
        for act in robot["actuators"]:
            # Every mjcf_actuator name referenced must exist
            if act["mjcf_actuator"] not in mjcf_actuators:
                raise ValueError(
                    f"manifest actuator '{act['actuator_id']}' references "
                    f"mjcf_actuator='{act['mjcf_actuator']}' which does not exist "
                    f"in MJCF. Known names: {sorted(mjcf_actuators)}"
                )
            # Every urdf_joint must be reachable via MJCF's actuator.joint
            if act["urdf_joint"] not in mjcf_actuator_joints:
                raise ValueError(
                    f"manifest actuator '{act['actuator_id']}' has urdf_joint="
                    f"'{act['urdf_joint']}' but no MJCF <position joint='...'> "
                    f"references it"
                )

            # For gripper, every mimic joint must be coupled via a mimic
            # tendon that is referenced by an <equality><tendon>.
            cap = act["capability"]
            if cap["kind"] == "GRIPPER_PARALLEL":
                primary = act["urdf_joint"]
                for mimic in act["gripper"]["mimic_joints"]:
                    mimic_joint = mimic["joint"]
                    tendon_name = f"mimic_{mimic_joint}"
                    if tendon_name not in mimic_tendons:
                        raise ValueError(
                            f"gripper mimic joint '{mimic_joint}' missing "
                            f"<tendon><fixed name='{tendon_name}'> in MJCF"
                        )
                    if tendon_name not in mjcf_equality_tendons:
                        raise ValueError(
                            f"mimic tendon '{tendon_name}' not referenced by "
                            f"<equality><tendon tendon1=...> in MJCF"
                        )
                    coefs = mimic_tendons[tendon_name]
                    # The fixed tendon encodes: length = 1*mimic + (-k)*primary
                    # so constraining length=0 gives mimic = k*primary.
                    expected_primary_coef = -float(mimic["multiplier"])
                    if mimic_joint not in coefs or abs(coefs[mimic_joint] - 1.0) > 1e-9:
                        raise ValueError(
                            f"mimic tendon '{tendon_name}' must have joint "
                            f"'{mimic_joint}' with coef=1, got {coefs.get(mimic_joint)}"
                        )
                    if primary not in coefs or abs(coefs[primary] - expected_primary_coef) > 1e-9:
                        raise ValueError(
                            f"mimic tendon '{tendon_name}' primary coef mismatch: "
                            f"expected {expected_primary_coef} "
                            f"(=-multiplier {mimic['multiplier']}), "
                            f"got {coefs.get(primary)}"
                        )


if __name__ == "__main__":
    sys.exit(main())
