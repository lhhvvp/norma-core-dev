"""World manifest loader.

Parses `hardware/elrobot/simulation/elrobot_follower.scene.yaml`
(hand-written, edited by humans) into an immutable dataclass tree.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml


# --------------------------------------------------------------------------
# Dataclass hierarchy (all frozen so passing them around is safe)
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class ActuatorCapability:
    kind: str  # "REVOLUTE_POSITION" | "PRISMATIC_POSITION" | "GRIPPER_PARALLEL"
    limit_min: Optional[float] = None
    limit_max: Optional[float] = None
    effort_limit: Optional[float] = None
    velocity_limit: Optional[float] = None


@dataclass(frozen=True)
class GripperMimic:
    joint: str
    multiplier: float


@dataclass(frozen=True)
class GripperMeta:
    primary_joint_range_rad: tuple
    normalized_range: tuple
    mimic_joints: tuple  # tuple[GripperMimic, ...]


@dataclass(frozen=True)
class ActuatorManifest:
    actuator_id: str
    display_name: str
    mjcf_joint: str
    mjcf_actuator: str
    capability: ActuatorCapability
    actuator_gains: dict
    gripper: Optional[GripperMeta] = None


@dataclass(frozen=True)
class SensorManifest:
    sensor_id: str
    display_name: str
    capability_kind: str
    source: Optional[str] = None


@dataclass(frozen=True)
class RobotManifest:
    robot_id: str
    actuators: tuple  # tuple[ActuatorManifest, ...]
    sensors: tuple  # tuple[SensorManifest, ...]


@dataclass(frozen=True)
class SceneConfig:
    timestep: float
    gravity: tuple
    integrator: str
    solver: str
    iterations: int


@dataclass(frozen=True)
class WorldManifest:
    world_name: str
    scene: SceneConfig
    robots: tuple  # tuple[RobotManifest, ...]
    mjcf_path: Path
    urdf_path: Optional[Path] = None  # MVP-2: sim no longer consumes URDF


# --------------------------------------------------------------------------
# Loader
# --------------------------------------------------------------------------


DEFAULT_ROBOT_ID = "default_robot"


def load_manifest(manifest_path: Path) -> WorldManifest:
    """Load and validate an MVP-2 scene.yaml.

    Schema (see spec §8.1):

        world_name: str               # required
        mjcf_path: str                # required, relative to the yaml file
        robot_id: str                 # optional, default='default_robot'
        scene_overrides:              # optional, overrides MJCF <option>
          timestep: float
          gravity: [x, y, z]
          integrator: str
          solver: str
          iterations: int
        scene_extras:                 # optional, runtime-added worldbody items
          lights: [...]
          floor: {...}
        actuator_annotations:         # optional; only for non-default capabilities
          - mjcf_actuator: str        # must exist in MJCF
            actuator_id: str          # id used by bridge + descriptor
            display_name: str
            capability:
              kind: REVOLUTE_POSITION | PRISMATIC_POSITION | GRIPPER_PARALLEL
              limit_min: float        # optional
              limit_max: float
              effort_limit: float
              velocity_limit: float
              normalized_range: [lo, hi]   # required when kind=GRIPPER_PARALLEL
            gripper:                  # required when kind=GRIPPER_PARALLEL
              primary_joint_range_rad: [lo, hi]
              mimic_joints:
                - {joint: str, multiplier: float}

    Actuators in the MJCF that are NOT listed in `actuator_annotations`
    and are MuJoCo `<position>` type are auto-synthesized as
    REVOLUTE_POSITION ActuatorManifest entries. `<motor>`, `<velocity>`,
    or `<general>` actuators without annotation are silently skipped
    (MVP-2 only ships the REVOLUTE_POSITION default).
    """
    manifest_path = Path(manifest_path)  # accept str too, matches MVP-1 duck-typing
    with manifest_path.open() as f:
        raw = yaml.safe_load(f) or {}

    if "mjcf_path" not in raw:
        raise ValueError(
            f"scene.yaml {manifest_path} missing required 'mjcf_path'"
        )
    if "world_name" not in raw:
        raise ValueError(
            f"scene.yaml {manifest_path} missing required 'world_name'"
        )

    manifest_dir = manifest_path.parent
    mjcf_path = (manifest_dir / raw["mjcf_path"]).resolve()
    if not mjcf_path.exists():
        raise ValueError(
            f"scene.yaml {manifest_path} references non-existent "
            f"mjcf_path: {mjcf_path}"
        )

    # Scene config — overrides MJCF <option>. Defaults match MVP-1 baseline
    # for backward compatibility when a yaml omits scene_overrides entirely.
    scene_overrides = raw.get("scene_overrides") or {}
    scene = SceneConfig(
        timestep=float(scene_overrides.get("timestep", 0.002)),
        gravity=tuple(scene_overrides.get("gravity", [0.0, 0.0, -9.81])),
        integrator=scene_overrides.get("integrator", "RK4"),
        solver=scene_overrides.get("solver", "Newton"),
        iterations=int(scene_overrides.get("iterations", 50)),
    )

    # Enumerate MJCF actuators → (name, joint_name, type_tag)
    mjcf_actuators = _enumerate_mjcf_actuators(mjcf_path)
    mjcf_actuator_names = {name for name, _, _ in mjcf_actuators}

    # Build annotation lookup (keyed by mjcf_actuator name)
    annotations = raw.get("actuator_annotations") or []
    annotation_by_name: dict[str, dict] = {}
    for ann in annotations:
        if "mjcf_actuator" not in ann:
            raise ValueError(
                f"actuator_annotation in {manifest_path} missing "
                f"required field 'mjcf_actuator'"
            )
        mjcf_name = ann["mjcf_actuator"]
        if mjcf_name not in mjcf_actuator_names:
            raise ValueError(
                f"actuator_annotation references mjcf_actuator "
                f"'{mjcf_name}' but no such actuator exists in "
                f"{mjcf_path}. Available: {sorted(mjcf_actuator_names)}"
            )
        annotation_by_name[mjcf_name] = ann

    # Synthesize ActuatorManifest list. Annotation takes precedence;
    # otherwise default to REVOLUTE_POSITION for <position> actuators.
    actuators: list[ActuatorManifest] = []
    for mjcf_name, joint_name, type_tag in mjcf_actuators:
        if mjcf_name in annotation_by_name:
            actuators.append(
                _parse_annotated_actuator(
                    annotation_by_name[mjcf_name], joint_name
                )
            )
        elif type_tag == "position":
            actuators.append(
                _synthesize_revolute_actuator(mjcf_name, joint_name)
            )
        else:
            # <motor> / <general> / <velocity> without annotation → skip
            continue

    robots = (
        RobotManifest(
            robot_id=raw.get("robot_id", DEFAULT_ROBOT_ID),
            actuators=tuple(actuators),
            sensors=(),  # MVP-2 does not consume sensors; deferred per spec §2.3
        ),
    )

    return WorldManifest(
        world_name=raw["world_name"],
        scene=scene,
        robots=robots,
        mjcf_path=mjcf_path,
        urdf_path=None,
    )


def _synthesize_revolute_actuator(
    mjcf_name: str, mjcf_joint: str
) -> ActuatorManifest:
    """Default ActuatorManifest for a <position> actuator with no
    scene.yaml annotation. actuator_id = mjcf_name, display_name humanized.
    All capability limits left as None (MJCF's ctrlrange / forcerange is
    the source of truth — downstream code reads them from the MjModel,
    not from the manifest)."""
    return ActuatorManifest(
        actuator_id=mjcf_name,
        display_name=mjcf_name.replace("_", " ").title(),
        mjcf_joint=mjcf_joint,
        mjcf_actuator=mjcf_name,
        capability=ActuatorCapability(kind="REVOLUTE_POSITION"),
        actuator_gains={},
        gripper=None,
    )


def _parse_annotated_actuator(
    ann: dict, mjcf_joint: str
) -> ActuatorManifest:
    """Parse an actuator_annotations entry into ActuatorManifest.
    `mjcf_joint` is resolved by the caller from MJCF (not from yaml).

    Spec alignment: `normalized_range` lives under `capability:` (not
    under `gripper:`), matching spec §8.1 yaml example. `primary_joint_range_rad`
    and `mimic_joints` live under `gripper:`.
    """
    cap_raw = ann["capability"]
    cap = ActuatorCapability(
        kind=cap_raw["kind"],
        limit_min=cap_raw.get("limit_min"),
        limit_max=cap_raw.get("limit_max"),
        effort_limit=cap_raw.get("effort_limit"),
        velocity_limit=cap_raw.get("velocity_limit"),
    )
    gripper: Optional[GripperMeta] = None
    if cap.kind == "GRIPPER_PARALLEL":
        if "normalized_range" not in cap_raw:
            raise ValueError(
                f"GRIPPER_PARALLEL capability on '{ann['mjcf_actuator']}' "
                f"missing 'normalized_range' (should live under capability:)"
            )
        normalized_range = tuple(cap_raw["normalized_range"])
        g_raw = ann.get("gripper")
        if g_raw is None:
            raise ValueError(
                f"actuator_annotation for '{ann['mjcf_actuator']}' has "
                f"kind GRIPPER_PARALLEL but no 'gripper:' metadata"
            )
        mimic = tuple(
            GripperMimic(joint=m["joint"], multiplier=float(m["multiplier"]))
            for m in g_raw.get("mimic_joints", [])
        )
        gripper = GripperMeta(
            primary_joint_range_rad=tuple(g_raw["primary_joint_range_rad"]),
            normalized_range=normalized_range,
            mimic_joints=mimic,
        )
    return ActuatorManifest(
        actuator_id=ann["actuator_id"],
        display_name=ann["display_name"],
        mjcf_joint=mjcf_joint,
        mjcf_actuator=ann["mjcf_actuator"],
        capability=cap,
        actuator_gains={},
        gripper=gripper,
    )


def _enumerate_mjcf_actuators(mjcf_path: Path) -> list[tuple[str, str, str]]:
    """Parse an MJCF file via MuJoCo's compiler (which resolves <include>)
    and return the actuator list as `(actuator_name, joint_name, type_tag)`
    tuples.

    `type_tag` values:
      - "position" — `<position>` actuator: synthesized as REVOLUTE_POSITION
        when no annotation is provided
      - "motor"    — `<motor>` actuator: requires explicit annotation
      - "general"  — `<general>` actuator: requires explicit annotation
      - "velocity" — `<velocity>` actuator: requires explicit annotation

    The type distinction is derived from the gain/bias type enum pair:
      position: gain=FIXED, bias=AFFINE
      motor:    gain=FIXED, bias=NONE
      general:  anything else
    """
    import mujoco  # imported lazily so this module stays lightweight

    if not mjcf_path.exists():
        raise FileNotFoundError(f"MJCF not found: {mjcf_path}")

    try:
        model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    except Exception as e:
        raise ValueError(f"failed to compile MJCF {mjcf_path}: {e}") from e

    # Resolve enum values via the typed enums (robust to MuJoCo version bumps)
    gain_fixed = int(mujoco.mjtGain.mjGAIN_FIXED)
    bias_affine = int(mujoco.mjtBias.mjBIAS_AFFINE)
    bias_none = int(mujoco.mjtBias.mjBIAS_NONE)
    joint_trn_type = int(mujoco.mjtTrn.mjTRN_JOINT)

    results: list[tuple[str, str, str]] = []
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if not name:
            continue  # skip unnamed actuators (rare)

        # Classify actuator type from gain/bias pair
        gain_type = int(model.actuator_gaintype[i])
        bias_type = int(model.actuator_biastype[i])
        if gain_type == gain_fixed and bias_type == bias_affine:
            type_tag = "position"
        elif gain_type == gain_fixed and bias_type == bias_none:
            type_tag = "motor"
        else:
            type_tag = "general"

        # Resolve the joint name this actuator controls.
        # actuator_trntype[i] can be JOINT (1) or other (tendon, site).
        # actuator_trnid[i, 0] is the joint id when trntype == JOINT.
        if int(model.actuator_trntype[i]) != joint_trn_type:
            continue  # non-joint actuators (tendons, sites) are not supported in MVP-2
        joint_id = int(model.actuator_trnid[i, 0])
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if not joint_name:
            continue  # actuator controlling unnamed joint — rare edge case, skip

        results.append((name, joint_name, type_tag))
    return results
