"""World manifest loader.

Parses `hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml`
(produced and edited by humans) into an immutable dataclass tree.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

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
    urdf_path: Path
    mjcf_path: Path


# --------------------------------------------------------------------------
# Loader
# --------------------------------------------------------------------------


def load_manifest(manifest_path: Path) -> WorldManifest:
    """Load and validate a world manifest yaml."""
    with manifest_path.open() as f:
        raw = yaml.safe_load(f)

    manifest_dir = manifest_path.parent
    urdf_path = (manifest_dir / raw["urdf_source"]).resolve()
    mjcf_path = (manifest_dir / raw["mjcf_output"]).resolve()

    scene = SceneConfig(
        timestep=float(raw["scene"]["timestep"]),
        gravity=tuple(raw["scene"]["gravity"]),
        integrator=raw["scene"]["integrator"],
        solver=raw["scene"]["solver"],
        iterations=int(raw["scene"]["iterations"]),
    )

    robots: list[RobotManifest] = []
    for r in raw["robots"]:
        actuators = tuple(_parse_actuator(a) for a in r["actuators"])
        sensors = tuple(_parse_sensor(s) for s in r.get("sensors", []))
        robots.append(
            RobotManifest(
                robot_id=r["robot_id"],
                actuators=actuators,
                sensors=sensors,
            )
        )

    return WorldManifest(
        world_name=raw["world_name"],
        scene=scene,
        robots=tuple(robots),
        urdf_path=urdf_path,
        mjcf_path=mjcf_path,
    )


def _parse_actuator(raw: dict[str, Any]) -> ActuatorManifest:
    cap_raw = raw["capability"]
    cap = ActuatorCapability(
        kind=cap_raw["kind"],
        limit_min=cap_raw.get("limit_min"),
        limit_max=cap_raw.get("limit_max"),
        effort_limit=cap_raw.get("effort_limit"),
        velocity_limit=cap_raw.get("velocity_limit"),
    )
    gripper: Optional[GripperMeta] = None
    if cap.kind == "GRIPPER_PARALLEL":
        g_raw = raw.get("gripper")
        if g_raw is None:
            raise ValueError(
                f"actuator '{raw['actuator_id']}' has capability "
                f"GRIPPER_PARALLEL but no 'gripper:' metadata"
            )
        mimic = tuple(
            GripperMimic(joint=m["joint"], multiplier=float(m["multiplier"]))
            for m in g_raw["mimic_joints"]
        )
        gripper = GripperMeta(
            primary_joint_range_rad=tuple(g_raw["primary_joint_range_rad"]),
            normalized_range=tuple(g_raw["normalized_range"]),
            mimic_joints=mimic,
        )
    return ActuatorManifest(
        actuator_id=raw["actuator_id"],
        display_name=raw["display_name"],
        mjcf_joint=raw["urdf_joint"],
        mjcf_actuator=raw["mjcf_actuator"],
        capability=cap,
        actuator_gains=dict(raw["actuator_gains"]),
        gripper=gripper,
    )


def _parse_sensor(raw: dict[str, Any]) -> SensorManifest:
    return SensorManifest(
        sensor_id=raw["sensor_id"],
        display_name=raw["display_name"],
        capability_kind=raw["capability"]["kind"],
        source=raw.get("source"),
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
