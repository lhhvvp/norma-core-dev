"""World manifest loader + source_hash verification.

Parses `hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml`
(produced and edited by humans) into an immutable dataclass tree and
verifies that the generated MJCF's embedded `source_hash=sha256:...`
comment matches `sha256(urdf_bytes + manifest_bytes)`. A mismatch
means the manifest/URDF were edited but `make regen-mjcf` was not
re-run; the caller should fail fast with an actionable error.
"""
from __future__ import annotations

import hashlib
import re
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
    urdf_joint: str
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
        urdf_joint=raw["urdf_joint"],
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


# --------------------------------------------------------------------------
# source_hash verification
# --------------------------------------------------------------------------


def verify_source_hash(manifest_path: Path, mjcf_path: Path) -> None:
    """Raise ValueError if the MJCF's embedded source_hash doesn't
    match sha256(urdf_bytes + manifest_bytes). Matches the hash
    written by `hardware/elrobot/simulation/worlds/gen.py`.
    """
    with manifest_path.open() as f:
        raw = yaml.safe_load(f)
    manifest_dir = manifest_path.parent
    urdf_path = (manifest_dir / raw["urdf_source"]).resolve()

    urdf_bytes = urdf_path.read_bytes()
    manifest_bytes = manifest_path.read_bytes()
    expected = hashlib.sha256(urdf_bytes + manifest_bytes).hexdigest()

    mjcf_text = mjcf_path.read_text()
    m = re.search(r"source_hash=sha256:([0-9a-f]{64})", mjcf_text)
    if m is None:
        raise ValueError(
            f"MJCF at {mjcf_path} has no source_hash comment. "
            f"Run 'make regen-mjcf'."
        )
    found = m.group(1)
    if found != expected:
        raise ValueError(
            f"MJCF source_hash mismatch. Run 'make regen-mjcf'.\n"
            f"  expected: sha256:{expected[:16]}...\n"
            f"  found:    sha256:{found[:16]}..."
        )
