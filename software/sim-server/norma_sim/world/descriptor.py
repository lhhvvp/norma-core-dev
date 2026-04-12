"""Build a `WorldDescriptor` protobuf from a `WorldManifest`.

Handshake flow (spec §6.5):
  sim-runtime → Hello
  norma_sim   → Welcome { world: WorldDescriptor }

`build_world_descriptor` takes the already-loaded manifest plus the
already-validated MuJoCo model (so actuator-count checks don't have
to re-parse the MJCF) and assembles the proto payload with the
capability mapping applied.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from ._proto import world_pb
from .manifest import ActuatorManifest, SensorManifest, WorldManifest

if TYPE_CHECKING:
    from .model import MuJoCoWorld


# Maps ActuatorManifest.capability.kind (string from yaml) to the
# generated ActuatorCapability_Kind enum. Kept in this file because
# descriptor.py is the only place we cross the manifest ↔ proto
# boundary; the rest of norma_sim works in manifest units.
_ACT_KIND_MAP = {
    "REVOLUTE_POSITION": world_pb.ActuatorCapability_Kind.CAP_REVOLUTE_POSITION,
    "PRISMATIC_POSITION": world_pb.ActuatorCapability_Kind.CAP_PRISMATIC_POSITION,
    "GRIPPER_PARALLEL": world_pb.ActuatorCapability_Kind.CAP_GRIPPER_PARALLEL,
}


_SENSOR_KIND_MAP = {
    "JOINT_STATE": world_pb.SensorCapability_Kind.SENSE_JOINT_STATE,
    "CAMERA_RGB": world_pb.SensorCapability_Kind.SENSE_CAMERA_RGB,
    "IMU_6DOF": world_pb.SensorCapability_Kind.SENSE_IMU_6_DOF,
}


def _build_actuator_descriptor(
    act: ActuatorManifest,
    ctrl_range: tuple[float, float] | None = None,
) -> "world_pb.ActuatorDescriptor":
    kind = _ACT_KIND_MAP.get(act.capability.kind)
    if kind is None:
        raise ValueError(
            f"unsupported actuator capability kind "
            f"'{act.capability.kind}' on {act.actuator_id}"
        )
    cap = world_pb.ActuatorCapability(
        kind=kind,
        limit_min=float(act.capability.limit_min or 0.0),
        limit_max=float(act.capability.limit_max or 0.0),
        effort_limit=float(act.capability.effort_limit or 0.0),
        velocity_limit=float(act.capability.velocity_limit or 0.0),
    )
    cr_min, cr_max = ctrl_range if ctrl_range else (0.0, 0.0)
    return world_pb.ActuatorDescriptor(
        actuator_id=act.actuator_id,
        display_name=act.display_name,
        capability=cap,
        ctrl_range_min=cr_min,
        ctrl_range_max=cr_max,
    )


def _build_sensor_descriptor(sensor: SensorManifest) -> "world_pb.SensorDescriptor":
    kind = _SENSOR_KIND_MAP.get(
        sensor.capability_kind,
        world_pb.SensorCapability_Kind.SENSE_UNSPECIFIED,
    )
    cap = world_pb.SensorCapability(kind=kind)
    return world_pb.SensorDescriptor(
        sensor_id=sensor.sensor_id,
        display_name=sensor.display_name,
        capability=cap,
    )


def build_world_descriptor(
    manifest: WorldManifest,
    world: "MuJoCoWorld | None" = None,
    publish_hz: int = 100,
    physics_hz: int = 500,
) -> "world_pb.WorldDescriptor":
    """Assemble a `WorldDescriptor` proto from a WorldManifest.

    MVP-2 notes:
    - `manifest.robots[i].sensors` is always empty (sensor enumeration
      from MJCF is deferred; see spec §2.3).
    - `manifest.urdf_path` may be None in MVP-2; this function does not
      read urdf_path, so the None case is transparent.

    `world` is used to extract MJCF ctrlrange per actuator, which the
    bridge uses to compute correct servo step ranges for the web UI.
    """
    # Build a lookup: mjcf_actuator_name → (ctrl_range_min, ctrl_range_max)
    ctrl_ranges: dict[str, tuple[float, float]] = {}
    if world is not None:
        import mujoco
        for i in range(world.model.nu):
            name = mujoco.mj_id2name(
                world.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i
            )
            if name:
                lo = float(world.model.actuator_ctrlrange[i, 0])
                hi = float(world.model.actuator_ctrlrange[i, 1])
                ctrl_ranges[name] = (lo, hi)

    robots = []
    for r in manifest.robots:
        actuators = [
            _build_actuator_descriptor(a, ctrl_ranges.get(a.mjcf_actuator))
            for a in r.actuators
        ]
        sensors = [
            _build_sensor_descriptor(s) for s in r.sensors
        ]
        robots.append(
            world_pb.RobotDescriptor(
                robot_id=r.robot_id,
                actuators=actuators,
                sensors=sensors,
            )
        )

    return world_pb.WorldDescriptor(
        world_name=manifest.world_name,
        robots=robots,
        initial_clock=world_pb.WorldClock(
            world_tick=0,
            sim_time_ns=0,
            wall_time_ns=0,
        ),
        publish_hz=publish_hz,
        physics_hz=physics_hz,
    )
