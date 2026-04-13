"""Capability-aware unit conversions.

★ This is the ONLY module in norma_sim that encodes capability
semantics. Every other module (actuation.py, snapshot.py, descriptor.py)
delegates here. That containment is important: when we add a new
capability kind (e.g. CAMERA_RGB or GRIPPER_ADAPTIVE) we edit exactly
one file.

The two conversions are inverses of each other. Both accept an
`ActuatorManifest` so they can read the capability kind and the
gripper metadata without the caller having to know the type system.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:  # Avoid a circular import at runtime.
    from .manifest import ActuatorManifest


def command_value_to_ctrl(command_value: float, actuator: "ActuatorManifest") -> float:
    """Translate a capability-keyed command value into the MJCF
    actuator's `data.ctrl[idx]` value.

    Mapping:
      REVOLUTE_POSITION / PRISMATIC_POSITION: identity (the command
        is already in the actuator's native unit)
      GRIPPER_PARALLEL: linearly interpolate
        command ∈ normalized_range  →  ctrl ∈ primary_joint_range_rad
        For the ElRobot gripper this is [0.0, 1.0] → [0.0, 2.2028] rad.
    """
    kind = actuator.capability.kind
    if kind in ("REVOLUTE_POSITION", "PRISMATIC_POSITION"):
        return float(command_value)
    if kind == "GRIPPER_PARALLEL":
        g = actuator.gripper
        if g is None:
            raise ValueError(
                f"GRIPPER_PARALLEL actuator '{actuator.actuator_id}' "
                f"has no gripper metadata"
            )
        norm_lo, norm_hi = g.normalized_range
        joint_lo, joint_hi = g.primary_joint_range_rad
        if norm_hi == norm_lo:
            raise ValueError("normalized_range has zero span")
        t = (command_value - norm_lo) / (norm_hi - norm_lo)
        return joint_lo + t * (joint_hi - joint_lo)
    raise ValueError(f"unsupported capability kind: {kind}")


def gripper_command_to_ctrl(
    normalized: float,
    actuator: "ActuatorManifest",
    ctrlrange: tuple[float, float],
) -> float:
    """Map gripper normalized [0,1] → MuJoCo ctrl value.

    Handles both GRIPPER_PARALLEL (via manifest metadata) and
    REVOLUTE_POSITION grippers (via ctrlrange linear interpolation).
    This is the SINGLE source of truth for gripper control mapping.
    """
    kind = actuator.capability.kind
    if kind == "GRIPPER_PARALLEL" and actuator.gripper is not None:
        return command_value_to_ctrl(normalized, actuator)
    # REVOLUTE_POSITION gripper: linear map [0,1] → ctrlrange
    lo, hi = ctrlrange
    return float(normalized) * (hi - lo) + lo


def gripper_qpos_to_normalized(
    qpos: float,
    actuator: "ActuatorManifest",
    ctrlrange: tuple[float, float],
) -> float:
    """Map gripper MuJoCo qpos → normalized [0,1].

    Inverse of ``gripper_command_to_ctrl``. Handles both
    GRIPPER_PARALLEL and REVOLUTE_POSITION grippers.
    """
    kind = actuator.capability.kind
    if kind == "GRIPPER_PARALLEL" and actuator.gripper is not None:
        return qpos_to_position_value(qpos, actuator)
    # REVOLUTE_POSITION gripper: inverse linear map
    lo, hi = ctrlrange
    rng = hi - lo
    return (float(qpos) - lo) / rng if rng > 0 else 0.0


def qpos_to_position_value(qpos: float, actuator: "ActuatorManifest") -> float:
    """Inverse of ``command_value_to_ctrl``.

    Used by the snapshot builder to report an ActuatorState to the
    sim-runtime in the same units clients use on the actuation side.
    """
    kind = actuator.capability.kind
    if kind in ("REVOLUTE_POSITION", "PRISMATIC_POSITION"):
        return float(qpos)
    if kind == "GRIPPER_PARALLEL":
        g = actuator.gripper
        if g is None:
            raise ValueError(
                f"GRIPPER_PARALLEL actuator '{actuator.actuator_id}' "
                f"has no gripper metadata"
            )
        joint_lo, joint_hi = g.primary_joint_range_rad
        norm_lo, norm_hi = g.normalized_range
        if joint_hi == joint_lo:
            raise ValueError("primary_joint_range_rad has zero span")
        t = (qpos - joint_lo) / (joint_hi - joint_lo)
        return norm_lo + t * (norm_hi - norm_lo)
    raise ValueError(f"unsupported capability kind: {kind}")
