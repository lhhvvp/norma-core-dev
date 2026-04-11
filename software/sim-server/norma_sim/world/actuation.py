"""`ActuationApplier` — translate `ActuationBatch` proto messages into
`MjData.ctrl` writes on a `MuJoCoWorld`.

For MVP-1 we only honour `set_position` intents (the one-field oneof
variant every scenario needs). The other variants land as log warnings
so upstream tooling can debug without a silent drop, but we do not
implement their semantics yet.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING

from . import capabilities
from ._proto import world_pb

if TYPE_CHECKING:
    from .model import MuJoCoWorld

_log = logging.getLogger("norma_sim.actuation")


@dataclass
class ApplyStats:
    """Per-batch counters surfaced to logs / health."""

    applied: int = 0
    unknown_actuator: int = 0
    unsupported_intent: int = 0
    disabled: int = 0


class ActuationApplier:
    """Apply capability-keyed actuation batches to a MuJoCoWorld."""

    def __init__(self, world: "MuJoCoWorld") -> None:
        self.world = world
        # MJCF actuator name → MjData.ctrl index, cached via MuJoCoWorld.
        self._resolve_by_mjcf_name = world.actuator_by_mjcf_name

        # Build a lookup: manifest actuator_id → mjcf_actuator name.
        # Bridges only know actuator_id (the capability-keyed ID), so
        # drain_and_apply needs the reverse direction at hot-path time.
        self._id_to_mjcf: dict[str, str] = {}
        for robot in world.manifest.robots:
            for act in robot.actuators:
                self._id_to_mjcf[act.actuator_id] = act.mjcf_actuator

    def drain_and_apply(self, batch: "world_pb.ActuationBatch") -> ApplyStats:
        stats = ApplyStats()
        commands = batch.commands or []
        for cmd in commands:
            if cmd is None:
                continue
            if cmd.ref is None:
                stats.unsupported_intent += 1
                continue

            mjcf_name = self._id_to_mjcf.get(cmd.ref.actuator_id)
            if mjcf_name is None:
                stats.unknown_actuator += 1
                _log.warning(
                    "unknown actuator",
                    extra={"extra_fields": {"actuator_id": cmd.ref.actuator_id}},
                )
                continue

            actuator = self._resolve_by_mjcf_name(mjcf_name)
            ctrl_idx = self.world.actuator_id_for(mjcf_name)
            if actuator is None or ctrl_idx is None:
                stats.unknown_actuator += 1
                continue

            # Which oneof variant is set?
            if cmd.set_position is not None:
                value = cmd.set_position.value
                ctrl_value = capabilities.command_value_to_ctrl(value, actuator)
                self.world.data.ctrl[ctrl_idx] = ctrl_value
                stats.applied += 1
            elif cmd.disable_torque is not None:
                # MVP-1 equivalent: park ctrl at current qpos so the
                # position controller holds.
                joint_addr = self.world.joint_qposadr_for(actuator.urdf_joint)
                if joint_addr is not None:
                    self.world.data.ctrl[ctrl_idx] = float(
                        self.world.data.qpos[joint_addr]
                    )
                stats.disabled += 1
            else:
                stats.unsupported_intent += 1
                _log.warning(
                    "unsupported actuation intent",
                    extra={"extra_fields": {"actuator_id": cmd.ref.actuator_id}},
                )
        return stats
