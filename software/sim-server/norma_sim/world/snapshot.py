"""`SnapshotBuilder` — read MjData state and assemble a WorldSnapshot proto."""
from __future__ import annotations

from typing import TYPE_CHECKING

from . import capabilities
from ._proto import world_pb

if TYPE_CHECKING:
    from .model import MuJoCoWorld


class SnapshotBuilder:
    """Build WorldSnapshot protos from a MuJoCoWorld.

    The builder caches the (actuator_id, mjcf_actuator, ctrl_idx,
    qpos_addr, ActuatorManifest) tuple list at construction so
    build() stays cheap even at high publish rates.
    """

    def __init__(self, world: "MuJoCoWorld") -> None:
        self.world = world
        self._rows: list[tuple[str, str, int, int, object]] = []
        for robot in world.manifest.robots:
            for act in robot.actuators:
                ctrl_idx = world.actuator_id_for(act.mjcf_actuator)
                qpos_addr = world.joint_qposadr_for(act.urdf_joint)
                if ctrl_idx is None or qpos_addr is None:
                    continue
                self._rows.append(
                    (
                        robot.robot_id,
                        act.actuator_id,
                        int(ctrl_idx),
                        int(qpos_addr),
                        act,
                    )
                )

    def build(self, clock: "world_pb.WorldClock | None") -> "world_pb.WorldSnapshot":
        actuators = []
        data = self.world.data
        for robot_id, act_id, ctrl_idx, qpos_addr, act in self._rows:
            qpos = float(data.qpos[qpos_addr])
            position_value = capabilities.qpos_to_position_value(qpos, act)
            # velocity index: jnt_dofadr for this joint gives qvel index
            goal_value = capabilities.qpos_to_position_value(
                float(data.ctrl[ctrl_idx]), act
            )
            actuators.append(
                world_pb.ActuatorState(
                    ref=world_pb.ActuatorRef(
                        robot_id=robot_id,
                        actuator_id=act_id,
                    ),
                    position_value=position_value,
                    velocity_value=0.0,  # MVP-1: joint-velocity readout deferred
                    effort_value=0.0,    # MVP-1: no sensor
                    torque_enabled=True,
                    moving=False,
                    goal_position_value=goal_value,
                )
            )
        return world_pb.WorldSnapshot(
            clock=clock,
            actuators=actuators,
            sensors=[],
        )
