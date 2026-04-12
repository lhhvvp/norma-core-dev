"""Envelope encode/decode helpers wrapping the gremlin_py generated proto.

The generated module is imported via the ``norma_sim.world._proto``
shim, which handles the repo-root sys.path insertion. All other ipc
modules import from here so they don't need to know about the
generated proto's actual filesystem path. If the build pipeline
later switches to google.protobuf bindings, only this file (and
``_proto.py``) change.

Gremlin_py design notes that matter for this codec:
  - Encoding is via plain dataclass `.encode()` → bytes.
  - Decoding is via a `<Msg>Reader(buf)` that lazily parses fields
    on demand via `get_<field>()` methods.
  - Oneof presence: the Reader stores a private `_<variant>_buf`
    attribute that is None when the variant wasn't on the wire.
    Gremlin_py does NOT expose a public "has field" predicate, so
    we reach into the private buffer for oneof detection. This
    keeps the public ipc API in terms of plain Envelope dataclasses
    with Optional oneof fields that are materialised fully.

If gremlin_py's internals change (rename `_hello_buf` etc.), this
file is the single point to update.
"""
from __future__ import annotations

from ..world._proto import world_pb

# Re-exports for session.py / server.py / diagnostic scripts.
Envelope = world_pb.Envelope
Hello = world_pb.Hello
Welcome = world_pb.Welcome
Goodbye = world_pb.Goodbye
Error = world_pb.Error
Error_Code = world_pb.Error_Code
ActuationBatch = world_pb.ActuationBatch
ActuationCommand = world_pb.ActuationCommand
WorldSnapshot = world_pb.WorldSnapshot
WorldDescriptor = world_pb.WorldDescriptor
WorldClock = world_pb.WorldClock
ActuatorState = world_pb.ActuatorState
ActuatorRef = world_pb.ActuatorRef
SetPosition = world_pb.SetPosition
QosLane = world_pb.QosLane
StepRequest = world_pb.StepRequest
StepResponse = world_pb.StepResponse
ResetRequest = world_pb.ResetRequest


def encode_envelope(env: "Envelope") -> bytes:
    return bytes(env.encode())


def decode_envelope(data: bytes) -> "Envelope":
    if not data:
        return Envelope()
    reader = world_pb.EnvelopeReader(memoryview(data))
    return Envelope(
        hello=_hello_from(reader) if reader._hello_buf is not None else None,
        welcome=_welcome_from(reader) if reader._welcome_buf is not None else None,
        actuation=_actuation_from(reader)
        if reader._actuation_buf is not None
        else None,
        snapshot=_snapshot_from(reader)
        if reader._snapshot_buf is not None
        else None,
        goodbye=_goodbye_from(reader) if reader._goodbye_buf is not None else None,
        error=_error_from(reader) if reader._error_buf is not None else None,
        step_request=_step_request_from(reader)
        if reader._step_request_buf is not None
        else None,
        step_response=_step_response_from(reader)
        if reader._step_response_buf is not None
        else None,
        reset_request=_reset_request_from(reader)
        if reader._reset_request_buf is not None
        else None,
    )


def _hello_from(env_reader) -> "Hello":
    r = env_reader.get_hello()
    return Hello(
        protocol_version=r.get_protocol_version(),
        client_role=r.get_client_role(),
        client_id=r.get_client_id(),
    )


def _welcome_from(env_reader) -> "Welcome":
    r = env_reader.get_welcome()
    return Welcome(
        protocol_version=r.get_protocol_version(),
        world=_world_descriptor_from(r) if r._world_buf is not None else None,
    )


def _world_descriptor_from(welcome_reader) -> "WorldDescriptor":
    wr = welcome_reader.get_world()
    robots = []
    for rd_reader in wr.get_robots():
        robots.append(_robot_descriptor_from_reader(rd_reader))
    clock = None
    if wr._initial_clock_buf is not None:
        clock = _world_clock_from(wr.get_initial_clock())
    return WorldDescriptor(
        world_name=wr.get_world_name(),
        robots=robots,
        initial_clock=clock,
        publish_hz=wr.get_publish_hz(),
        physics_hz=wr.get_physics_hz(),
    )


def _robot_descriptor_from_reader(rd_reader) -> "world_pb.RobotDescriptor":
    actuators = []
    for a in rd_reader.get_actuators():
        cap = None
        if a._capability_buf is not None:
            cap_r = a.get_capability()
            cap = world_pb.ActuatorCapability(
                kind=cap_r.get_kind(),
                limit_min=cap_r.get_limit_min(),
                limit_max=cap_r.get_limit_max(),
                effort_limit=cap_r.get_effort_limit(),
                velocity_limit=cap_r.get_velocity_limit(),
            )
        actuators.append(
            world_pb.ActuatorDescriptor(
                actuator_id=a.get_actuator_id(),
                display_name=a.get_display_name(),
                capability=cap,
                ctrl_range_min=a.get_ctrl_range_min(),
                ctrl_range_max=a.get_ctrl_range_max(),
            )
        )
    sensors = []
    for s in rd_reader.get_sensors():
        cap = None
        if s._capability_buf is not None:
            cap_r = s.get_capability()
            cap = world_pb.SensorCapability(kind=cap_r.get_kind())
        sensors.append(
            world_pb.SensorDescriptor(
                sensor_id=s.get_sensor_id(),
                display_name=s.get_display_name(),
                capability=cap,
            )
        )
    return world_pb.RobotDescriptor(
        robot_id=rd_reader.get_robot_id(),
        actuators=actuators,
        sensors=sensors,
    )


def _world_clock_from(clock_reader) -> "WorldClock":
    return WorldClock(
        world_tick=clock_reader.get_world_tick(),
        sim_time_ns=clock_reader.get_sim_time_ns(),
        wall_time_ns=clock_reader.get_wall_time_ns(),
    )


def _actuation_from(env_reader) -> "ActuationBatch":
    r = env_reader.get_actuation()
    as_of = None
    if r._as_of_buf is not None:
        as_of = _world_clock_from(r.get_as_of())
    commands = []
    for c in r.get_commands():
        ref = None
        if c._ref_buf is not None:
            ref_r = c.get_ref()
            ref = ActuatorRef(
                robot_id=ref_r.get_robot_id(),
                actuator_id=ref_r.get_actuator_id(),
            )
        set_pos = None
        if c._set_position_buf is not None:
            sp_r = c.get_set_position()
            set_pos = SetPosition(
                value=sp_r.get_value(),
                max_velocity=sp_r.get_max_velocity(),
            )
        commands.append(
            ActuationCommand(ref=ref, set_position=set_pos)
        )
    return ActuationBatch(
        as_of=as_of,
        commands=commands,
        lane=r.get_lane(),
    )


def _snapshot_from(env_reader) -> "WorldSnapshot":
    return _snapshot_from_reader(env_reader.get_snapshot())


def _goodbye_from(env_reader) -> "Goodbye":
    r = env_reader.get_goodbye()
    return Goodbye(reason=r.get_reason())


def _error_from(env_reader) -> "Error":
    r = env_reader.get_error()
    return Error(code=r.get_code(), message=r.get_message())


def _step_request_from(env_reader) -> "StepRequest":
    r = env_reader.get_step_request()
    return StepRequest(n_ticks=r.get_n_ticks())


def _step_response_from(env_reader) -> "StepResponse":
    r = env_reader.get_step_response()
    snap = None
    if r._snapshot_buf is not None:
        snap = _snapshot_from_reader(r.get_snapshot())
    return StepResponse(snapshot=snap)


def _reset_request_from(env_reader) -> "ResetRequest":
    r = env_reader.get_reset_request()
    return ResetRequest(seed=r.get_seed())


def _snapshot_from_reader(r) -> "WorldSnapshot":
    """Decode a WorldSnapshot from a reader (shared by Envelope.snapshot and StepResponse)."""
    clock = None
    if r._clock_buf is not None:
        clock = _world_clock_from(r.get_clock())
    actuators = []
    for a in r.get_actuators():
        ref = None
        if a._ref_buf is not None:
            rr = a.get_ref()
            ref = ActuatorRef(
                robot_id=rr.get_robot_id(),
                actuator_id=rr.get_actuator_id(),
            )
        actuators.append(
            ActuatorState(
                ref=ref,
                position_value=a.get_position_value(),
                velocity_value=a.get_velocity_value(),
                effort_value=a.get_effort_value(),
                torque_enabled=a.get_torque_enabled(),
                moving=a.get_moving(),
                goal_position_value=a.get_goal_position_value(),
            )
        )
    sensors = []
    for s in r.get_sensors():
        s_ref = None
        if s._ref_buf is not None:
            sr = s.get_ref()
            s_ref = world_pb.SensorRef(
                robot_id=sr.get_robot_id(),
                sensor_id=sr.get_sensor_id(),
            )
        cam_frame = None
        if s._camera_frame_buf is not None:
            cf = s.get_camera_frame()
            cam_frame = world_pb.CameraFrame(
                width=cf.get_width(),
                height=cf.get_height(),
                encoding=cf.get_encoding(),
                data=bytes(cf.get_data()),
                capture_tick=cf.get_capture_tick(),
            )
        sensors.append(
            world_pb.SensorSample(ref=s_ref, camera_frame=cam_frame)
        )
    return WorldSnapshot(clock=clock, actuators=actuators, sensors=sensors)
