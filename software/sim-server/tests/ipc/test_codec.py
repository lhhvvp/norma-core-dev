"""Roundtrip tests for ipc.codec."""
import pytest

try:
    from norma_sim.ipc.codec import (
        ActuationBatch,
        ActuationCommand,
        ActuatorRef,
        Envelope,
        Error,
        Error_Code,
        Goodbye,
        Hello,
        QosLane,
        SetPosition,
        Welcome,
        WorldDescriptor,
        decode_envelope,
        encode_envelope,
    )
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"proto not importable: {_ERR}")


def test_codec_hello_roundtrip():
    env = Envelope(
        hello=Hello(
            protocol_version=1,
            client_role="test-client",
            client_id="abc-123",
        )
    )
    encoded = encode_envelope(env)
    decoded = decode_envelope(encoded)
    assert decoded.hello is not None
    assert decoded.hello.protocol_version == 1
    assert decoded.hello.client_role == "test-client"
    assert decoded.hello.client_id == "abc-123"
    # Other oneof variants must all be None.
    assert decoded.welcome is None
    assert decoded.actuation is None


def test_codec_goodbye_roundtrip():
    env = Envelope(goodbye=Goodbye(reason="bye"))
    encoded = encode_envelope(env)
    decoded = decode_envelope(encoded)
    assert decoded.goodbye is not None
    assert decoded.goodbye.reason == "bye"


def test_codec_error_roundtrip():
    env = Envelope(
        error=Error(
            code=Error_Code.E_PROTOCOL_VERSION,
            message="ours=1 theirs=99",
        )
    )
    encoded = encode_envelope(env)
    decoded = decode_envelope(encoded)
    assert decoded.error is not None
    assert decoded.error.code == Error_Code.E_PROTOCOL_VERSION
    assert decoded.error.message == "ours=1 theirs=99"


def test_codec_welcome_with_descriptor():
    env = Envelope(
        welcome=Welcome(
            protocol_version=1,
            world=WorldDescriptor(
                world_name="test_world",
                robots=[],
                publish_hz=100,
                physics_hz=500,
            ),
        )
    )
    encoded = encode_envelope(env)
    decoded = decode_envelope(encoded)
    assert decoded.welcome is not None
    assert decoded.welcome.protocol_version == 1
    assert decoded.welcome.world is not None
    assert decoded.welcome.world.world_name == "test_world"
    assert decoded.welcome.world.publish_hz == 100
    assert decoded.welcome.world.physics_hz == 500


def test_codec_actuation_roundtrip():
    env = Envelope(
        actuation=ActuationBatch(
            as_of=None,
            commands=[
                ActuationCommand(
                    ref=ActuatorRef(
                        robot_id="elrobot_follower",
                        actuator_id="rev_motor_01",
                    ),
                    set_position=SetPosition(value=0.5, max_velocity=0.0),
                )
            ],
            lane=QosLane.QOS_LOSSY_SETPOINT,
        )
    )
    encoded = encode_envelope(env)
    decoded = decode_envelope(encoded)
    assert decoded.actuation is not None
    assert decoded.actuation.lane == QosLane.QOS_LOSSY_SETPOINT
    assert decoded.actuation.commands is not None
    assert len(decoded.actuation.commands) == 1
    cmd = decoded.actuation.commands[0]
    assert cmd.ref is not None
    assert cmd.ref.actuator_id == "rev_motor_01"
    assert cmd.set_position is not None
    assert cmd.set_position.value == pytest.approx(0.5)


def test_codec_empty_envelope_roundtrip():
    env = Envelope()
    encoded = encode_envelope(env)
    # Empty envelope encodes to empty bytes with gremlin_py.
    assert isinstance(encoded, bytes)
    decoded = decode_envelope(encoded)
    assert decoded.hello is None
    assert decoded.welcome is None
