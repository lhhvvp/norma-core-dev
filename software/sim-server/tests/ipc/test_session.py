"""Tests for ClientSession handshake + loops."""
import asyncio
import os
import tempfile

import pytest

try:
    from norma_sim.ipc.codec import (
        Envelope,
        Error_Code,
        Hello,
        WorldDescriptor,
        decode_envelope,
        encode_envelope,
    )
    from norma_sim.ipc.framing import read_frame, write_frame
    from norma_sim.ipc.session import PROTOCOL_VERSION, ClientSession
    from norma_sim.world.manifest import load_manifest
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"proto not importable: {_ERR}")


def _run(coro):
    return asyncio.run(coro)


async def _pair():
    """Create a connected UDS pair + tempdir cleanup."""
    tmp = tempfile.mkdtemp()
    sock_path = os.path.join(tmp, "s.sock")
    conn_future = asyncio.get_running_loop().create_future()

    async def _on_conn(reader, writer):
        if not conn_future.done():
            conn_future.set_result((reader, writer))

    server = await asyncio.start_unix_server(_on_conn, path=sock_path)
    client_reader, client_writer = await asyncio.open_unix_connection(sock_path)
    server_reader, server_writer = await conn_future

    async def _cleanup():
        for w in (client_writer, server_writer):
            try:
                w.close()
                await w.wait_closed()
            except Exception:
                pass
        server.close()
        await server.wait_closed()
        try:
            os.unlink(sock_path)
        except FileNotFoundError:
            pass
        os.rmdir(tmp)

    return client_reader, client_writer, server_reader, server_writer, _cleanup


def _fake_descriptor(name: str) -> "WorldDescriptor":
    return WorldDescriptor(
        world_name=name,
        robots=[],
        publish_hz=100,
        physics_hz=500,
    )


def test_session_handshake_happy_path(menagerie_scene_yaml):
    async def _inner():
        cr, cw, sr, sw, cleanup = await _pair()
        try:
            manifest = load_manifest(menagerie_scene_yaml)
            actuations = []
            session = ClientSession(
                reader=sr,
                writer=sw,
                manifest=manifest,
                descriptor=_fake_descriptor("t1"),
                on_actuation=actuations.append,
                snapshot_queue=asyncio.Queue(),
                session_id="sess-1",
            )

            async def _client():
                await write_frame(
                    cw,
                    encode_envelope(
                        Envelope(
                            hello=Hello(
                                protocol_version=PROTOCOL_VERSION,
                                client_role="test",
                                client_id="c1",
                            )
                        )
                    ),
                )
                welcome_bytes = await read_frame(cr)
                return decode_envelope(welcome_bytes)

            async def _drive_session():
                await session._handshake()

            decoded, _ = await asyncio.gather(_client(), _drive_session())
            assert decoded.welcome is not None
            assert decoded.welcome.protocol_version == 1
            assert decoded.welcome.world is not None
            assert decoded.welcome.world.world_name == "t1"
        finally:
            await cleanup()

    _run(_inner())


def test_session_handshake_wrong_version(menagerie_scene_yaml):
    async def _inner():
        cr, cw, sr, sw, cleanup = await _pair()
        try:
            manifest = load_manifest(menagerie_scene_yaml)
            session = ClientSession(
                reader=sr,
                writer=sw,
                manifest=manifest,
                descriptor=_fake_descriptor("t2"),
                on_actuation=lambda _b: None,
                snapshot_queue=asyncio.Queue(),
                session_id="sess-2",
            )

            async def _client():
                await write_frame(
                    cw,
                    encode_envelope(
                        Envelope(
                            hello=Hello(
                                protocol_version=99,
                                client_role="test",
                                client_id="bad-version",
                            )
                        )
                    ),
                )
                return decode_envelope(await read_frame(cr))

            async def _drive():
                ok = await session._handshake()
                return ok

            reply, ok = await asyncio.gather(_client(), _drive())
            assert ok is False
            assert reply.error is not None
            assert reply.error.code == Error_Code.E_PROTOCOL_VERSION
        finally:
            await cleanup()

    _run(_inner())


def test_session_handshake_missing_hello(menagerie_scene_yaml):
    """A client that sends something other than Hello as first
    frame must receive an Error and be rejected."""

    async def _inner():
        cr, cw, sr, sw, cleanup = await _pair()
        try:
            manifest = load_manifest(menagerie_scene_yaml)
            session = ClientSession(
                reader=sr,
                writer=sw,
                manifest=manifest,
                descriptor=_fake_descriptor("t3"),
                on_actuation=lambda _b: None,
                snapshot_queue=asyncio.Queue(),
                session_id="sess-3",
            )

            async def _client():
                # Send a Goodbye first instead of Hello.
                from norma_sim.ipc.codec import Goodbye
                await write_frame(
                    cw, encode_envelope(Envelope(goodbye=Goodbye(reason="nope")))
                )
                return decode_envelope(await read_frame(cr))

            async def _drive():
                return await session._handshake()

            reply, ok = await asyncio.gather(_client(), _drive())
            assert ok is False
            assert reply.error is not None

        finally:
            await cleanup()

    _run(_inner())
