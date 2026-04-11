"""Tests for IpcServer: start/stop, handshake via real UDS."""
import asyncio
import os
import tempfile
from pathlib import Path

import pytest

try:
    from norma_sim.ipc.codec import (
        Envelope,
        Hello,
        WorldDescriptor,
        WorldSnapshot,
        decode_envelope,
        encode_envelope,
    )
    from norma_sim.ipc.framing import read_frame, write_frame
    from norma_sim.ipc.server import IpcServer
    from norma_sim.ipc.session import PROTOCOL_VERSION
    from norma_sim.world.manifest import load_manifest
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"proto not importable: {_ERR}")


def _run(coro):
    return asyncio.run(coro)


def _descriptor(name: str) -> "WorldDescriptor":
    return WorldDescriptor(
        world_name=name,
        robots=[],
        publish_hz=100,
        physics_hz=500,
    )


async def _make_server(menagerie_scene_yaml, tmp_dir: Path) -> IpcServer:
    manifest = load_manifest(menagerie_scene_yaml)
    actuations: list = []
    server = IpcServer(
        socket_path=tmp_dir / "sim.sock",
        manifest=manifest,
        descriptor=_descriptor("srv-test"),
        on_actuation=actuations.append,
    )
    await server.start()
    return server


def test_server_start_creates_socket(menagerie_scene_yaml):
    async def _inner():
        tmp = Path(tempfile.mkdtemp())
        try:
            server = await _make_server(menagerie_scene_yaml, tmp)
            assert (tmp / "sim.sock").exists()
            mode = os.stat(tmp / "sim.sock").st_mode & 0o777
            assert mode == 0o600
            await server.stop()
            assert not (tmp / "sim.sock").exists()
        finally:
            if tmp.exists():
                os.rmdir(tmp)

    _run(_inner())


def test_server_accepts_handshake(menagerie_scene_yaml):
    async def _inner():
        tmp = Path(tempfile.mkdtemp())
        try:
            server = await _make_server(menagerie_scene_yaml, tmp)
            sock_path = tmp / "sim.sock"
            reader, writer = await asyncio.open_unix_connection(str(sock_path))
            # Client sends Hello
            await write_frame(
                writer,
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
            welcome = decode_envelope(await read_frame(reader))
            assert welcome.welcome is not None
            assert welcome.welcome.world is not None
            assert welcome.welcome.world.world_name == "srv-test"

            writer.close()
            await writer.wait_closed()
            await server.stop()
        finally:
            if (tmp / "sim.sock").exists():
                (tmp / "sim.sock").unlink()
            if tmp.exists():
                os.rmdir(tmp)

    _run(_inner())


def test_server_broadcast_fan_out(menagerie_scene_yaml):
    """Two clients connect, both go through handshake, then server
    broadcasts one snapshot — both clients must receive it."""

    async def _inner():
        tmp = Path(tempfile.mkdtemp())
        try:
            server = await _make_server(menagerie_scene_yaml, tmp)
            sock_path = tmp / "sim.sock"

            async def _client(name: str):
                r, w = await asyncio.open_unix_connection(str(sock_path))
                await write_frame(
                    w,
                    encode_envelope(
                        Envelope(
                            hello=Hello(
                                protocol_version=PROTOCOL_VERSION,
                                client_role="test",
                                client_id=name,
                            )
                        )
                    ),
                )
                welcome = decode_envelope(await read_frame(r))
                assert welcome.welcome is not None
                return r, w

            c1_r, c1_w = await _client("c1")
            c2_r, c2_w = await _client("c2")

            # Wait a tick for both sessions to register in IpcServer.
            for _ in range(20):
                if server.session_count >= 2:
                    break
                await asyncio.sleep(0.02)
            assert server.session_count == 2

            # NOTE: gremlin_py's encoder skips any field whose encoded
            # size is 0 (see world.py Envelope.encode_to), so the
            # snapshot must carry at least one non-default field for
            # the Envelope wire tag to survive. A populated clock is
            # the cheapest way to guarantee presence.
            from norma_sim.ipc.codec import WorldClock

            snap = WorldSnapshot(
                clock=WorldClock(world_tick=7, sim_time_ns=14_000_000, wall_time_ns=0),
                actuators=[],
                sensors=[],
            )
            server.broadcast_snapshot(snap)

            env1 = decode_envelope(await read_frame(c1_r))
            env2 = decode_envelope(await read_frame(c2_r))
            assert env1.snapshot is not None
            assert env2.snapshot is not None

            for w in (c1_w, c2_w):
                w.close()
                try:
                    await w.wait_closed()
                except Exception:
                    pass

            await server.stop()
        finally:
            if (tmp / "sim.sock").exists():
                (tmp / "sim.sock").unlink()
            if tmp.exists():
                os.rmdir(tmp)

    _run(_inner())
