"""Tests for asyncio length-delimited framing.

Uses plain `asyncio.run` + a real UDS socketpair under a temp path.
anyio/pytest-asyncio are not available in this dev environment, so
each test is a sync pytest function that runs an inner async helper.
"""
import asyncio
import os
import struct
import tempfile

import pytest

from norma_sim.ipc.framing import MAX_FRAME_LEN, read_frame, write_frame


async def _connected_uds():
    """Start a short-lived asyncio UDS server and connect to it,
    returning the (client_reader, client_writer, server_reader,
    server_writer) quad plus a `cleanup` coroutine."""
    tmp = tempfile.mkdtemp()
    sock_path = os.path.join(tmp, "f.sock")

    conn_future: asyncio.Future = asyncio.get_running_loop().create_future()

    async def _on_conn(reader, writer):
        if not conn_future.done():
            conn_future.set_result((reader, writer))

    server = await asyncio.start_unix_server(_on_conn, path=sock_path)

    client_reader, client_writer = await asyncio.open_unix_connection(sock_path)
    server_reader, server_writer = await conn_future

    async def _cleanup():
        client_writer.close()
        try:
            await client_writer.wait_closed()
        except Exception:
            pass
        server_writer.close()
        try:
            await server_writer.wait_closed()
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


def _run(coro):
    return asyncio.run(coro)


def test_framing_roundtrip_small():
    async def _inner():
        cr, cw, sr, sw, cleanup = await _connected_uds()
        try:
            payload = b"hello"
            await write_frame(cw, payload)
            got = await read_frame(sr)
            assert got == payload
        finally:
            await cleanup()

    _run(_inner())


def test_framing_roundtrip_1mb_boundary():
    """1 MiB exceeds the default UDS socket buffer, so write_frame +
    drain must race against read_frame rather than run sequentially."""

    async def _inner():
        cr, cw, sr, sw, cleanup = await _connected_uds()
        try:
            payload = b"A" * (1024 * 1024)
            got, _ = await asyncio.gather(read_frame(sr), write_frame(cw, payload))
            assert got == payload
            assert len(got) == 1024 * 1024
        finally:
            await cleanup()

    _run(_inner())


def test_framing_empty_payload():
    """Empty payload is a valid zero-length frame — must round-trip."""

    async def _inner():
        cr, cw, sr, sw, cleanup = await _connected_uds()
        try:
            await write_frame(cw, b"")
            got = await read_frame(sr)
            assert got == b""
        finally:
            await cleanup()

    _run(_inner())


def test_framing_rejects_oversized_header():
    """A peer advertising a frame > MAX_FRAME_LEN must be refused
    rather than risk OOM."""

    async def _inner():
        cr, cw, sr, sw, cleanup = await _connected_uds()
        try:
            bad_header = struct.pack(">I", MAX_FRAME_LEN + 1)
            cw.write(bad_header)
            await cw.drain()
            with pytest.raises(ValueError, match="frame too large"):
                await read_frame(sr)
        finally:
            await cleanup()

    _run(_inner())
