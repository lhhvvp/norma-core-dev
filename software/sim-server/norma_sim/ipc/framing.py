"""Length-delimited framing for the asyncio UDS transport.

Every frame is `u32_be length | payload`. Max frame length is 16 MiB,
matching the Rust sim-runtime's tokio_util LengthDelimitedCodec in
`software/sim-runtime/src/ipc/framing.rs`. A malformed peer cannot
exhaust memory.
"""
from __future__ import annotations

import asyncio
import struct

MAX_FRAME_LEN = 16 * 1024 * 1024  # 16 MiB


async def read_frame(reader: asyncio.StreamReader) -> bytes:
    header = await reader.readexactly(4)
    (length,) = struct.unpack(">I", header)
    if length > MAX_FRAME_LEN:
        raise ValueError(f"frame too large: {length} bytes")
    if length == 0:
        return b""
    return await reader.readexactly(length)


async def write_frame(writer: asyncio.StreamWriter, payload: bytes) -> None:
    if len(payload) > MAX_FRAME_LEN:
        raise ValueError(f"frame too large: {len(payload)} bytes")
    header = struct.pack(">I", len(payload))
    writer.write(header)
    if payload:
        writer.write(payload)
    await writer.drain()
