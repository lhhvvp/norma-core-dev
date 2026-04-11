"""Per-client async session: handshake + reader/writer loops.

A `ClientSession` is constructed by `IpcServer._handle_client` for
each accepted UDS connection. It owns the handshake, the inbound
decoder loop (routing actuation to an injected callback), and the
outbound encoder loop (draining a per-session snapshot queue).

The session has no shared state with siblings — all fan-out happens
at the IpcServer level by pushing each snapshot into every session's
queue.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Callable, Optional

from ..world.manifest import WorldManifest
from .codec import (
    ActuationBatch,
    Envelope,
    Error,
    Error_Code,
    Welcome,
    WorldDescriptor,
    WorldSnapshot,
    decode_envelope,
    encode_envelope,
)
from .framing import read_frame, write_frame

PROTOCOL_VERSION = 1

_log = logging.getLogger("norma_sim.ipc.session")


class ClientSession:
    def __init__(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
        manifest: WorldManifest,
        descriptor: "WorldDescriptor",
        on_actuation: Callable[["ActuationBatch"], None],
        snapshot_queue: "asyncio.Queue[Optional[WorldSnapshot]]",
        session_id: str,
    ) -> None:
        self.reader = reader
        self.writer = writer
        self.manifest = manifest
        self.descriptor = descriptor
        self.on_actuation = on_actuation
        self.snapshot_queue = snapshot_queue
        self.session_id = session_id
        self._closed = False

    async def run(self) -> None:
        """Main entry: handshake → reader + writer loops in parallel."""
        try:
            ok = await self._handshake()
            if not ok:
                return
            await asyncio.gather(
                self._reader_loop(),
                self._writer_loop(),
                return_exceptions=True,
            )
        finally:
            await self._close()

    async def _handshake(self) -> bool:
        """Read the client's Hello, reply with Welcome or Error.

        Returns True on success, False on any failure (caller should
        then close the session without entering the loops).
        """
        try:
            frame = await read_frame(self.reader)
        except (asyncio.IncompleteReadError, ConnectionResetError):
            _log.warning("session %s closed before Hello", self.session_id)
            return False

        env = decode_envelope(frame)
        if env.hello is None:
            _log.warning("session %s first frame is not Hello", self.session_id)
            await self._send(
                Envelope(
                    error=Error(
                        code=Error_Code.E_PROTOCOL_VERSION,
                        message="expected Hello",
                    )
                )
            )
            return False

        hello = env.hello
        if hello.protocol_version != PROTOCOL_VERSION:
            await self._send(
                Envelope(
                    error=Error(
                        code=Error_Code.E_PROTOCOL_VERSION,
                        message=(
                            f"server version {PROTOCOL_VERSION}, "
                            f"client {hello.protocol_version}"
                        ),
                    )
                )
            )
            _log.warning(
                "session %s protocol mismatch: client=%d server=%d",
                self.session_id,
                hello.protocol_version,
                PROTOCOL_VERSION,
            )
            return False

        await self._send(
            Envelope(
                welcome=Welcome(
                    protocol_version=PROTOCOL_VERSION,
                    world=self.descriptor,
                )
            )
        )
        _log.info(
            "session %s handshake ok role=%s id=%s",
            self.session_id,
            hello.client_role,
            hello.client_id,
        )
        return True

    async def _reader_loop(self) -> None:
        while not self._closed:
            try:
                frame = await read_frame(self.reader)
            except (asyncio.IncompleteReadError, ConnectionResetError):
                return
            env = decode_envelope(frame)
            if env.actuation is not None:
                try:
                    self.on_actuation(env.actuation)
                except Exception:
                    _log.exception(
                        "on_actuation callback raised in session %s",
                        self.session_id,
                    )
            elif env.goodbye is not None:
                _log.info(
                    "session %s received Goodbye: %s",
                    self.session_id,
                    env.goodbye.reason,
                )
                return
            # Silently ignore any other payload.

    async def _writer_loop(self) -> None:
        while not self._closed:
            snap = await self.snapshot_queue.get()
            if snap is None:
                return
            env = Envelope(snapshot=snap)
            try:
                await write_frame(self.writer, encode_envelope(env))
            except (ConnectionResetError, BrokenPipeError):
                return

    async def _send(self, env: "Envelope") -> None:
        try:
            await write_frame(self.writer, encode_envelope(env))
        except (ConnectionResetError, BrokenPipeError):
            pass

    async def _close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.writer.close()
            await self.writer.wait_closed()
        except Exception:
            pass
