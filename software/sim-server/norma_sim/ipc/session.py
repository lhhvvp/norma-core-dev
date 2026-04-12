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
    StepResponse,
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
        on_step: Optional[Callable[[int], "WorldSnapshot"]] = None,
        on_reset: Optional[Callable[[], "WorldSnapshot"]] = None,
    ) -> None:
        self.reader = reader
        self.writer = writer
        self.manifest = manifest
        self.descriptor = descriptor
        self.on_actuation = on_actuation
        self.snapshot_queue = snapshot_queue
        self.session_id = session_id
        self.on_step = on_step
        self.on_reset = on_reset
        self._closed = False

    async def run(self) -> None:
        """Main entry: handshake → reader + writer loops in parallel.

        We use ``asyncio.wait`` with ``FIRST_COMPLETED`` instead of
        ``gather`` so that when one loop exits (typically the reader
        on EOF) the other is cancelled promptly. Otherwise the
        writer_loop would block forever on ``snapshot_queue.get()``
        and Server.wait_closed() would never return.
        """
        try:
            ok = await self._handshake()
            if not ok:
                return
            reader_task = asyncio.create_task(self._reader_loop())
            writer_task = asyncio.create_task(self._writer_loop())
            try:
                done, pending = await asyncio.wait(
                    {reader_task, writer_task},
                    return_when=asyncio.FIRST_COMPLETED,
                )
            finally:
                for task in (reader_task, writer_task):
                    if not task.done():
                        task.cancel()
                # Drain cancellations so exceptions propagate to logs.
                for task in (reader_task, writer_task):
                    try:
                        await task
                    except (asyncio.CancelledError, Exception):
                        pass
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
            elif env.step_request is not None:
                await self._handle_step_request(env.step_request)
            elif env.reset_request is not None:
                await self._handle_reset_request(env.reset_request)
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

    async def _handle_step_request(self, req) -> None:
        if self.on_step is None:
            _log.warning(
                "session %s received StepRequest but scheduler is not stepping-mode",
                self.session_id,
            )
            return
        try:
            n = req.n_ticks if req.n_ticks > 0 else 1
            snapshot = self.on_step(n)
            await self._send(Envelope(step_response=StepResponse(snapshot=snapshot)))
        except Exception:
            _log.exception("on_step raised in session %s", self.session_id)

    async def _handle_reset_request(self, req) -> None:
        if self.on_reset is None:
            _log.warning(
                "session %s received ResetRequest but scheduler is not stepping-mode",
                self.session_id,
            )
            return
        try:
            seed = req.seed if req.seed > 0 else None
            snapshot = self.on_reset(seed)
            await self._send(Envelope(step_response=StepResponse(snapshot=snapshot)))
        except Exception:
            _log.exception("on_reset raised in session %s", self.session_id)

    async def _close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.writer.close()
            await self.writer.wait_closed()
        except Exception:
            pass
