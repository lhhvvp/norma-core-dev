"""Asyncio UDS server + snapshot fan-out.

`IpcServer` owns the listening unix socket, creates one
`ClientSession` per connection, and broadcasts `WorldSnapshot`s to
every active session's per-session queue. The scheduler (on a
worker thread) calls `broadcast_snapshot`, which must be safe to
invoke from outside the asyncio loop — see `cli.py` for the
`loop.call_soon_threadsafe` bridge.
"""
from __future__ import annotations

import asyncio
import logging
import os
import uuid
from pathlib import Path
from typing import Callable, List, Optional

from ..world.manifest import WorldManifest
from .codec import ActuationBatch, WorldDescriptor, WorldSnapshot
from .session import ClientSession

_log = logging.getLogger("norma_sim.ipc.server")


class IpcServer:
    def __init__(
        self,
        socket_path: Path,
        manifest: WorldManifest,
        descriptor: "WorldDescriptor",
        on_actuation: Callable[["ActuationBatch"], None],
        on_step: Optional[Callable[[int], "WorldSnapshot"]] = None,
        on_reset: Optional[Callable[[], "WorldSnapshot"]] = None,
    ) -> None:
        self.socket_path = Path(socket_path)
        self.manifest = manifest
        self.descriptor = descriptor
        self.on_actuation = on_actuation
        self.on_step = on_step
        self.on_reset = on_reset
        self._sessions: List[ClientSession] = []
        self._sessions_lock = asyncio.Lock()
        self._server: asyncio.base_events.Server | None = None

    async def start(self) -> None:
        # Clean up stale socket from a previous run. The parent
        # directory is assumed to already exist (created by the
        # Rust sim-runtime's TempRuntimeDir or the caller).
        if self.socket_path.exists():
            try:
                self.socket_path.unlink()
            except OSError:
                pass
        self._server = await asyncio.start_unix_server(
            self._handle_client,
            path=str(self.socket_path),
        )
        try:
            os.chmod(self.socket_path, 0o600)
        except OSError:
            pass
        _log.info("ipc server listening", extra={"extra_fields": {"socket": str(self.socket_path)}})

    async def _handle_client(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        session_id = str(uuid.uuid4())
        snapshot_queue: asyncio.Queue = asyncio.Queue(maxsize=32)
        session = ClientSession(
            reader=reader,
            writer=writer,
            manifest=self.manifest,
            descriptor=self.descriptor,
            on_actuation=self.on_actuation,
            snapshot_queue=snapshot_queue,
            session_id=session_id,
            on_step=self.on_step,
            on_reset=self.on_reset,
        )
        async with self._sessions_lock:
            self._sessions.append(session)
        _log.info(
            "client connected",
            extra={"extra_fields": {"session_id": session_id}},
        )
        try:
            await session.run()
        finally:
            async with self._sessions_lock:
                if session in self._sessions:
                    self._sessions.remove(session)
            _log.info(
                "client disconnected",
                extra={"extra_fields": {"session_id": session_id}},
            )

    def broadcast_snapshot(self, snap: "WorldSnapshot") -> None:
        """Fan out a snapshot to every active session's queue.

        Must be called FROM the asyncio event loop. If the scheduler
        is on a worker thread, wrap the call in
        ``loop.call_soon_threadsafe(server.broadcast_snapshot, snap)``.
        """
        # Iterate snapshot of current sessions to avoid holding the lock
        # during put_nowait (which can block if the queue is full).
        for session in list(self._sessions):
            try:
                session.snapshot_queue.put_nowait(snap)
            except asyncio.QueueFull:
                _log.warning(
                    "session queue full, dropping snapshot",
                    extra={"extra_fields": {"session_id": session.session_id}},
                )

    async def stop(self) -> None:
        if self._server is not None:
            self._server.close()
            try:
                await self._server.wait_closed()
            except Exception:
                pass
            self._server = None
        # Sentinel drain: let writer loops exit by pushing None.
        async with self._sessions_lock:
            sessions = list(self._sessions)
        for s in sessions:
            try:
                s.snapshot_queue.put_nowait(None)  # type: ignore[arg-type]
            except asyncio.QueueFull:
                pass
        if self.socket_path.exists():
            try:
                self.socket_path.unlink()
            except OSError:
                pass

    @property
    def session_count(self) -> int:
        return len(self._sessions)
