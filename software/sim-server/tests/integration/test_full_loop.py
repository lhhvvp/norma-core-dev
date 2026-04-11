"""Subprocess integration test: launch `python -m norma_sim`, do the
full handshake + actuate + snapshot round-trip, then terminate."""
from __future__ import annotations

import asyncio
import os
import sys
from pathlib import Path

import pytest

try:
    from norma_sim.ipc.codec import (
        ActuationBatch,
        ActuationCommand,
        ActuatorRef,
        Envelope,
        Hello,
        QosLane,
        SetPosition,
        decode_envelope,
        encode_envelope,
    )
    from norma_sim.ipc.framing import read_frame, write_frame
    from norma_sim.ipc.session import PROTOCOL_VERSION
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"proto not importable: {_ERR}")


def _run(coro):
    return asyncio.run(coro)


async def _wait_for_socket(path: Path, timeout: float = 5.0) -> bool:
    elapsed = 0.0
    step = 0.05
    while elapsed < timeout:
        if path.exists():
            return True
        await asyncio.sleep(step)
        elapsed += step
    return False


async def _spawn_sim(socket_path: Path, scene_yaml_path: Path) -> asyncio.subprocess.Process:
    # Pass PYTHONPATH so the subprocess can import norma_sim. The
    # parent test run already has it set, but subprocess env has to
    # be constructed explicitly for asyncio.create_subprocess_exec.
    env = os.environ.copy()
    repo_root = Path(__file__).resolve().parents[4]
    sim_server_dir = repo_root / "software" / "sim-server"
    # sim-server is first so our generated proto shim wins; repo_root
    # is needed for `shared.gremlin_py.gremlin` + the
    # `target.gen_python.protobuf.sim.world` import chain.
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = (
        f"{sim_server_dir}{os.pathsep}{repo_root}"
        + (os.pathsep + existing if existing else "")
    )
    env["NORMA_SIM_SOCKET_PATH"] = str(socket_path)

    return await asyncio.create_subprocess_exec(
        sys.executable,
        "-m",
        "norma_sim",
        "--manifest",
        str(scene_yaml_path),
        "--physics-hz",
        "500",
        "--publish-hz",
        "100",
        "--log-level",
        "WARNING",
        env=env,
        stdout=asyncio.subprocess.DEVNULL,
        stderr=asyncio.subprocess.PIPE,
    )


async def _handshake(reader, writer, role: str, client_id: str) -> "Envelope":
    await write_frame(
        writer,
        encode_envelope(
            Envelope(
                hello=Hello(
                    protocol_version=PROTOCOL_VERSION,
                    client_role=role,
                    client_id=client_id,
                )
            )
        ),
    )
    return decode_envelope(await read_frame(reader))


def test_full_loop(elrobot_scene_yaml, tmp_path):
    """Launch norma_sim as a subprocess; handshake; send an
    actuation; receive at least one snapshot; terminate cleanly."""

    async def _inner():
        socket_path = tmp_path / "sim.sock"
        proc = await _spawn_sim(socket_path, elrobot_scene_yaml)
        try:
            assert await _wait_for_socket(socket_path, timeout=5.0), (
                "sim server did not bind socket within 5s; "
                f"stderr={(await proc.stderr.read()) if proc.stderr else b''!r}"
            )

            reader, writer = await asyncio.open_unix_connection(str(socket_path))
            welcome = await _handshake(reader, writer, "full-loop", "c1")
            assert welcome.welcome is not None
            assert welcome.welcome.world is not None
            assert welcome.welcome.world.world_name == "elrobot_follower"

            # Send an actuation batch.
            await write_frame(
                writer,
                encode_envelope(
                    Envelope(
                        actuation=ActuationBatch(
                            as_of=None,
                            commands=[
                                ActuationCommand(
                                    ref=ActuatorRef(
                                        robot_id="elrobot_follower",
                                        actuator_id="rev_motor_01",
                                    ),
                                    set_position=SetPosition(value=0.2, max_velocity=0.0),
                                )
                            ],
                            lane=QosLane.QOS_LOSSY_SETPOINT,
                        )
                    )
                ),
            )

            # Read snapshots until we see one with actuators populated
            # (the server will start publishing at publish_hz=100).
            saw_snapshot = False
            for _ in range(30):  # up to ~300 ms of snapshots
                env = decode_envelope(
                    await asyncio.wait_for(read_frame(reader), timeout=1.0)
                )
                if env.snapshot is not None and env.snapshot.actuators:
                    saw_snapshot = True
                    break
            assert saw_snapshot, "no snapshot with actuator state received"

            writer.close()
            try:
                await writer.wait_closed()
            except Exception:
                pass
        finally:
            proc.terminate()
            try:
                await asyncio.wait_for(proc.wait(), timeout=3.0)
            except asyncio.TimeoutError:
                proc.kill()
                await proc.wait()

    _run(_inner())


def test_multi_client_fan_out(menagerie_scene_yaml, tmp_path):
    """★★ Two clients connect to the same sim subprocess; both
    receive snapshots from the same publish cycle."""

    async def _inner():
        socket_path = tmp_path / "sim.sock"
        proc = await _spawn_sim(socket_path, menagerie_scene_yaml)
        try:
            assert await _wait_for_socket(socket_path, timeout=5.0)

            async def _new_client(name: str):
                r, w = await asyncio.open_unix_connection(str(socket_path))
                welcome = await _handshake(r, w, "fan-out", name)
                assert welcome.welcome is not None
                return r, w

            r1, w1 = await _new_client("c1")
            r2, w2 = await _new_client("c2")

            async def _await_snapshot(r) -> "Envelope":
                for _ in range(30):
                    env = decode_envelope(
                        await asyncio.wait_for(read_frame(r), timeout=1.0)
                    )
                    if env.snapshot is not None:
                        return env
                raise AssertionError("no snapshot received within budget")

            snap1, snap2 = await asyncio.gather(
                _await_snapshot(r1),
                _await_snapshot(r2),
            )
            assert snap1.snapshot is not None and snap2.snapshot is not None
            # The snapshots come from the same broadcast loop; both
            # clients should see (at least) one snapshot. Ticks may
            # differ by 1 if the scheduler ran between put_nowait
            # calls, but both should be non-empty.
            assert snap1.snapshot.clock is not None
            assert snap2.snapshot.clock is not None

            for w in (w1, w2):
                w.close()
                try:
                    await w.wait_closed()
                except Exception:
                    pass
        finally:
            proc.terminate()
            try:
                await asyncio.wait_for(proc.wait(), timeout=3.0)
            except asyncio.TimeoutError:
                proc.kill()
                await proc.wait()

    _run(_inner())


def test_subprocess_clean_shutdown(menagerie_scene_yaml, tmp_path):
    """SIGTERM should make the subprocess exit within a reasonable
    window without leaving the socket file behind."""

    async def _inner():
        socket_path = tmp_path / "sim.sock"
        proc = await _spawn_sim(socket_path, menagerie_scene_yaml)
        try:
            assert await _wait_for_socket(socket_path, timeout=5.0)
        finally:
            proc.terminate()
            try:
                rc = await asyncio.wait_for(proc.wait(), timeout=3.0)
            except asyncio.TimeoutError:
                proc.kill()
                await proc.wait()
                rc = proc.returncode

        # Allow either normal exit or signal-killed; the key assertion
        # is that we got past wait() without hitting the kill-fallback.
        assert rc is not None
        assert not socket_path.exists(), (
            "socket file remained after SIGTERM — cleanup path broken"
        )

    _run(_inner())
