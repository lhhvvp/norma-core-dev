#!/usr/bin/env python3
"""Connect to a running norma_sim, complete the handshake, and print
each WorldSnapshot as it arrives. Ctrl+C to exit.

Usage:
  PYTHONPATH=software/sim-server python3 \\
    software/sim-server/scripts/inspect.py --socket /tmp/norma-sim-dev.sock
"""
from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

from norma_sim.ipc.codec import (
    Envelope,
    Hello,
    decode_envelope,
    encode_envelope,
)
from norma_sim.ipc.framing import read_frame, write_frame
from norma_sim.ipc.session import PROTOCOL_VERSION


def _format_actuator(state) -> str:
    return (
        f"{state.ref.actuator_id if state.ref else '?':<20} "
        f"pos={state.position_value:+.4f} "
        f"goal={state.goal_position_value:+.4f} "
        f"tq_on={'1' if state.torque_enabled else '0'} "
        f"moving={'1' if state.moving else '0'}"
    )


async def _run(socket_path: Path, limit: int | None, client_id: str) -> int:
    try:
        reader, writer = await asyncio.open_unix_connection(str(socket_path))
    except (FileNotFoundError, ConnectionRefusedError) as e:
        print(f"ERROR: cannot connect to {socket_path}: {e}", file=sys.stderr)
        return 1

    # Handshake
    await write_frame(
        writer,
        encode_envelope(
            Envelope(
                hello=Hello(
                    protocol_version=PROTOCOL_VERSION,
                    client_role="inspect",
                    client_id=client_id,
                )
            )
        ),
    )
    welcome = decode_envelope(await read_frame(reader))
    if welcome.welcome is None:
        if welcome.error is not None:
            print(
                f"ERROR: server replied with {welcome.error.code.name if hasattr(welcome.error.code, 'name') else welcome.error.code}: "
                f"{welcome.error.message}",
                file=sys.stderr,
            )
        else:
            print("ERROR: expected Welcome, got something else", file=sys.stderr)
        return 2
    world = welcome.welcome.world
    if world:
        print(f"# connected to '{world.world_name}' (publish_hz={world.publish_hz} physics_hz={world.physics_hz})")

    count = 0
    try:
        while True:
            env = decode_envelope(await read_frame(reader))
            if env.snapshot is None:
                continue
            count += 1
            snap = env.snapshot
            tick = snap.clock.world_tick if snap.clock else "?"
            print(f"--- tick {tick} ({count} snapshot{'s' if count != 1 else ''}) ---")
            for a in snap.actuators or []:
                print(f"  {_format_actuator(a)}")
            if limit is not None and count >= limit:
                return 0
    except (asyncio.IncompleteReadError, ConnectionResetError):
        print("# connection closed by server")
        return 0
    finally:
        writer.close()
        try:
            await writer.wait_closed()
        except Exception:
            pass


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Inspect a running norma_sim")
    ap.add_argument("--socket", type=Path, required=True)
    ap.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Exit after N snapshots (default: run until Ctrl+C)",
    )
    ap.add_argument("--client-id", default="inspect-cli")
    args = ap.parse_args(argv)

    try:
        return asyncio.run(_run(args.socket, args.limit, args.client_id))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
