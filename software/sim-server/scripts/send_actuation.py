#!/usr/bin/env python3
"""Connect to a running norma_sim, complete the handshake, send one
ActuationBatch constructed from CLI args, then exit.

Usage:
  PYTHONPATH=software/sim-server python3 \\
    software/sim-server/scripts/send_actuation.py \\
    --socket /tmp/norma-sim-dev.sock \\
    --robot elrobot_follower --actuator rev_motor_01 --value 0.5
"""
from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

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


_LANE_MAP = {
    "lossy": QosLane.QOS_LOSSY_SETPOINT,
    "reliable": QosLane.QOS_RELIABLE_CONTROL,
}


async def _run(
    socket_path: Path,
    robot_id: str,
    actuator_id: str,
    value: float,
    lane: str,
) -> int:
    try:
        reader, writer = await asyncio.open_unix_connection(str(socket_path))
    except (FileNotFoundError, ConnectionRefusedError) as e:
        print(f"ERROR: cannot connect to {socket_path}: {e}", file=sys.stderr)
        return 1
    try:
        # Handshake
        await write_frame(
            writer,
            encode_envelope(
                Envelope(
                    hello=Hello(
                        protocol_version=PROTOCOL_VERSION,
                        client_role="send_actuation",
                        client_id="send-cli",
                    )
                )
            ),
        )
        welcome = decode_envelope(await read_frame(reader))
        if welcome.welcome is None:
            if welcome.error is not None:
                print(
                    f"ERROR: {welcome.error.code}: {welcome.error.message}",
                    file=sys.stderr,
                )
            return 2

        batch = ActuationBatch(
            as_of=None,
            commands=[
                ActuationCommand(
                    ref=ActuatorRef(robot_id=robot_id, actuator_id=actuator_id),
                    set_position=SetPosition(value=value, max_velocity=0.0),
                )
            ],
            lane=_LANE_MAP[lane],
        )
        await write_frame(writer, encode_envelope(Envelope(actuation=batch)))
        print(f"sent set_position {actuator_id}={value} ({lane})")
        return 0
    finally:
        writer.close()
        try:
            await writer.wait_closed()
        except Exception:
            pass


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Send one ActuationBatch to norma_sim")
    ap.add_argument("--socket", type=Path, required=True)
    ap.add_argument("--robot", required=True, help="robot_id (e.g. elrobot_follower)")
    ap.add_argument("--actuator", required=True, help="actuator_id (e.g. rev_motor_01)")
    ap.add_argument("--value", type=float, required=True, help="set_position value")
    ap.add_argument(
        "--lane",
        choices=sorted(_LANE_MAP),
        default="lossy",
        help="QoS lane: 'lossy' (drop-oldest setpoint) or 'reliable' (discrete action)",
    )
    args = ap.parse_args(argv)
    try:
        return asyncio.run(
            _run(args.socket, args.robot, args.actuator, args.value, args.lane)
        )
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
