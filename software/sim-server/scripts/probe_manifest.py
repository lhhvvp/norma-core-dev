#!/usr/bin/env python3
"""No-network diagnostic: load a world manifest, verify its MJCF's
source_hash against the inputs, and print a readable summary of the
WorldDescriptor that sim-runtime would see at handshake.

Usage:
  PYTHONPATH=software/sim-server python3 \\
    software/sim-server/scripts/probe_manifest.py \\
    --manifest hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from norma_sim.world.descriptor import build_world_descriptor
from norma_sim.world.manifest import load_manifest, verify_source_hash


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Probe a norma_sim world manifest")
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument(
        "--no-verify-hash",
        action="store_true",
        help="Skip the sha256 check (e.g. when testing a gen.py in progress)",
    )
    args = ap.parse_args(argv)

    if not args.manifest.exists():
        print(f"ERROR: manifest not found: {args.manifest}", file=sys.stderr)
        return 1

    try:
        manifest = load_manifest(args.manifest)
    except Exception as e:
        print(f"ERROR: manifest load failed: {e}", file=sys.stderr)
        return 1

    if not args.no_verify_hash:
        try:
            verify_source_hash(args.manifest, manifest.mjcf_path)
            hash_line = f"source_hash OK ({manifest.mjcf_path.name})"
        except Exception as e:
            print(f"ERROR: source_hash: {e}", file=sys.stderr)
            return 2
    else:
        hash_line = "source_hash verification: SKIPPED"

    desc = build_world_descriptor(manifest)

    print(f"world_name:      {manifest.world_name}")
    print(f"urdf_path:       {manifest.urdf_path}")
    print(f"mjcf_path:       {manifest.mjcf_path}")
    print(f"scene.timestep:  {manifest.scene.timestep}s")
    print(f"scene.gravity:   {manifest.scene.gravity}")
    print(f"scene.integrator:{manifest.scene.integrator}")
    print(f"scene.solver:    {manifest.scene.solver}")
    print(f"scene.iterations:{manifest.scene.iterations}")
    print(hash_line)
    print()

    print(f"robots ({len(desc.robots or [])}):")
    for robot in desc.robots or []:
        print(f"  {robot.robot_id}")
        print(f"    actuators ({len(robot.actuators or [])}):")
        for a in robot.actuators or []:
            cap = a.capability
            print(
                f"      {a.actuator_id:<20} "
                f"display={a.display_name!r:<22} "
                f"kind={cap.kind.name if cap and hasattr(cap.kind, 'name') else cap and cap.kind}"
            )
        if robot.sensors:
            print(f"    sensors ({len(robot.sensors)}):")
            for s in robot.sensors:
                print(f"      {s.sensor_id:<20} display={s.display_name!r}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
