#!/usr/bin/env python3
"""Probe a norma_sim world manifest.

Loads a world manifest (MVP-2 .scene.yaml schema), builds a
WorldDescriptor, and prints a readable summary.

Usage:
  PYTHONPATH=software/sim-server python3 \\
    software/sim-server/scripts/probe_manifest.py \\
    --manifest hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from norma_sim.world.descriptor import build_world_descriptor
from norma_sim.world.manifest import load_manifest


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Probe a norma_sim world manifest")
    ap.add_argument("--manifest", type=Path, required=True)
    args = ap.parse_args(argv)

    if not args.manifest.exists():
        print(f"ERROR: manifest not found: {args.manifest}", file=sys.stderr)
        return 1

    try:
        manifest = load_manifest(args.manifest)
    except Exception as e:
        print(f"ERROR: manifest load failed: {e}", file=sys.stderr)
        return 1

    desc = build_world_descriptor(manifest)

    print(f"world_name:      {manifest.world_name}")
    print(f"urdf_path:       {manifest.urdf_path if manifest.urdf_path else '(not used in MVP-2)'}")
    print(f"mjcf_path:       {manifest.mjcf_path}")
    print(f"scene.timestep:  {manifest.scene.timestep}s")
    print(f"scene.gravity:   {manifest.scene.gravity}")
    print(f"scene.integrator:{manifest.scene.integrator}")
    print(f"scene.solver:    {manifest.scene.solver}")
    print(f"scene.iterations:{manifest.scene.iterations}")
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
