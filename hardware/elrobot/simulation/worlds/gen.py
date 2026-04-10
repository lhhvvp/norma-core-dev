#!/usr/bin/env python3
"""
gen.py — derive MJCF from world.yaml manifest + URDF.

Pipeline (see docs/superpowers/specs/2026-04-10-simulation-integration-design.md §9.3):
  1. Load manifest yaml
  2. Load URDF via mujoco.MjModel.from_xml_path (MuJoCo auto-detects <robot> root)
  3. Dump canonical MJCF via mujoco.mj_saveLastXML to a temp baseline file
  4. Parse the baseline MJCF, modify/add option+compiler+equality+actuator+default+worldbody
  5. Embed sha256(urdf + manifest) as an XML comment in the output MJCF
  6. Self-check consistency

Usage:
  python3 hardware/elrobot/simulation/worlds/gen.py
  python3 hardware/elrobot/simulation/worlds/gen.py --manifest other.yaml

Exit codes:
  0 = success
  1 = manifest / URDF load error (bad input)
  2 = self-check failure (internal inconsistency)
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

# Enforce Python version explicitly — the plan assumes 3.11+ for PEP 585 generics and dataclass features
if sys.version_info < (3, 11):
    sys.exit(f"gen.py requires Python 3.11+; got {sys.version_info}")

import yaml


def main() -> int:
    ap = argparse.ArgumentParser(description="Derive MJCF from world.yaml manifest + URDF")
    ap.add_argument(
        "--manifest",
        type=Path,
        default=Path(__file__).parent / "elrobot_follower.world.yaml",
        help="Path to world.yaml manifest",
    )
    args = ap.parse_args()

    try:
        manifest = load_manifest(args.manifest)
    except Exception as e:
        print(f"ERROR: failed to load manifest {args.manifest}: {e}", file=sys.stderr)
        return 1

    manifest_dir = args.manifest.parent.resolve()
    urdf_path = (manifest_dir / manifest["urdf_source"]).resolve()
    mjcf_path = (manifest_dir / manifest["mjcf_output"]).resolve()

    if not urdf_path.is_file():
        print(f"ERROR: URDF not found at {urdf_path}", file=sys.stderr)
        return 1

    # Compute source hash (urdf bytes + manifest yaml bytes) BEFORE we touch
    # anything on disk. This is what the runtime (Chunk 5) verifies.
    urdf_bytes = urdf_path.read_bytes()
    manifest_bytes = args.manifest.read_bytes()
    source_hash = hashlib.sha256(urdf_bytes + manifest_bytes).hexdigest()

    # Build MJCF via MuJoCo-powered pipeline (Task 1.4 fills this in)
    try:
        mjcf_root = build_mjcf(manifest, urdf_path, manifest_dir)
    except NotImplementedError:
        # Skeleton stage — re-raise so the test in Task 1.3 Step 2 catches it
        raise
    except Exception as e:
        print(f"ERROR: build_mjcf failed: {e}", file=sys.stderr)
        return 1

    # Write MJCF with source_hash comment (Task 1.4 fills this in)
    write_mjcf(mjcf_path, mjcf_root, source_hash)

    # Self-check (Task 1.5 fills this in; Task 1.3 skeleton is a no-op)
    try:
        run_self_check(manifest, mjcf_path)
    except Exception as e:
        print(f"ERROR: self-check failed: {e}", file=sys.stderr)
        return 2

    print(f"gen.py OK: {mjcf_path}")
    print(f"  urdf: {urdf_path} ({len(urdf_bytes)} bytes)")
    print(f"  source_hash=sha256:{source_hash[:16]}...")
    return 0


def load_manifest(path: Path) -> dict[str, Any]:
    with path.open() as f:
        return yaml.safe_load(f)


def build_mjcf(manifest: dict[str, Any], urdf_path: Path, manifest_dir: Path) -> ET.Element:
    """Placeholder; Task 1.4 fleshes this out via mujoco.MjModel + mj_saveLastXML + XML edits."""
    raise NotImplementedError("build_mjcf is implemented in Task 1.4")


def write_mjcf(path: Path, mjcf_elem: ET.Element, source_hash: str) -> None:
    """Placeholder; Task 1.4 fleshes this out."""
    raise NotImplementedError("write_mjcf is implemented in Task 1.4")


def run_self_check(manifest: dict[str, Any], mjcf_path: Path) -> None:
    """Placeholder; Task 1.5 fleshes this out. Task 1.3 skeleton is a no-op."""
    pass


if __name__ == "__main__":
    sys.exit(main())
