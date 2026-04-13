#!/usr/bin/env python3
"""Validate a LeRobot dataset for quality issues.

Usage:
    cd software/sim-server
    PYTHONPATH=. python3 scripts/validate_dataset.py datasets/norma_sim_pick_v1
    PYTHONPATH=. python3 scripts/validate_dataset.py datasets/norma_sim_pick_v1 --repo-id norma/sim_pick_v1
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_sim_server_dir = str(Path(__file__).resolve().parents[1])
if _sim_server_dir not in sys.path:
    sys.path.insert(0, _sim_server_dir)


def main():
    parser = argparse.ArgumentParser(description="Validate LeRobot dataset quality")
    parser.add_argument("dataset_path", help="Path to dataset directory")
    parser.add_argument("--repo-id", default="norma/sim_pick_v1", help="Dataset repo ID")
    parser.add_argument("--max-episodes", type=int, default=None, help="Limit episodes to check")
    args = parser.parse_args()

    from norma_sim.data_quality import validate_dataset

    report = validate_dataset(
        dataset_path=args.dataset_path,
        repo_id=args.repo_id,
        max_episodes=args.max_episodes,
    )
    print(report)

    if report.critical:
        sys.exit(1)


if __name__ == "__main__":
    main()
