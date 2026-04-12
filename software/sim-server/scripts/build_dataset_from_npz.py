#!/usr/bin/env python3
"""Build LeRobot dataset from pre-generated .npz episode files.

Usage:
    PYTHONPATH=. python3 scripts/build_dataset_from_npz.py /tmp/norma_batch_3x4ufgcz
"""
import os
import sys
import time
from pathlib import Path

import numpy as np

_sim_server_dir = str(Path(__file__).resolve().parents[1])
if _sim_server_dir not in sys.path:
    sys.path.insert(0, _sim_server_dir)

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
CAMERAS = ["top", "wrist.top"]
TASK = "pick up the red cube and place it to the side"


def main():
    npz_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/norma_batch_3x4ufgcz"
    dataset_dir = sys.argv[2] if len(sys.argv) > 2 else "datasets/norma_sim_pick_v1"

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import shutil

    if Path(dataset_dir).exists():
        shutil.rmtree(dataset_dir)

    motor_names = [f"{n}.pos" for n in JOINT_NAMES] + ["gripper.pos"]
    features = {
        "observation.state": {"dtype": "float32", "shape": (6,), "names": {"motors": motor_names}},
        "action": {"dtype": "float32", "shape": (6,), "names": {"motors": motor_names}},
    }
    for cam in CAMERAS:
        features[f"observation.images.{cam}"] = {
            "dtype": "image", "shape": (480, 640, 3),
            "names": ["height", "width", "channel"],
        }

    dataset = LeRobotDataset.create(
        repo_id="norma/sim_pick_v1",
        fps=30,
        features=features,
        root=dataset_dir,
        robot_type="norma_sim",
        use_videos=True,
        image_writer_processes=4,
        image_writer_threads=4,
    )

    npz_files = sorted(f for f in os.listdir(npz_dir) if f.endswith(".npz"))
    print(f"Building dataset from {len(npz_files)} episodes → {dataset_dir}")
    print(f"  use_videos=True, image_writer_processes=4")

    t0 = time.monotonic()
    total_frames = 0
    for i, fname in enumerate(npz_files):
        data = np.load(os.path.join(npz_dir, fname))
        n = len(data["states"])
        for j in range(n):
            frame = {
                "observation.state": data["states"][j],
                "action": data["actions"][j],
                "task": TASK,
            }
            if data["images_top"].size > 0:
                frame["observation.images.top"] = data["images_top"][j]
            if data["images_wrist"].size > 0:
                frame["observation.images.wrist.top"] = data["images_wrist"][j]
            dataset.add_frame(frame)
        dataset.save_episode()
        total_frames += n
        data.close()
        if (i + 1) % 5 == 0 or i == len(npz_files) - 1:
            elapsed = time.monotonic() - t0
            print(f"  {i+1}/{len(npz_files)} episodes | {total_frames} frames | {elapsed:.0f}s")

    print(f"\nDone: {dataset.num_episodes} episodes, {dataset.num_frames} frames in {time.monotonic()-t0:.1f}s")


if __name__ == "__main__":
    main()
