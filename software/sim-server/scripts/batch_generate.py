#!/usr/bin/env python3
"""Fast single-process batch data generator for ACT training.

Uses FastSim (in-process MuJoCo, no subprocess/IPC) with configurable
camera resolution. Writes directly to LeRobot dataset — no intermediate
.npz files, no multiprocessing.

Usage:
    cd software/sim-server
    PYTHONPATH=. MUJOCO_GL=egl python3 scripts/batch_generate.py
    PYTHONPATH=. MUJOCO_GL=egl python3 scripts/batch_generate.py --episodes 50 --camera-size 224

Output: datasets/norma_sim_pick_v1/ (LeRobot v3 format)
"""
from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

import numpy as np

_sim_server_dir = str(Path(__file__).resolve().parents[1])
if _sim_server_dir not in sys.path:
    sys.path.insert(0, _sim_server_dir)

REPO_ROOT = Path(__file__).resolve().parents[3]
MANIFEST = str(REPO_ROOT / "hardware/elrobot/simulation/manifests/norma/therobotstudio_so101_tabletop.scene.yaml")

from norma_sim.lerobot_helpers import (
    JOINT_NAMES, GRIPPER_NAME, ALL_MOTOR_NAMES,
    build_state_vector, build_action_vector,
)

TASK = "pick up the red cube and place it to the side"


def interpolate(start: list[float], end: list[float], t: float) -> list[float]:
    return [s + (e - s) * t for s, e in zip(start, end)]


def generate_waypoints(rng: np.random.Generator) -> list[tuple]:
    """Generate randomized pick-and-place waypoints."""
    pan = rng.uniform(-0.8, 0.8)
    approach_flex = rng.uniform(1.2, 1.6)
    lift_flex = rng.uniform(0.8, 1.2)
    speed = rng.uniform(0.7, 1.3)

    def s(base_steps: int) -> int:
        return max(10, int(base_steps * speed))

    home = [float(n) for n in rng.normal(0, 0.05, size=5)]

    return [
        ("home",      home,                                      0.0, s(30)),
        ("above",     [0.0, -0.6,  1.3, -0.1,  0.0],           0.0, s(40)),
        ("approach",  [0.0, -0.6,  approach_flex,  0.0,  0.0],  0.0, s(30)),
        ("grasp",     [0.0, -0.6,  approach_flex,  0.0,  0.0],  1.0, s(20)),
        ("lift",      [0.0, -0.6,  lift_flex, -0.3,  0.0],      1.0, s(40)),
        ("carry",     [pan, -0.4,  0.8, -0.2,  0.0],            1.0, s(40)),
        ("release",   [pan, -0.4,  0.8, -0.2,  0.0],            0.0, s(20)),
        ("home",      home,                                      0.0, s(40)),
    ]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Fast batch data generator")
    p.add_argument("--episodes", type=int, default=200)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--camera-size", type=int, default=224, help="Camera height=width (square)")
    p.add_argument("--cameras", nargs="+", default=["top"], help="Camera names")
    p.add_argument("--dataset-dir", default="datasets/norma_sim_pick_v1")
    p.add_argument("--repo-id", default="norma/sim_pick_v1")
    p.add_argument("--action-noise", type=float, default=0.02)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--no-videos", action="store_true")
    return p.parse_args()


def main():
    args = parse_args()
    cam_h = cam_w = args.camera_size

    print(f"=== Fast Batch Generator (single process, in-process MuJoCo) ===")
    print(f"  Episodes:     {args.episodes}")
    print(f"  Cameras:      {args.cameras} @ {cam_h}×{cam_w}")
    print(f"  Action noise: {args.action_noise} rad")
    print(f"  Dataset:      {args.dataset_dir}")
    print()

    # ── 1. Create FastSim ──
    from norma_sim.fast_sim import FastSim

    cameras = {name: (cam_h, cam_w) for name in args.cameras}
    sim = FastSim(
        manifest_path=MANIFEST,
        cameras=cameras,
        physics_hz=500,
        action_hz=args.fps,
    )
    print(f"FastSim ready: {len(sim._joint_indices)} joints, {len(sim._gripper_indices)} grippers")

    # ── 2. Create LeRobot dataset ──
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(ALL_MOTOR_NAMES),),
            "names": {"motors": ALL_MOTOR_NAMES},
        },
        "action": {
            "dtype": "float32",
            "shape": (len(ALL_MOTOR_NAMES),),
            "names": {"motors": ALL_MOTOR_NAMES},
        },
    }
    for cam_name in args.cameras:
        features[f"observation.images.{cam_name}"] = {
            "dtype": "image",
            "shape": (cam_h, cam_w, 3),
            "names": ["height", "width", "channel"],
        }

    dataset_dir = Path(args.dataset_dir)
    if dataset_dir.exists():
        shutil.rmtree(dataset_dir)
    dataset_dir.parent.mkdir(parents=True, exist_ok=True)

    use_videos = not args.no_videos
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features=features,
        root=str(dataset_dir),
        robot_type="norma_sim",
        use_videos=use_videos,
        image_writer_processes=4 if use_videos else 0,
        image_writer_threads=4,
    )

    # ── 3. Generate episodes — direct write, no intermediate files ──
    print(f"Generating {args.episodes} episodes...\n")
    t0 = time.monotonic()

    for ep in range(args.episodes):
        rng = np.random.default_rng(seed=args.seed + ep)
        waypoints = generate_waypoints(rng)
        sim.reset()

        current_joints = list(waypoints[0][1])
        current_gripper = waypoints[0][2]
        frame_count = 0

        for wp_name, target_joints, target_gripper, n_steps in waypoints:
            start_joints = list(current_joints)
            start_gripper = current_gripper

            for step in range(n_steps):
                t = (step + 1) / n_steps
                joints = interpolate(start_joints, target_joints, t)
                gripper = start_gripper + (target_gripper - start_gripper) * t
                noisy_joints = [j + rng.normal(0, args.action_noise) for j in joints]

                obs = sim.step(
                    joint_positions=np.array(noisy_joints),
                    gripper_normalized=gripper,
                )

                frame = {
                    "observation.state": build_state_vector(obs),
                    "action": build_action_vector(noisy_joints, gripper),
                    "task": TASK,
                }
                for cam_name in args.cameras:
                    cam_key = f"camera.{cam_name}"
                    if cam_key in obs:
                        frame[f"observation.images.{cam_name}"] = obs[cam_key]

                dataset.add_frame(frame)
                frame_count += 1

            current_joints = target_joints
            current_gripper = target_gripper

        dataset.save_episode()

        elapsed = time.monotonic() - t0
        rate = (ep + 1) / elapsed
        eta = (args.episodes - ep - 1) / rate if rate > 0 else 0
        if (ep + 1) % 10 == 0 or ep == 0 or ep == args.episodes - 1:
            print(
                f"  {ep+1:4d}/{args.episodes} | "
                f"{frame_count:3d} frames | "
                f"{rate:.1f} ep/s | "
                f"{elapsed:.0f}s elapsed | "
                f"ETA {eta:.0f}s"
            )

    elapsed_total = time.monotonic() - t0
    sim.close()

    print(f"\n=== Done ===")
    print(f"  Directory:  {dataset_dir}")
    print(f"  Episodes:   {dataset.num_episodes}")
    print(f"  Frames:     {dataset.num_frames}")
    print(f"  Features:   {list(dataset.features.keys())}")
    print(f"  Time:       {elapsed_total:.1f}s ({elapsed_total/60:.1f} min)")


if __name__ == "__main__":
    main()
