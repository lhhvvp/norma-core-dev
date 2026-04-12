#!/usr/bin/env python3
"""Batch data generator for ACT training with domain randomization.

Generates a LeRobot-compatible dataset of scripted pick-and-place
trajectories with randomized target positions, speeds, and noise.

Each episode follows: home → above → approach → grasp → lift → carry → release → home
with per-episode randomization of joint targets, interpolation speed,
action noise, and starting posture.

Usage:
    cd software/sim-server
    PYTHONPATH=. python3 scripts/batch_generate.py
    PYTHONPATH=. python3 scripts/batch_generate.py --episodes 50 --no-videos  # quick test

Output: datasets/norma_sim_pick_v1/ (LeRobot v3 format)
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

# Ensure norma_sim is importable
_sim_server_dir = str(Path(__file__).resolve().parents[1])
if _sim_server_dir not in sys.path:
    sys.path.insert(0, _sim_server_dir)

REPO_ROOT = Path(__file__).resolve().parents[3]
MANIFEST = REPO_ROOT / "hardware/elrobot/simulation/manifests/norma/therobotstudio_so101_tabletop.scene.yaml"


def interpolate(start: list[float], end: list[float], t: float) -> list[float]:
    """Linear interpolation between two joint vectors."""
    return [s + (e - s) * t for s, e in zip(start, end)]


def generate_waypoints(rng: np.random.Generator) -> list[tuple]:
    """Generate one set of randomized pick-and-place waypoints.

    Randomizes:
    - shoulder_pan target: where to place (-0.8 to 0.8 rad)
    - elbow_flex approach depth: how deep to reach (1.2 to 1.6 rad)
    - lift height: how high to lift (0.8 to 1.2 rad)
    - speed factor: overall motion speed (0.7x to 1.3x)
    - home position: slight jitter on starting posture
    """
    pan = rng.uniform(-0.8, 0.8)
    approach_flex = rng.uniform(1.2, 1.6)
    lift_flex = rng.uniform(0.8, 1.2)
    speed = rng.uniform(0.7, 1.3)

    def s(base_steps: int) -> int:
        return max(10, int(base_steps * speed))

    home_noise = rng.normal(0, 0.05, size=5)
    home = [float(n) for n in home_noise]  # small offset from zeros

    return [
        ("home",      home,                                        0, s(30)),
        ("above",     [0.0, -0.6,  1.3, -0.1,  0.0],             0, s(40)),
        ("approach",  [0.0, -0.6,  approach_flex,  0.0,  0.0],    0, s(30)),
        ("grasp",     [0.0, -0.6,  approach_flex,  0.0,  0.0],  100, s(20)),
        ("lift",      [0.0, -0.6,  lift_flex, -0.3,  0.0],      100, s(40)),
        ("carry",     [pan, -0.4,  0.8, -0.2,  0.0],            100, s(40)),
        ("release",   [pan, -0.4,  0.8, -0.2,  0.0],              0, s(20)),
        ("home",      home,                                        0, s(40)),
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Batch data generator for ACT training")
    parser.add_argument("--episodes", type=int, default=200, help="Number of episodes to generate")
    parser.add_argument("--fps", type=int, default=30, help="Dataset FPS")
    parser.add_argument("--dataset-dir", type=str, default="datasets/norma_sim_pick_v1")
    parser.add_argument("--repo-id", type=str, default="norma/sim_pick_v1")
    parser.add_argument("--action-noise", type=float, default=0.02, help="Action noise std (rad)")
    parser.add_argument("--no-videos", action="store_true", help="Save as images instead of MP4")
    parser.add_argument("--seed", type=int, default=0, help="Base random seed")
    return parser.parse_args()


def main():
    args = parse_args()

    from norma_sim.lerobot_robot import NormaSimRobot, NormaSimRobotConfig

    print(f"=== Batch Data Generator ===")
    print(f"  Episodes:     {args.episodes}")
    print(f"  FPS:          {args.fps}")
    print(f"  Dataset:      {args.dataset_dir}")
    print(f"  Action noise: {args.action_noise} rad")
    print(f"  Videos:       {not args.no_videos}")
    print(f"  Base seed:    {args.seed}")
    print()

    # ── 1. Create robot ──
    config = NormaSimRobotConfig(
        manifest_path=str(MANIFEST),
        physics_hz=500,
        action_hz=args.fps,
        render_port=0,  # headless for speed
        cameras=["top", "wrist.top"],
    )
    robot = NormaSimRobot(config)
    robot.connect()
    print(f"Robot connected. Obs features: {list(robot.observation_features.keys())}")

    # ── 2. Create LeRobot dataset ──
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    motor_names = [f"{n}.pos" for n in robot.JOINT_NAMES] + [f"{robot.GRIPPER_NAME}.pos"]

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(motor_names),),
            "names": {"motors": motor_names},
        },
        "action": {
            "dtype": "float32",
            "shape": (len(motor_names),),
            "names": {"motors": motor_names},
        },
    }
    for cam_name in config.cameras:
        features[f"observation.images.{cam_name}"] = {
            "dtype": "image",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channel"],
        }

    dataset_dir = Path(args.dataset_dir)
    dataset_dir.mkdir(parents=True, exist_ok=True)

    use_videos = not args.no_videos
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features=features,
        root=str(dataset_dir),
        robot_type="norma_sim",
        use_videos=use_videos,
        image_writer_processes=2 if use_videos else 0,
        image_writer_threads=4 if use_videos else 1,
    )

    # ── 3. Record episodes ──
    print(f"\nRecording {args.episodes} episodes...\n")
    t0 = time.monotonic()

    for ep in range(args.episodes):
        rng = np.random.default_rng(seed=args.seed + ep)
        waypoints = generate_waypoints(rng)

        # Reset sim between episodes for clean state
        robot.reset()

        # Start from this episode's randomized home
        current_joints = list(waypoints[0][1])
        current_gripper = float(waypoints[0][2])
        frame_count = 0

        for wp_name, target_joints, target_gripper, n_steps in waypoints:
            start_joints = list(current_joints)
            start_gripper = current_gripper

            for step in range(n_steps):
                t = (step + 1) / n_steps

                # Interpolate toward target
                joints = interpolate(start_joints, target_joints, t)
                gripper = start_gripper + (target_gripper - start_gripper) * t

                # Add action noise (not to gripper — it's binary open/close)
                noisy_joints = [j + rng.normal(0, args.action_noise) for j in joints]

                # Build and send action (LeRobot 0-100 gripper scale)
                action = {}
                for i, name in enumerate(robot.JOINT_NAMES):
                    action[f"{name}.pos"] = noisy_joints[i]
                action[f"{robot.GRIPPER_NAME}.pos"] = gripper

                robot.send_action(action)
                obs = robot.get_observation()

                # State vector: actual observed positions
                state = [obs[f"{n}.pos"] for n in robot.JOINT_NAMES]
                state.append(obs[f"{robot.GRIPPER_NAME}.pos"])

                # Action vector: commanded targets (with noise)
                action_vec = list(noisy_joints) + [gripper]

                frame = {
                    "observation.state": np.array(state, dtype=np.float32),
                    "action": np.array(action_vec, dtype=np.float32),
                    "task": "pick up the red cube and place it to the side",
                }

                for cam_name in config.cameras:
                    img_key = f"observation.images.{cam_name}"
                    if img_key in obs:
                        frame[img_key] = obs[img_key]

                dataset.add_frame(frame)
                frame_count += 1

            current_joints = target_joints
            current_gripper = float(target_gripper)

        dataset.save_episode()

        elapsed = time.monotonic() - t0
        eps_per_sec = (ep + 1) / elapsed
        eta = (args.episodes - ep - 1) / eps_per_sec if eps_per_sec > 0 else 0
        print(
            f"  Episode {ep + 1:4d}/{args.episodes} | "
            f"{frame_count:3d} frames | "
            f"{elapsed:.0f}s elapsed | "
            f"ETA {eta:.0f}s"
        )

    # ── 4. Finalize ──
    dataset.consolidate()

    elapsed_total = time.monotonic() - t0
    print(f"\n=== Done ===")
    print(f"  Directory:  {dataset_dir}")
    print(f"  Episodes:   {dataset.num_episodes}")
    print(f"  Frames:     {dataset.num_frames}")
    print(f"  Features:   {list(dataset.features.keys())}")
    print(f"  Time:       {elapsed_total:.1f}s ({elapsed_total / 60:.1f} min)")

    robot.disconnect()


if __name__ == "__main__":
    main()
