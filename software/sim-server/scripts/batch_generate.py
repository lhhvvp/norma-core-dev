#!/usr/bin/env python3
"""Parallel batch data generator for ACT training with domain randomization.

Uses N worker processes, each running its own MuJoCo sim, to generate
episodes in parallel. Workers save raw episode data as .npz files;
the main process then builds the LeRobot dataset sequentially.

Usage:
    cd software/sim-server
    PYTHONPATH=. MUJOCO_GL=egl python3 scripts/batch_generate.py
    PYTHONPATH=. MUJOCO_GL=egl python3 scripts/batch_generate.py --episodes 50 --workers 4

Output: datasets/norma_sim_pick_v1/ (LeRobot v3 format)
"""
from __future__ import annotations

import argparse
import multiprocessing as mp
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

# Ensure norma_sim is importable
_sim_server_dir = str(Path(__file__).resolve().parents[1])
if _sim_server_dir not in sys.path:
    sys.path.insert(0, _sim_server_dir)

REPO_ROOT = Path(__file__).resolve().parents[3]
MANIFEST = str(REPO_ROOT / "hardware/elrobot/simulation/manifests/norma/therobotstudio_so101_tabletop.scene.yaml")

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_NAME = "gripper"
CAMERAS = ["top", "wrist.top"]
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
        ("home",      home,                                        0, s(30)),
        ("above",     [0.0, -0.6,  1.3, -0.1,  0.0],             0, s(40)),
        ("approach",  [0.0, -0.6,  approach_flex,  0.0,  0.0],    0, s(30)),
        ("grasp",     [0.0, -0.6,  approach_flex,  0.0,  0.0],  100, s(20)),
        ("lift",      [0.0, -0.6,  lift_flex, -0.3,  0.0],      100, s(40)),
        ("carry",     [pan, -0.4,  0.8, -0.2,  0.0],            100, s(40)),
        ("release",   [pan, -0.4,  0.8, -0.2,  0.0],              0, s(20)),
        ("home",      home,                                        0, s(40)),
    ]


# ─── Worker ────────────────────────────────────────────────────────────

def _worker_init():
    """Suppress KeyboardInterrupt in worker processes."""
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def _generate_one_episode(robot, ep_idx, base_seed, action_noise, tmp_dir):
    """Generate one episode using an already-connected robot. Returns .npz path."""
    robot.reset()

    rng = np.random.default_rng(seed=base_seed + ep_idx)
    waypoints = generate_waypoints(rng)

    states, actions, images_top, images_wrist = [], [], [], []
    current_joints = list(waypoints[0][1])
    current_gripper = float(waypoints[0][2])

    for wp_name, target_joints, target_gripper, n_steps in waypoints:
        start_joints = list(current_joints)
        start_gripper = current_gripper

        for step in range(n_steps):
            t = (step + 1) / n_steps
            joints = interpolate(start_joints, target_joints, t)
            gripper = start_gripper + (target_gripper - start_gripper) * t
            noisy_joints = [j + rng.normal(0, action_noise) for j in joints]

            action = {f"{n}.pos": noisy_joints[i] for i, n in enumerate(JOINT_NAMES)}
            action[f"{GRIPPER_NAME}.pos"] = gripper

            robot.send_action(action)
            obs = robot.get_observation()

            state_vec = [obs[f"{n}.pos"] for n in JOINT_NAMES] + [obs[f"{GRIPPER_NAME}.pos"]]
            action_vec = list(noisy_joints) + [gripper]

            states.append(np.array(state_vec, dtype=np.float32))
            actions.append(np.array(action_vec, dtype=np.float32))

            img_top = obs.get("observation.images.top")
            img_wrist = obs.get("observation.images.wrist.top")
            if img_top is not None:
                images_top.append(img_top)
            if img_wrist is not None:
                images_wrist.append(img_wrist)

        current_joints = target_joints
        current_gripper = float(target_gripper)

    out_path = os.path.join(tmp_dir, f"ep_{ep_idx:06d}.npz")
    np.savez(
        out_path,
        states=np.array(states),
        actions=np.array(actions),
        images_top=np.array(images_top) if images_top else np.array([], dtype=np.uint8),
        images_wrist=np.array(images_wrist) if images_wrist else np.array([], dtype=np.uint8),
    )
    return out_path


def _worker_batch(args: tuple) -> list[str]:
    """Worker: create ONE sim, reuse it for all episodes in batch."""
    ep_indices, base_seed, action_noise, manifest_path, fps, tmp_dir = args

    from norma_sim.lerobot_robot import NormaSimRobot, NormaSimRobotConfig

    config = NormaSimRobotConfig(
        manifest_path=manifest_path,
        physics_hz=500,
        action_hz=fps,
        render_port=0,
        cameras=CAMERAS,
    )
    robot = NormaSimRobot(config)
    robot.connect()

    paths = []
    for ep_idx in ep_indices:
        path = _generate_one_episode(robot, ep_idx, base_seed, action_noise, tmp_dir)
        paths.append(path)

    robot.disconnect()
    return paths


# ─── Main ──────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Parallel batch data generator for ACT training")
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--workers", type=int, default=0, help="Parallel workers (0=auto)")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--dataset-dir", type=str, default="datasets/norma_sim_pick_v1")
    parser.add_argument("--repo-id", type=str, default="norma/sim_pick_v1")
    parser.add_argument("--action-noise", type=float, default=0.02, help="Rad")
    parser.add_argument("--no-videos", action="store_true", help="Save as images instead of MP4")
    parser.add_argument("--seed", type=int, default=0)
    return parser.parse_args()


def main():
    args = parse_args()
    n_workers = args.workers if args.workers > 0 else min(mp.cpu_count() - 2, args.episodes, 16)
    n_workers = max(1, n_workers)

    print(f"=== Parallel Batch Data Generator ===")
    print(f"  Episodes:     {args.episodes}")
    print(f"  Workers:      {n_workers}")
    print(f"  FPS:          {args.fps}")
    print(f"  Dataset:      {args.dataset_dir}")
    print(f"  Action noise: {args.action_noise} rad")
    print(f"  Videos:       {not args.no_videos}")
    print(f"  Seed:         {args.seed}")
    print()

    # ── Phase 1: Parallel episode generation ──
    tmp_dir = tempfile.mkdtemp(prefix="norma_batch_")
    print(f"Phase 1: Generating episodes in parallel → {tmp_dir}")

    # Split episodes across workers — each worker gets a contiguous batch
    # so it can reuse one sim subprocess for multiple episodes
    indices = list(range(args.episodes))
    chunk_size = max(1, len(indices) // n_workers)
    chunks = []
    for i in range(0, len(indices), chunk_size):
        chunks.append(indices[i:i + chunk_size])
    # Rebalance: merge last tiny chunk into previous
    if len(chunks) > n_workers and len(chunks[-1]) < chunk_size // 2:
        chunks[-2].extend(chunks.pop())
    actual_workers = len(chunks)

    worker_args = [
        (chunk, args.seed, args.action_noise, MANIFEST, args.fps, tmp_dir)
        for chunk in chunks
    ]

    t0 = time.monotonic()
    completed = 0

    with mp.Pool(actual_workers, initializer=_worker_init) as pool:
        try:
            for paths in pool.imap_unordered(_worker_batch, worker_args):
                completed += len(paths)
                elapsed = time.monotonic() - t0
                rate = completed / elapsed
                eta = (args.episodes - completed) / rate if rate > 0 else 0
                print(f"  {completed:4d}/{args.episodes} episodes | {elapsed:.0f}s | {rate:.1f} ep/s | ETA {eta:.0f}s")
        except KeyboardInterrupt:
            pool.terminate()
            pool.join()
            shutil.rmtree(tmp_dir, ignore_errors=True)
            print("\nAborted.")
            return

    t_gen = time.monotonic() - t0
    print(f"\nPhase 1 done: {completed} episodes in {t_gen:.1f}s ({t_gen/60:.1f} min)\n")

    # ── Phase 2: Build LeRobot dataset from .npz files ──
    print("Phase 2: Building LeRobot dataset...")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    motor_names = [f"{n}.pos" for n in JOINT_NAMES] + [f"{GRIPPER_NAME}.pos"]
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
    for cam_name in CAMERAS:
        features[f"observation.images.{cam_name}"] = {
            "dtype": "image",
            "shape": (480, 640, 3),
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

    t1 = time.monotonic()
    total_frames = 0
    for ep_idx in range(args.episodes):
        npz_path = os.path.join(tmp_dir, f"ep_{ep_idx:06d}.npz")
        data = np.load(npz_path)
        n_frames = len(data["states"])

        for i in range(n_frames):
            frame = {
                "observation.state": data["states"][i],
                "action": data["actions"][i],
                "task": TASK,
            }
            if data["images_top"].size > 0:
                frame["observation.images.top"] = data["images_top"][i]
            if data["images_wrist"].size > 0:
                frame["observation.images.wrist.top"] = data["images_wrist"][i]
            dataset.add_frame(frame)

        dataset.save_episode()
        total_frames += n_frames
        data.close()
        os.unlink(npz_path)  # free disk as we go

        if (ep_idx + 1) % 20 == 0 or ep_idx == args.episodes - 1:
            print(f"  {ep_idx + 1:4d}/{args.episodes} episodes written | {total_frames} frames")

    t_build = time.monotonic() - t1
    print(f"Phase 2 done: {t_build:.1f}s ({t_build/60:.1f} min)\n")

    # ── Cleanup ──
    shutil.rmtree(tmp_dir, ignore_errors=True)

    elapsed_total = time.monotonic() - t0
    print(f"=== Complete ===")
    print(f"  Directory:  {dataset_dir}")
    print(f"  Episodes:   {dataset.num_episodes}")
    print(f"  Frames:     {dataset.num_frames}")
    print(f"  Features:   {list(dataset.features.keys())}")
    print(f"  Total time: {elapsed_total:.1f}s ({elapsed_total / 60:.1f} min)")
    print(f"    Phase 1 (generate): {t_gen:.1f}s")
    print(f"    Phase 2 (dataset):  {t_build:.1f}s")


if __name__ == "__main__":
    main()
