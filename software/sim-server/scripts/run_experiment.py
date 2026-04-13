#!/usr/bin/env python3
"""Run a complete experiment from a single YAML config.

Phases:
  1. Generate dataset (FastSim + Task + RobotSpec)
  2. Train policy (LeRobot)
  3. Evaluate (optional)

Usage:
    cd software/sim-server
    PYTHONPATH=. python3 scripts/run_experiment.py experiments/pick_v1.yaml
    PYTHONPATH=. python3 scripts/run_experiment.py experiments/pick_v1.yaml --phase data
    PYTHONPATH=. python3 scripts/run_experiment.py experiments/pick_v1.yaml --phase train
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

_sim_server_dir = str(Path(__file__).resolve().parents[1])
if _sim_server_dir not in sys.path:
    sys.path.insert(0, _sim_server_dir)

REPO_ROOT = Path(__file__).resolve().parents[3]


def interpolate(start: list[float], end: list[float], t: float) -> list[float]:
    return [s + (e - s) * t for s, e in zip(start, end)]


def phase_data(config):
    """Phase 1: Generate dataset using FastSim + Task + RobotSpec."""
    from norma_sim.experiment import ExperimentConfig
    from norma_sim.fast_sim import FastSim
    from norma_sim.lerobot_helpers import RobotSpec
    from norma_sim.tasks import REGISTRY
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    # ── Resolve manifest path ──
    manifest = config.robot.manifest
    if not Path(manifest).is_absolute():
        manifest = str(REPO_ROOT / manifest)

    # ── Set GL environment ──
    for k, v in config.sim.gl_env.items():
        os.environ[k] = v

    # ── Create sim ──
    cam_configs = config.camera_configs
    sim = FastSim(
        manifest_path=manifest,
        cameras=cam_configs,
        physics_hz=config.sim.physics_hz,
        action_hz=config.sim.action_hz,
    )

    # ── RobotSpec from manifest (dynamic, no hardcoded names) ──
    spec = RobotSpec.from_world(sim.world)
    print(f"Robot: {spec.n_joints} joints, {spec.n_grippers} grippers")
    print(f"  motors: {spec.motor_names}")

    # ── Task from registry ──
    task = REGISTRY[config.task.name]
    print(f"Task: {task.name} — {task.description}")

    # ── Create dataset ──
    dataset_dir = Path(config.dataset.root)
    if dataset_dir.exists():
        shutil.rmtree(dataset_dir)
    dataset_dir.parent.mkdir(parents=True, exist_ok=True)

    features = spec.build_features(cam_configs)
    dataset = LeRobotDataset.create(
        repo_id=config.dataset.repo_id,
        fps=config.sim.action_hz,
        features=features,
        root=str(dataset_dir),
        robot_type="norma_sim",
        use_videos=config.dataset.use_videos,
        image_writer_processes=4 if config.dataset.use_videos else 0,
        image_writer_threads=4,
    )

    # ── Generate episodes ──
    n_eps = config.task.episodes
    noise = config.task.action_noise
    seed = config.task.seed

    print(f"\nGenerating {n_eps} episodes (noise={noise}, seed={seed})...\n")
    t0 = time.monotonic()

    for ep in range(n_eps):
        rng = np.random.default_rng(seed=seed + ep)
        traj = task.generate_trajectory(rng)
        sim.reset()

        current_joints = list(traj.waypoints[0][1])
        current_gripper = traj.waypoints[0][2]
        frame_count = 0

        for wp_name, target_joints, target_gripper, n_steps in traj.waypoints:
            start_joints = list(current_joints)
            start_gripper = current_gripper

            for step in range(n_steps):
                t = (step + 1) / n_steps
                joints = interpolate(start_joints, target_joints, t)
                gripper = start_gripper + (target_gripper - start_gripper) * t
                noisy_joints = [j + rng.normal(0, noise) for j in joints]

                obs = sim.step(np.array(noisy_joints), gripper)

                frame = {
                    "observation.state": spec.build_state_vector(obs),
                    "action": spec.build_action_vector(noisy_joints, gripper),
                    "task": task.description,
                }
                for cam_name in cam_configs:
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
        eta = (n_eps - ep - 1) / rate if rate > 0 else 0
        if (ep + 1) % 10 == 0 or ep == 0 or ep == n_eps - 1:
            print(
                f"  {ep+1:4d}/{n_eps} | "
                f"{frame_count:3d} frames | "
                f"{rate:.1f} ep/s | "
                f"ETA {eta:.0f}s"
            )

    sim.close()
    elapsed_total = time.monotonic() - t0

    print(f"\n=== Data Generation Complete ===")
    print(f"  Dataset:   {dataset_dir}")
    print(f"  Episodes:  {dataset.num_episodes}")
    print(f"  Frames:    {dataset.num_frames}")
    print(f"  Time:      {elapsed_total:.1f}s ({elapsed_total/60:.1f} min)")
    return dataset_dir


def phase_validate(config):
    """Phase 1.5: Validate dataset quality before training."""
    from norma_sim.data_quality import validate_dataset

    print(f"\n=== Data Validation ===")
    report = validate_dataset(
        dataset_path=config.dataset.root,
        repo_id=config.dataset.repo_id,
    )
    print(report)

    if report.critical:
        print("\n✗ Critical data quality failure — aborting pipeline.")
        sys.exit(1)

    if report.n_failed > 0:
        print(f"\n⚠ {report.n_failed} episodes failed but within tolerance. Continuing.")


def phase_train(config):
    """Phase 2: Launch LeRobot training."""
    tc = config.training
    dc = config.dataset

    cmd = [
        sys.executable, "-m", "lerobot.scripts.lerobot_train",
        f"--policy.type={tc.policy}",
        f"--policy.repo_id=norma/{tc.policy}_{config.task.name}",
        "--policy.push_to_hub=false",
        f"--dataset.repo_id={dc.repo_id}",
        f"--dataset.root={dc.root}",
        f"--batch_size={tc.batch_size}",
        f"--steps={tc.steps}",
        f"--save_freq={tc.save_freq}",
        f"--log_freq={tc.log_freq}",
        f"--eval_freq=0",
        f"--num_workers={tc.num_workers}",
        f"--output_dir={tc.output_dir}",
    ]

    print(f"\n=== Training ===")
    print(f"  Command: {' '.join(cmd)}")
    print()
    subprocess.run(cmd, check=True)


def phase_eval(config):
    """Phase 3: Run evaluation on latest checkpoint."""
    ckpt_dir = Path(config.training.output_dir) / "checkpoints" / "last" / "pretrained_model"
    if not ckpt_dir.exists():
        # Try numbered checkpoint
        ckpt_base = Path(config.training.output_dir) / "checkpoints"
        numbered = sorted([d for d in ckpt_base.iterdir() if d.name.isdigit()]) if ckpt_base.exists() else []
        if numbered:
            ckpt_dir = numbered[-1] / "pretrained_model"
        else:
            print(f"No checkpoint found in {ckpt_base}")
            return

    cmd = [
        sys.executable, "scripts/eval_policy.py",
        f"--checkpoint={ckpt_dir}",
        "--episodes=5",
        "--max-steps=300",
        "--device=cuda",
    ]

    print(f"\n=== Evaluation ===")
    print(f"  Checkpoint: {ckpt_dir}")
    print()
    subprocess.run(cmd, check=True)


def main():
    parser = argparse.ArgumentParser(description="Run experiment from YAML config")
    parser.add_argument("config", help="Path to experiment YAML")
    parser.add_argument("--phase", choices=["data", "validate", "train", "eval", "all"], default="all")
    args = parser.parse_args()

    from norma_sim.experiment import ExperimentConfig
    config = ExperimentConfig.load(args.config)

    print(f"=== Experiment: {args.config} ===")
    print(f"  Robot:    {config.robot.manifest}")
    print(f"  Sim:      {config.sim.backend} @ {config.sim.physics_hz}Hz")
    print(f"  Cameras:  {config.cameras}")
    print(f"  Task:     {config.task.name} × {config.task.episodes} episodes")
    print(f"  Training: {config.training.policy}, {config.training.steps} steps")
    print()

    if args.phase in ("data", "all"):
        phase_data(config)

    if args.phase in ("data", "validate", "all"):
        phase_validate(config)

    if args.phase in ("train", "all"):
        phase_train(config)

    if args.phase in ("eval", "all"):
        phase_eval(config)


if __name__ == "__main__":
    main()
