#!/usr/bin/env python3
"""Record a scripted pick-and-place demo to LeRobotDataset.

Generates a "reach → approach → grasp → lift → return" trajectory
using NormaSimRobot and saves it as a LeRobot-compatible dataset.

Usage:
    cd software/sim-server
    python3 scripts/record_scripted_demo.py

Output: datasets/norma_sim_pick_demo/ (LeRobotDataset format)
"""
from __future__ import annotations

import time
from pathlib import Path

import sys
import numpy as np

# Ensure norma_sim is importable
_sim_server_dir = str(Path(__file__).resolve().parents[1])
if _sim_server_dir not in sys.path:
    sys.path.insert(0, _sim_server_dir)

REPO_ROOT = Path(__file__).resolve().parents[3]
MANIFEST = REPO_ROOT / "hardware/elrobot/simulation/manifests/norma/therobotstudio_so101_tabletop.scene.yaml"

# Joint waypoints: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]
# Gripper: 0 = open, 100 = closed (LeRobot scale)
WAYPOINTS = [
    # (name, joints, gripper, steps)
    ("home",      [0.0,  0.0,  0.0,  0.0,  0.0],   0,  30),
    ("above",     [0.0, -0.6,  1.3, -0.1,  0.0],   0,  40),
    ("approach",  [0.0, -0.6,  1.5,  0.0,  0.0],   0,  30),
    ("grasp",     [0.0, -0.6,  1.5,  0.0,  0.0], 100,  20),
    ("lift",      [0.0, -0.6,  1.0, -0.3,  0.0], 100,  40),
    ("carry",     [0.5, -0.4,  0.8, -0.2,  0.0], 100,  40),
    ("release",   [0.5, -0.4,  0.8, -0.2,  0.0],   0,  20),
    ("home",      [0.0,  0.0,  0.0,  0.0,  0.0],   0,  40),
]


def interpolate(start, end, t):
    """Linear interpolation."""
    return [s + (e - s) * t for s, e in zip(start, end)]


def main():
    from norma_sim.lerobot_robot import NormaSimRobot, NormaSimRobotConfig

    print("Creating NormaSimRobot with cameras...")
    config = NormaSimRobotConfig(
        manifest_path=str(MANIFEST),
        physics_hz=500,
        action_hz=30,
        render_port=0,  # no mjviser for recording speed
        cameras=["top", "wrist.top"],
    )
    robot = NormaSimRobot(config)
    robot.connect()
    print(f"  obs features: {list(robot.observation_features.keys())}")

    # ── Build features dict for LeRobotDataset ──
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    features = {}

    # State: 6 floats (5 joints + gripper)
    motor_names = [f"{n}.pos" for n in robot.JOINT_NAMES] + [f"{robot.GRIPPER_NAME}.pos"]
    features["observation.state"] = {
        "dtype": "float32",
        "shape": (len(motor_names),),
        "names": {"motors": motor_names},
    }

    # Action: same shape as state
    features["action"] = {
        "dtype": "float32",
        "shape": (len(motor_names),),
        "names": {"motors": motor_names},
    }

    # Images
    for cam_name in config.cameras:
        features[f"observation.images.{cam_name}"] = {
            "dtype": "image",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channel"],
        }

    dataset_dir = Path("datasets/norma_sim_pick_demo")
    if dataset_dir.exists():
        import shutil
        shutil.rmtree(dataset_dir)
    dataset_dir.parent.mkdir(parents=True, exist_ok=True)

    dataset = LeRobotDataset.create(
        repo_id="norma/sim_pick_demo",
        fps=30,
        features=features,
        root=str(dataset_dir),
        robot_type="norma_sim",
        use_videos=False,  # save as images, simpler
        image_writer_processes=0,
        image_writer_threads=1,
    )

    # ── Record episodes ──
    num_episodes = 2
    print(f"\nRecording {num_episodes} episodes...")
    print(f"  Open http://localhost:8012 to watch\n")

    for ep in range(num_episodes):
        # Randomize cube position slightly for diversity
        # (can't actually move cube in sim without reset, but joint targets vary)
        print(f"Episode {ep + 1}/{num_episodes}")

        # Start from home
        current_joints = [0.0] * 5
        current_gripper = 0.0

        for wp_name, target_joints, target_gripper, n_steps in WAYPOINTS:
            start_joints = list(current_joints)
            start_gripper = current_gripper

            for step in range(n_steps):
                t = (step + 1) / n_steps
                # Interpolate
                joints = interpolate(start_joints, target_joints, t)
                gripper = start_gripper + (target_gripper - start_gripper) * t

                # Build action dict (LeRobot scale)
                action = {}
                for i, name in enumerate(robot.JOINT_NAMES):
                    action[f"{name}.pos"] = joints[i]
                action[f"{robot.GRIPPER_NAME}.pos"] = gripper

                # Send action
                robot.send_action(action)

                # Get observation
                obs = robot.get_observation()

                # Build state vector
                state = [obs[f"{n}.pos"] for n in robot.JOINT_NAMES]
                state.append(obs[f"{robot.GRIPPER_NAME}.pos"])

                # Build action vector
                action_vec = list(joints) + [gripper]

                # Build frame
                frame = {
                    "observation.state": np.array(state, dtype=np.float32),
                    "action": np.array(action_vec, dtype=np.float32),
                    "task": "pick up the red cube",
                }

                # Add images
                for cam_name in config.cameras:
                    img_key = f"observation.images.{cam_name}"
                    if img_key in obs:
                        frame[img_key] = obs[img_key]

                dataset.add_frame(frame)

            current_joints = target_joints
            current_gripper = target_gripper

        dataset.save_episode()
        print(f"  saved episode {ep + 1} ({dataset.num_frames} total frames)")

    # ── Finalize ──
    print(f"\nDataset saved to: {dataset_dir}")
    print(f"  Episodes: {dataset.num_episodes}")
    print(f"  Frames: {dataset.num_frames}")
    print(f"  Features: {list(dataset.features.keys())}")

    robot.disconnect()
    print("Done!")


if __name__ == "__main__":
    main()
