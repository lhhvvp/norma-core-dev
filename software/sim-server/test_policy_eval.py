#!/usr/bin/env python3
"""End-to-end policy evaluation in NormaCore sim.

Downloads a pre-trained ACT checkpoint from HuggingFace, runs inference
with NormaSimRobot, and visualizes in mjviser.

Uses real MuJoCo-rendered camera images when cameras are configured.
Falls back to dummy images (zeros) if no cameras are available.

Usage:
    fuser -k 8012/tcp 2>/dev/null
    cd software/sim-server
    python3 test_policy_eval.py
"""
from pathlib import Path
import time

import numpy as np
import torch


def main():
    repo = Path(__file__).resolve().parents[2]
    manifest = (
        repo / "hardware/elrobot/simulation/manifests/norma"
        / "therobotstudio_so101.scene.yaml"
    )
    if not manifest.exists():
        print(f"ERROR: {manifest} not found")
        return 1

    # ── 1. Load policy from HuggingFace ──
    print("Loading ACT policy from HuggingFace...")
    from lerobot.policies.act.modeling_act import ACTPolicy

    model_id = "CursedRock17/so101_block_grab_act"
    policy = ACTPolicy.from_pretrained(model_id)
    policy.eval()

    device = torch.device("cpu")
    policy = policy.to(device)

    config = policy.config
    print(f"Policy: {model_id}")
    print(f"  input_features: {list(config.input_features.keys())}")
    action_feat = config.output_features['action']
    action_shape = action_feat.shape if hasattr(action_feat, 'shape') else action_feat.get('shape')
    print(f"  output: action dim={action_shape}")
    print(f"  chunk_size: {config.chunk_size}")

    # ── 2. Load preprocessor/postprocessor for normalization ──
    from lerobot.policies.act.modeling_act import ACTPolicy
    # The policy has built-in normalize/unnormalize if stats are loaded
    # For dummy eval, we skip normalization and just get raw actions

    # ── 3. Create NormaSimRobot ──
    print("\nCreating NormaSimRobot with mjviser on :8012...")
    from norma_sim.lerobot_robot import NormaSimRobot, NormaSimRobotConfig

    # Enable cameras matching what the policy expects
    policy_cam_names = [
        k.replace("observation.images.", "")
        for k in config.input_features
        if "image" in k.lower()
    ]
    print(f"  policy needs cameras: {policy_cam_names}")

    robot_config = NormaSimRobotConfig(
        manifest_path=str(manifest),
        physics_hz=500,
        action_hz=30,
        render_port=8012,
        cameras=policy_cam_names,
    )
    robot = NormaSimRobot(robot_config)
    robot.connect()

    print(f"  obs features: {list(robot.observation_features.keys())}")
    print(f"  action features: {list(robot.action_features.keys())}")

    # ── 4. Build observation dict for policy ──
    # Policy expects: observation.state (6,) + observation.images.* (3, H, W)
    # We provide real state + dummy images

    # Figure out image shapes from policy config
    image_keys = []
    for key, feat in config.input_features.items():
        if "image" in key.lower():
            shape = tuple(feat.shape if hasattr(feat, 'shape') else feat["shape"])
            image_keys.append((key, shape))
            print(f"  policy expects image: {key} shape={shape}")

    # ── 5. Run eval loop ──
    print(f"\n--- Open http://localhost:8012 to watch ---")
    print(f"--- Running 300 steps (~10 sec at 30Hz) ---")
    has_real_cameras = bool(policy_cam_names)
    if has_real_cameras:
        print(f"--- Using REAL MuJoCo-rendered camera images ---\n")
    else:
        print(f"--- No cameras configured, using dummy images ---\n")

    try:
        for step_i in range(300):
            # Get real state + camera from sim
            obs = robot.get_observation()
            state = torch.tensor([
                obs["shoulder_pan.pos"],
                obs["shoulder_lift.pos"],
                obs["elbow_flex.pos"],
                obs["wrist_flex.pos"],
                obs["wrist_roll.pos"],
                obs["gripper.pos"],
            ], dtype=torch.float32).unsqueeze(0).to(device)  # (1, 6)

            # Build policy input batch
            batch = {"observation.state": state}

            # Add camera images (real from MuJoCo if available, dummy otherwise)
            for key, shape in image_keys:
                obs_key = f"observation.images.{key.replace('observation.images.', '')}"
                if obs_key in obs and isinstance(obs[obs_key], np.ndarray):
                    # Real rendered image: (H, W, 3) uint8 → (1, 3, H, W) float32
                    img = obs[obs_key].astype(np.float32) / 255.0
                    img = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).to(device)
                else:
                    img = torch.zeros(1, *shape, dtype=torch.float32).to(device)
                batch[key] = img

            # Run inference
            with torch.no_grad():
                action_tensor = policy.select_action(batch)  # (1, action_dim)

            # Convert to robot action dict
            action_np = action_tensor.squeeze(0).cpu().numpy()  # (6,)
            action_dict = {}
            for i, name in enumerate(robot.JOINT_NAMES):
                action_dict[f"{name}.pos"] = float(action_np[i])
            action_dict["gripper.pos"] = float(action_np[5])

            # Send to sim
            robot.send_action(action_dict)

            # Pace to real-time
            time.sleep(1.0 / robot_config.action_hz)

            if step_i % 30 == 0:
                print(
                    f"  step {step_i:3d}: "
                    f"action[0]={action_np[0]:+.3f} "
                    f"state[0]={obs['shoulder_pan.pos']:+.3f}"
                )

    except KeyboardInterrupt:
        print("\n  (stopped)")

    # ── 6. Cleanup ──
    print("\nDisconnecting...")
    robot.disconnect()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
