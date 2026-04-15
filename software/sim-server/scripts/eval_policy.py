#!/usr/bin/env python3
"""Evaluate a trained policy in NormaSimEnv.

Loads a locally-trained ACT (or other LeRobot) checkpoint, runs
multi-episode rollouts in the simulator, and reports action statistics.

Usage:
    cd software/sim-server

    # Headless, pure numeric evaluation
    PYTHONPATH=. python3 scripts/eval_policy.py \
      --checkpoint outputs/act_pick_v1/checkpoints/050000 \
      --episodes 20

    # With mjviser visualization
    PYTHONPATH=. python3 scripts/eval_policy.py \
      --checkpoint outputs/act_pick_v1/checkpoints/050000 \
      --episodes 5 --render-port 8012
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch

# Ensure norma_sim is importable
_sim_server_dir = str(Path(__file__).resolve().parents[1])
if _sim_server_dir not in sys.path:
    sys.path.insert(0, _sim_server_dir)

REPO_ROOT = Path(__file__).resolve().parents[3]
MANIFEST = REPO_ROOT / "hardware/elrobot/simulation/manifests/norma/therobotstudio_so101_tabletop.scene.yaml"

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
MOTOR_NAMES = JOINT_NAMES + ["gripper"]


def load_policy(checkpoint_path: str, device: torch.device):
    """Load ACT policy from a local checkpoint directory."""
    from lerobot.policies.act.modeling_act import ACTPolicy

    policy = ACTPolicy.from_pretrained(checkpoint_path)
    policy.eval()
    policy = policy.to(device)

    print(f"Policy loaded: {checkpoint_path}")
    print(f"  input_features:  {list(policy.config.input_features.keys())}")
    action_feat = policy.config.output_features["action"]
    shape = action_feat.shape if hasattr(action_feat, "shape") else action_feat.get("shape")
    print(f"  action shape:    {shape}")
    print(f"  chunk_size:      {policy.config.chunk_size}")
    return policy


def build_batch(
    obs: dict,
    image_keys: list[tuple[str, tuple]],
    device: torch.device,
) -> dict[str, torch.Tensor]:
    """Convert robot observation → policy input batch."""
    state = torch.tensor(
        [obs[f"{n}.pos"] for n in JOINT_NAMES] + [obs["gripper.pos"]],
        dtype=torch.float32,
    ).unsqueeze(0).to(device)

    batch: dict[str, torch.Tensor] = {"observation.state": state}

    for key, shape in image_keys:
        obs_key = f"observation.images.{key.replace('observation.images.', '')}"
        if obs_key in obs and isinstance(obs[obs_key], np.ndarray):
            img = obs[obs_key].astype(np.float32) / 255.0
            img = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).to(device)
        else:
            img = torch.zeros(1, *shape, dtype=torch.float32, device=device)
        batch[key] = img

    return batch


def run_episode(
    robot,
    policy,
    image_keys: list[tuple[str, tuple]],
    device: torch.device,
    max_steps: int,
    realtime: bool,
    action_hz: int,
    object_key: str | None = None,
) -> dict:
    """Run one evaluation episode, return action/state trajectories.

    If ``object_key`` (e.g. ``"object.cube.pos"``) is provided, also
    track the peak z-height of that object across the episode so the
    caller can compute a strict "was actually lifted" success metric.
    """
    actions = []
    states = []
    final_obs: dict = {}
    initial_z: float | None = None
    peak_z: float | None = None

    for step_i in range(max_steps):
        obs = robot.get_observation()
        batch = build_batch(obs, image_keys, device)

        with torch.no_grad():
            action_tensor = policy.select_action(batch)

        action_np = action_tensor.squeeze(0).cpu().numpy()

        # Send to sim
        action_dict = {}
        for i, name in enumerate(JOINT_NAMES):
            action_dict[f"{name}.pos"] = float(action_np[i])
        action_dict["gripper.pos"] = float(action_np[5])
        robot.send_action(action_dict)

        actions.append(action_np.copy())
        state_vec = [obs[f"{n}.pos"] for n in JOINT_NAMES] + [obs["gripper.pos"]]
        states.append(np.array(state_vec, dtype=np.float32))

        if object_key and object_key in obs:
            z = float(obs[object_key][2])
            if initial_z is None:
                initial_z = z
                peak_z = z
            else:
                peak_z = max(peak_z, z) if peak_z is not None else z

        if realtime:
            time.sleep(1.0 / action_hz)

    # Grab one more obs after the last send_action so the success check
    # sees the fully-settled final state.
    final_obs = robot.get_observation()

    return {
        "actions": np.array(actions),
        "states": np.array(states),
        "final_obs": final_obs,
        "initial_object_z": initial_z,
        "peak_object_z": peak_z,
    }


def print_summary(results: list[dict], episodes: int):
    """Print action/state statistics + task success rate across all episodes."""
    all_actions = np.concatenate([r["actions"] for r in results])
    all_states = np.concatenate([r["states"] for r in results])

    print(f"\n{'=' * 60}")
    print(f"Evaluation Summary ({episodes} episodes, {len(all_actions)} total steps)")
    print(f"{'=' * 60}")

    # Task success rate — two metrics:
    #   1. Loose: task.check_success() — horizontal displacement only
    #   2. Strict: horizontal displacement AND cube was actually lifted
    #      above initial z at some point (requires peak_z tracking)
    outcomes = [r.get("success") for r in results]
    determinable = [o for o in outcomes if o is not None]
    print(f"\nTask success:")
    if determinable:
        loose_hits = sum(1 for o in determinable if o)
        loose_rate = loose_hits / len(determinable)
        undetermined = len(outcomes) - len(determinable)
        print(f"  loose  (horizontal displacement only):  "
              f"{loose_rate:.1%}  ({loose_hits}/{len(determinable)})")

        # Strict: AND cube was lifted above initial z
        strict_determinable = [
            r for r in results
            if r.get("success") is not None
            and r.get("peak_object_z") is not None
            and r.get("initial_object_z") is not None
        ]
        if strict_determinable:
            lift_threshold = 0.005  # cube must rise >5mm above start
            strict_hits = sum(
                1 for r in strict_determinable
                if r["success"] is True
                and (r["peak_object_z"] - r["initial_object_z"]) > lift_threshold
            )
            strict_rate = strict_hits / len(strict_determinable)
            avg_lift = np.mean([
                r["peak_object_z"] - r["initial_object_z"]
                for r in strict_determinable
            ])
            max_lift = max(
                r["peak_object_z"] - r["initial_object_z"]
                for r in strict_determinable
            )
            print(f"  strict (also lifted >{lift_threshold * 1000:.0f}mm above start):  "
                  f"{strict_rate:.1%}  ({strict_hits}/{len(strict_determinable)})")
            print(f"  peak lift stats: mean={avg_lift * 1000:+.1f}mm  max={max_lift * 1000:+.1f}mm")
            if strict_hits == 0 and loose_hits > 0:
                print(f"  WARNING: loose-success episodes never lifted the object — "
                      f"arm is pushing, not picking")
        if undetermined:
            print(f"  undetermined:  {undetermined}  (missing object pose in obs)")
    else:
        print(f"  UNKNOWN (task.check_success returned None for all episodes)")

    print(f"\nAction statistics:")
    print(f"  {'joint':15s}  {'mean':>8s}  {'std':>7s}  {'min':>8s}  {'max':>8s}")
    print(f"  {'-' * 50}")
    for i, name in enumerate(MOTOR_NAMES):
        col = all_actions[:, i]
        print(f"  {name:15s}  {col.mean():+8.3f}  {col.std():7.3f}  {col.min():+8.3f}  {col.max():+8.3f}")

    print(f"\nState statistics:")
    print(f"  {'joint':15s}  {'mean':>8s}  {'std':>7s}  {'min':>8s}  {'max':>8s}")
    print(f"  {'-' * 50}")
    for i, name in enumerate(MOTOR_NAMES):
        col = all_states[:, i]
        print(f"  {name:15s}  {col.mean():+8.3f}  {col.std():7.3f}  {col.min():+8.3f}  {col.max():+8.3f}")

    # Check for degenerate behavior
    action_range = all_actions.max() - all_actions.min()
    state_range = all_states.max() - all_states.min()
    print(f"\nHealth checks:")
    print(f"  Action range:  {action_range:.3f}  {'OK' if action_range > 0.1 else 'WARNING: near-constant actions'}")
    print(f"  State range:   {state_range:.3f}  {'OK' if state_range > 0.1 else 'WARNING: robot barely moved'}")
    has_nan = np.isnan(all_actions).any() or np.isnan(all_states).any()
    print(f"  NaN check:     {'FAIL — NaN detected!' if has_nan else 'OK'}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate a trained policy in NormaSimEnv")
    parser.add_argument("--checkpoint", required=True, help="Path to local checkpoint directory")
    parser.add_argument("--episodes", type=int, default=10, help="Number of evaluation episodes")
    parser.add_argument("--max-steps", type=int, default=300, help="Max steps per episode (300 ~ 10s at 30Hz)")
    parser.add_argument("--render-port", type=int, default=0, help="mjviser port (0 = headless)")
    parser.add_argument("--device", default="cuda", help="torch device")
    parser.add_argument("--realtime", action="store_true", help="Pace to wall-clock (for visualization)")
    return parser.parse_args()


def main():
    args = parse_args()

    device = torch.device(args.device)
    if args.device == "cuda" and not torch.cuda.is_available():
        print("CUDA not available, falling back to CPU")
        device = torch.device("cpu")

    # If render port is set, default to realtime pacing
    realtime = args.realtime or args.render_port > 0

    # ── 1. Load policy ──
    policy = load_policy(args.checkpoint, device)

    # Detect which cameras the policy expects
    image_keys: list[tuple[str, tuple]] = []
    for key, feat in policy.config.input_features.items():
        if "image" in key.lower():
            shape = tuple(feat.shape if hasattr(feat, "shape") else feat["shape"])
            image_keys.append((key, shape))
    cam_names = [k.replace("observation.images.", "") for k, _ in image_keys]
    print(f"  cameras needed: {cam_names if cam_names else '(none)'}")

    # ── 2. Instantiate task (defines success criterion + tracked objects) ──
    from norma_sim.tasks.pick_and_place import PickAndPlace
    task = PickAndPlace()
    tracked = [task.object_body_name]
    print(f"Task: {task.name!r} — tracking object body: {tracked}")

    # ── 3. Create robot ──
    from norma_sim.lerobot_robot import NormaSimRobot, NormaSimRobotConfig

    robot_config = NormaSimRobotConfig(
        manifest_path=str(MANIFEST),
        physics_hz=500,
        action_hz=30,
        render_port=args.render_port,
        cameras=cam_names,
        tracked_objects=tracked,
    )
    robot = NormaSimRobot(robot_config)
    robot.connect()
    print(f"Robot connected. Obs features: {list(robot.observation_features.keys())}")

    if args.render_port:
        print(f"\n  Open http://localhost:{args.render_port} to watch\n")

    # ── 4. Run evaluation episodes ──
    object_key = f"object.{task.object_body_name}.pos"
    results = []
    for ep in range(args.episodes):
        robot.reset()

        t0 = time.monotonic()
        result = run_episode(
            robot, policy, image_keys, device,
            args.max_steps, realtime, robot_config.action_hz,
            object_key=object_key,
        )
        elapsed = time.monotonic() - t0

        # Score the episode via task.check_success (returns True/False/None)
        result["success"] = task.check_success(result["final_obs"])
        results.append(result)

        # Per-episode quick summary
        act = result["actions"]
        outcome = result["success"]
        outcome_str = "[ok]  success " if outcome is True else "[fail]        " if outcome is False else "[?]   unknown "
        print(
            f"  Episode {ep + 1:3d}/{args.episodes} | "
            f"{len(act)} steps | "
            f"{elapsed:.1f}s | "
            f"{outcome_str} | "
            f"action_mean={act.mean():+.3f} action_std={act.std():.3f}"
        )

    # ── 5. Summary ──
    print_summary(results, args.episodes)

    robot.disconnect()
    print("\nDone.")


if __name__ == "__main__":
    main()
