#!/usr/bin/env python3
"""Manual test script for NormaSimEnv.

Usage:
    cd software/sim-server
    python3 test_gym_env_manual.py
"""
from pathlib import Path


def main():
    # ── 1. 找 scene.yaml ──
    repo = Path(__file__).resolve().parents[2]

    # 优先用 TheRobotStudio SO101（和 make sim-debug-so101 同一个模型）
    candidates = [
        repo / "hardware/elrobot/simulation/manifests/norma/therobotstudio_so101.scene.yaml",
        repo / "hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml",
    ]
    manifest = None
    for c in candidates:
        if c.exists():
            manifest = c
            break
    if manifest is None:
        print(f"ERROR: no scene.yaml found in {candidates}")
        return 1

    print(f"manifest: {manifest}")

    # ── 2. 创建 env ──
    from norma_sim.gym_env import NormaSimEnv

    print("creating NormaSimEnv with mjviser on :8012 ...")
    env = NormaSimEnv(
        manifest_path=manifest,
        physics_hz=500,
        action_hz=30,
        render_port=8012,
    )

    print(f"action_space:      {env.action_space}")
    print(f"observation_space:  {env.observation_space}")
    print(f"actuators:          {len(env._actuator_ids)}")
    for i, (rid, aid) in enumerate(env._actuator_ids):
        lo, hi = env._actuator_limits[i]
        kind = "gripper" if i in env._gripper_indices else "joint"
        print(f"  [{i}] {rid}/{aid}  ({kind})  range=[{lo:.4f}, {hi:.4f}] rad")

    import time

    # ── 3. reset ──
    print("\n--- env.reset() ---")
    obs, info = env.reset()
    print(f"obs keys: {list(obs.keys())}")
    if "joints" in obs:
        print(f"joints:   {obs['joints']}")
    if "gripper" in obs:
        print(f"gripper:  {obs['gripper']}")
    print(f"info:     {info}")

    # ── 4. 持续 step 循环（可视化） ──
    print("\n--- Open http://localhost:8012 in your browser ---")
    print("--- Running 300 steps with random actions (10 seconds) ---")
    print("--- Press Ctrl+C to stop early ---")
    try:
        for i in range(300):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            tick = info.get("world_tick", "?")
            if i % 30 == 0:  # print every ~1s
                j0 = obs["joints"][0] if "joints" in obs else 0
                print(f"  step {i+1}: tick={tick}  joint[0]={j0:.4f}")
            time.sleep(1.0 / env.action_hz)  # pace to real-time (~30 Hz)
    except KeyboardInterrupt:
        print("\n  (stopped by user)")

    # ── 5. 确定性验证 ──
    print("\n--- determinism check: reset + 5 steps ---")
    obs1, _ = env.reset()
    for _ in range(5):
        obs, *_ = env.step(env.action_space.sample())

    obs2, _ = env.reset()
    if "joints" in obs1 and "joints" in obs2:
        import numpy as np
        diff = np.abs(np.asarray(obs1["joints"]) - np.asarray(obs2["joints"]))
        print(f"  reset obs diff (should be ~0): max={diff.max():.2e}")
        if diff.max() < 1e-10:
            print("  PASS: reset is deterministic")
        else:
            print("  FAIL: reset states differ!")

    # ── 6. close ──
    print("\n--- env.close() ---")
    env.close()
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
