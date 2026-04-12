#!/usr/bin/env python3
"""Run ACT policy through Station (Path B) — same pipeline as real hardware.

Prerequisites:
    # Terminal 1: start Station + realtime sim
    make sim-debug-so101

    # Terminal 2: run this script
    cd software/sim-server
    python3 test_policy_station.py

This connects to Station via station_py, reads motor state from
st3215/inference queue, runs ACT policy inference, and sends ST3215
GoalPosition commands. The bridge translates to/from the realtime sim.

Camera: uses dummy images (Station sim doesn't provide camera frames).
"""
from __future__ import annotations

import asyncio
import logging
import math
import struct
import sys
import time
from pathlib import Path

import numpy as np
import torch

# Add paths
repo_root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(repo_root / "software/station/shared"))
sys.path.insert(0, str(repo_root))

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
logger = logging.getLogger("policy_station")

# ── Constants ──
STEPS_PER_REV = 4096
RAD_PER_STEP = 2.0 * math.pi / STEPS_PER_REV
PRESENT_POSITION_ADDR = 0x38
GOAL_POSITION_ADDR = 0x2A

# Motor config matching therobotstudio-so101.yaml preset
MOTORS = [
    {"name": "shoulder_pan",  "motor_id": 1, "offset": 2048},
    {"name": "shoulder_lift", "motor_id": 2, "offset": 2048},
    {"name": "elbow_flex",    "motor_id": 3, "offset": 2048},
    {"name": "wrist_flex",    "motor_id": 4, "offset": 2048},
    {"name": "wrist_roll",    "motor_id": 5, "offset": 2048},
    {"name": "gripper",       "motor_id": 6, "offset": 2048},
]
BUS_SERIAL = "sim://therobotstudio-so101"


def steps_to_rad(steps: int, offset: int) -> float:
    return (steps - offset) * RAD_PER_STEP


def rad_to_steps(rad: float, offset: int) -> int:
    return max(0, min(4095, round(rad / RAD_PER_STEP) + offset))


def parse_position(state_bytes: bytes) -> int:
    if len(state_bytes) >= PRESENT_POSITION_ADDR + 2:
        raw = struct.unpack("<H", state_bytes[PRESENT_POSITION_ADDR:PRESENT_POSITION_ADDR + 2])[0]
        # Handle sign-magnitude encoding
        if raw & 0x8000:
            return (4096 - (raw & 0x0FFF)) & 0x0FFF
        return raw & 0x0FFF
    return 0


async def main():
    # ── 1. Load ACT policy ──
    logger.info("Loading ACT policy...")
    from lerobot.policies.act.modeling_act import ACTPolicy

    model_id = "CursedRock17/so101_block_grab_act"
    policy = ACTPolicy.from_pretrained(model_id)
    policy.eval()
    device = torch.device("cpu")
    policy = policy.to(device)

    config = policy.config
    logger.info(f"Policy: {model_id}")
    logger.info(f"  inputs: {list(config.input_features.keys())}")

    image_keys = []
    for key, feat in config.input_features.items():
        if "image" in key.lower():
            shape = tuple(feat.shape if hasattr(feat, "shape") else feat["shape"])
            image_keys.append((key, shape))

    # ── 1b. Local MuJoCo model for camera rendering ──
    import mujoco
    mjcf_path = repo_root / "hardware/elrobot/simulation/vendor/therobotstudio/SO101/scene.xml"
    mj_model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    mj_data = mujoco.MjData(mj_model)

    # Camera configs matching stepping.py defaults
    from norma_sim.scheduler.stepping import DEFAULT_CAMERAS
    renderers = {}
    for key, shape in image_keys:
        cam_name = key.replace("observation.images.", "")
        if cam_name in DEFAULT_CAMERAS:
            cfg = DEFAULT_CAMERAS[cam_name]
            renderers[key] = {
                "renderer": mujoco.Renderer(mj_model, height=cfg.height, width=cfg.width),
                "cam_cfg": cfg,
            }
            logger.info(f"  local camera: {cam_name} ({cfg.width}x{cfg.height})")
    has_cameras = bool(renderers)
    if has_cameras:
        logger.info(f"  camera rendering: LOCAL MuJoCo (real images!)")
    else:
        logger.info(f"  camera rendering: dummy (no matching presets)")

    # ── 2. Connect to Station ──
    logger.info("Connecting to Station...")
    from station_py import new_station_client, send_commands, StreamEntry
    from target.gen_python.protobuf.station import commands as cmd_pb
    from target.gen_python.protobuf.station import drivers as drv_pb
    from target.gen_python.protobuf.drivers.st3215 import st3215 as st3215_pb

    client = await new_station_client("localhost", logger)
    logger.info("Connected to Station")

    # ── 3. Subscribe to motor state ──
    state_queue = asyncio.Queue()
    error_queue = client.follow("st3215/inference", state_queue)
    logger.info("Subscribed to st3215/inference")

    # Wait for first state
    logger.info("Waiting for first motor state...")
    latest_motor_positions = {}  # motor_id → steps

    async def read_state_once():
        entry = await asyncio.wait_for(state_queue.get(), timeout=5.0)
        data = bytes(entry.Data)
        reader = st3215_pb.InferenceStateReader(memoryview(data))
        for bus_state in reader.get_buses():
            for motor_state in bus_state.get_motors():
                mid = motor_state.get_id()
                state_bytes = bytes(motor_state.get_state())
                pos = parse_position(state_bytes)
                latest_motor_positions[mid] = pos

    await read_state_once()
    logger.info(f"Initial positions: {latest_motor_positions}")

    # ── 4. Policy loop ──
    logger.info("")
    logger.info("=== Running ACT policy through Station (Path B) ===")
    logger.info(f"=== Camera: {'LOCAL MuJoCo render' if has_cameras else 'dummy'} ===")
    logger.info("=== Watch :8889 Web UI ===")
    logger.info("=== Ctrl+C to stop ===")
    logger.info("")

    action_hz = 30
    try:
        for step_i in range(300):
            t0 = time.monotonic()

            # Drain latest state (non-blocking)
            while not state_queue.empty():
                try:
                    entry = state_queue.get_nowait()
                    data = bytes(entry.Data)
                    reader = st3215_pb.InferenceStateReader(memoryview(data))
                    for bus_state in reader.get_buses():
                        for motor_state in bus_state.get_motors():
                            mid = motor_state.get_id()
                            state_bytes = bytes(motor_state.get_state())
                            latest_motor_positions[mid] = parse_position(state_bytes)
                except asyncio.QueueEmpty:
                    break

            # Build obs: steps → radians
            joint_rads = []
            for m in MOTORS:
                steps = latest_motor_positions.get(m["motor_id"], m["offset"])
                joint_rads.append(steps_to_rad(steps, m["offset"]))

            state_tensor = torch.tensor(joint_rads, dtype=torch.float32).unsqueeze(0).to(device)
            batch = {"observation.state": state_tensor}

            # Camera images: render locally from joint state
            if has_cameras:
                # Copy joint rads into local MuJoCo data
                for j, m in enumerate(MOTORS):
                    # Find joint qposadr for this motor
                    joint_idx = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, m["name"])
                    if joint_idx >= 0:
                        mj_data.qpos[mj_model.jnt_qposadr[joint_idx]] = joint_rads[j]
                mujoco.mj_forward(mj_model, mj_data)

                for key, shape in image_keys:
                    if key in renderers:
                        r = renderers[key]
                        cam = mujoco.MjvCamera()
                        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                        cam.lookat[:] = r["cam_cfg"].lookat
                        cam.distance = r["cam_cfg"].distance
                        cam.azimuth = r["cam_cfg"].azimuth
                        cam.elevation = r["cam_cfg"].elevation
                        r["renderer"].update_scene(mj_data, camera=cam)
                        pixels = r["renderer"].render()
                        img = pixels.astype(np.float32) / 255.0
                        img_t = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).to(device)
                        batch[key] = img_t
                    else:
                        batch[key] = torch.zeros(1, *shape, dtype=torch.float32).to(device)
            else:
                for key, shape in image_keys:
                    batch[key] = torch.zeros(1, *shape, dtype=torch.float32).to(device)

            # Inference
            with torch.no_grad():
                action_tensor = policy.select_action(batch)
            action_np = action_tensor.squeeze(0).cpu().numpy()

            # Send commands: radians → steps → ST3215 GoalPosition
            command_list = []
            for i, m in enumerate(MOTORS):
                goal_steps = rad_to_steps(float(action_np[i]), m["offset"])
                st3215_cmd = st3215_pb.Command(
                    target_bus_serial=BUS_SERIAL,
                    write=st3215_pb.ST3215WriteCommand(
                        motor_id=m["motor_id"],
                        address=GOAL_POSITION_ADDR,
                        value=goal_steps.to_bytes(2, byteorder="little"),
                    ),
                )
                command_list.append(
                    cmd_pb.DriverCommand(
                        type=drv_pb.StationCommandType.STC_ST3215_COMMAND,
                        body=st3215_cmd.encode(),
                    )
                )
            await send_commands(client, command_list)

            # Pace
            elapsed = time.monotonic() - t0
            sleep_time = (1.0 / action_hz) - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

            if step_i % 30 == 0:
                logger.info(
                    f"step {step_i:3d}: "
                    f"action[0]={action_np[0]:+.3f}rad "
                    f"→ {rad_to_steps(float(action_np[0]), 2048)} steps  "
                    f"state[0]={joint_rads[0]:+.3f}rad"
                )

    except KeyboardInterrupt:
        logger.info("(stopped)")

    logger.info("Done.")


if __name__ == "__main__":
    asyncio.run(main())
