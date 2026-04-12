"""`NormaSimEnv` — Gymnasium env that talks directly to norma_sim.

Supports position control for joints/grippers and optional camera
rendering (pass ``cameras=["top", "wrist.top"]`` to enable).

Usage::

    env = NormaSimEnv(manifest_path="path/to/scene.yaml")
    obs, info = env.reset()
    for _ in range(100):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
    env.close()

The env launches ``norma_sim --mode stepping`` as a subprocess and
communicates over a Unix domain socket using the Envelope protocol.
Each ``step()`` sends an ActuationBatch + StepRequest and waits for
the synchronous StepResponse — deterministic, no dropped frames.
"""
from __future__ import annotations

import asyncio
import os
import signal
import struct
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any, Optional

import gymnasium as gym
import numpy as np

# Import the codec layer for proto encode/decode.
# The _proto shim handles sys.path; this module must be imported
# after the norma_sim package is on sys.path (either via pip -e
# or by running from the sim-server directory).
from .ipc.codec import (
    ActuationBatch,
    ActuationCommand,
    ActuatorRef,
    Envelope,
    Hello,
    Goodbye,
    ResetRequest,
    SetPosition,
    StepRequest,
    WorldSnapshot,
    decode_envelope,
    encode_envelope,
)


PROTOCOL_VERSION = 1


class NormaSimEnv(gym.Env):
    """Gymnasium env wrapping norma_sim in stepping mode.

    Parameters
    ----------
    manifest_path : str | Path
        Path to the ``.scene.yaml`` manifest.
    physics_hz : int
        MuJoCo physics rate (default 500).
    action_hz : int
        Env step rate — each ``step()`` advances
        ``physics_hz / action_hz`` physics ticks (default 30).
    auto_launch : bool
        If True (default), launch norma_sim as a subprocess.
        If False, connect to an existing instance at *socket_path*.
    socket_path : str | Path | None
        UDS path.  If None, a temp path is generated.
    """

    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        manifest_path: str | Path,
        physics_hz: int = 500,
        action_hz: int = 30,
        auto_launch: bool = True,
        socket_path: str | Path | None = None,
        render_port: int = 0,
        cameras: list[str] | None = None,
    ) -> None:
        super().__init__()
        self.manifest_path = Path(manifest_path).resolve()
        self.physics_hz = physics_hz
        self.action_hz = action_hz
        self.n_substeps = round(physics_hz / action_hz)
        self.actual_action_hz = physics_hz / self.n_substeps
        self.render_port = render_port
        self.camera_names = cameras or []

        # Socket setup
        if socket_path is None:
            self._tmp_dir = tempfile.mkdtemp(prefix="norma_sim_gym_")
            self._socket_path = Path(self._tmp_dir) / "sim.sock"
        else:
            self._tmp_dir = None
            self._socket_path = Path(socket_path)

        self._process: Optional[subprocess.Popen] = None
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._descriptor = None
        self._actuator_ids: list[tuple[str, str]] = []  # (robot_id, actuator_id)
        self._actuator_limits: list[tuple[float, float]] = []  # (min, max) rad
        self._gripper_indices: list[int] = []
        self._joint_indices: list[int] = []

        # Launch and connect
        if auto_launch:
            self._launch_sim()
        self._connect()
        self._build_spaces()

    # ── Lifecycle ──

    def _launch_sim(self) -> None:
        cmd = [
            "python3", "-m", "norma_sim",
            "--manifest", str(self.manifest_path),
            "--socket", str(self._socket_path),
            "--physics-hz", str(self.physics_hz),
            "--mode", "stepping",
        ]
        if self.render_port > 0:
            cmd.extend(["--render-port", str(self.render_port)])
        if self.camera_names:
            cmd.extend(["--cameras"] + self.camera_names)
        # norma_sim needs its package on PYTHONPATH.  Derive from
        # this file's location: gym_env.py lives in norma_sim/, its
        # parent is the sim-server directory that must be on the path.
        sim_server_dir = str(Path(__file__).resolve().parents[1])
        env = os.environ.copy()
        existing = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = f"{sim_server_dir}:{existing}" if existing else sim_server_dir

        # When render_port is set, let subprocess output go to terminal
        # so the user can see the mjviser URL.  Otherwise capture for
        # clean error reporting.
        if self.render_port > 0:
            self._process = subprocess.Popen(cmd, env=env)
        else:
            self._process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env,
            )
        # Wait for socket to appear
        deadline = time.monotonic() + 10.0
        while not self._socket_path.exists():
            if time.monotonic() > deadline:
                self._kill_process()
                raise TimeoutError(
                    f"norma_sim did not create socket at {self._socket_path} "
                    f"within 10s"
                )
            if self._process.poll() is not None:
                stderr = ""
                if self._process.stderr is not None:
                    stderr = self._process.stderr.read().decode()
                raise RuntimeError(
                    f"norma_sim exited with code {self._process.returncode}: {stderr}"
                )
            time.sleep(0.05)

    def _connect(self) -> None:
        self._loop = asyncio.new_event_loop()
        self._reader, self._writer = self._loop.run_until_complete(
            asyncio.open_unix_connection(str(self._socket_path))
        )
        # Handshake
        hello = Envelope(
            hello=Hello(
                protocol_version=PROTOCOL_VERSION,
                client_role="gymnasium",
                client_id=f"NormaSimEnv-{os.getpid()}",
            )
        )
        self._send(hello)
        resp = self._recv()
        if resp.welcome is None:
            err_msg = resp.error.message if resp.error else "unknown"
            raise ConnectionError(f"Handshake failed: {err_msg}")
        self._descriptor = resp.welcome.world

    def _build_spaces(self) -> None:
        """Derive action/observation spaces from WorldDescriptor."""
        desc = self._descriptor
        for robot in desc.robots:
            for i, act in enumerate(robot.actuators):
                idx = len(self._actuator_ids)
                self._actuator_ids.append((robot.robot_id, act.actuator_id))

                # Prefer MJCF ctrlrange (always populated); fall back to
                # capability limits; last resort ±π.
                lo = act.ctrl_range_min
                hi = act.ctrl_range_max
                if lo == 0.0 and hi == 0.0 and act.capability is not None:
                    lo = act.capability.limit_min
                    hi = act.capability.limit_max
                if lo == 0.0 and hi == 0.0:
                    lo, hi = -3.14159, 3.14159
                self._actuator_limits.append((lo, hi))

                # Classify: gripper vs joint.
                # CAP_GRIPPER_PARALLEL (kind=3) from annotation, OR
                # heuristic: actuator name contains "gripper".
                cap = act.capability
                is_gripper = (
                    (cap is not None and cap.kind == 3)
                    or "gripper" in act.actuator_id.lower()
                )
                if is_gripper:
                    self._gripper_indices.append(idx)
                else:
                    self._joint_indices.append(idx)

        n_joints = len(self._joint_indices)
        n_grippers = len(self._gripper_indices)

        # Action space
        spaces = {}
        if n_joints > 0:
            j_low = np.array([self._actuator_limits[i][0] for i in self._joint_indices], dtype=np.float64)
            j_high = np.array([self._actuator_limits[i][1] for i in self._joint_indices], dtype=np.float64)
            spaces["joints"] = gym.spaces.Box(low=j_low, high=j_high, dtype=np.float64)
        if n_grippers > 0:
            spaces["gripper"] = gym.spaces.Box(
                low=0.0, high=1.0, shape=(n_grippers,), dtype=np.float64,
            )
        self.action_space = gym.spaces.Dict(spaces)
        self.observation_space = gym.spaces.Dict(spaces)

    def close(self) -> None:
        if self._writer is not None:
            try:
                self._send(Envelope(goodbye=Goodbye(reason="env.close()")))
            except Exception:
                pass
            try:
                self._writer.close()
                if self._loop:
                    self._loop.run_until_complete(self._writer.wait_closed())
            except Exception:
                pass
            self._writer = None
            self._reader = None
        if self._loop is not None:
            self._loop.close()
            self._loop = None
        self._kill_process()
        # Cleanup temp socket
        if self._tmp_dir is not None:
            try:
                if self._socket_path.exists():
                    self._socket_path.unlink()
                Path(self._tmp_dir).rmdir()
            except OSError:
                pass

    def _kill_process(self) -> None:
        if self._process is not None and self._process.poll() is None:
            self._process.send_signal(signal.SIGTERM)
            try:
                self._process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None

    # ── IPC helpers ──

    def _send(self, env: Envelope) -> None:
        assert self._writer is not None and self._loop is not None
        payload = encode_envelope(env)
        header = struct.pack(">I", len(payload))
        self._writer.write(header)
        if payload:
            self._writer.write(payload)
        self._loop.run_until_complete(self._writer.drain())

    def _recv(self) -> Envelope:
        assert self._reader is not None and self._loop is not None
        header = self._loop.run_until_complete(self._reader.readexactly(4))
        (length,) = struct.unpack(">I", header)
        if length == 0:
            return Envelope()
        data = self._loop.run_until_complete(self._reader.readexactly(length))
        return decode_envelope(data)

    # ── Gymnasium API ──

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        super().reset(seed=seed, options=options)
        # seed=0 means default (no randomization); non-zero reserved for
        # future domain randomization.  Always send ≥1 to avoid gremlin_py
        # empty-message bug (zero-value proto fields produce zero wire size).
        proto_seed = seed if seed and seed > 0 else 1
        self._send(Envelope(reset_request=ResetRequest(seed=proto_seed)))
        resp = self._recv()
        if resp.step_response is None or resp.step_response.snapshot is None:
            raise RuntimeError("ResetRequest did not return StepResponse")
        return self._snapshot_to_obs(resp.step_response.snapshot)

    def step(
        self, action: dict[str, Any]
    ) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        # 1. Send actuation
        commands = self._action_to_commands(action)
        self._send(Envelope(actuation=ActuationBatch(commands=commands)))
        # 2. Request step
        self._send(Envelope(step_request=StepRequest(n_ticks=self.n_substeps)))
        # 3. Receive response
        resp = self._recv()
        if resp.step_response is None or resp.step_response.snapshot is None:
            raise RuntimeError("StepRequest did not return StepResponse")
        obs, info = self._snapshot_to_obs(resp.step_response.snapshot)
        return obs, 0.0, False, False, info

    # ── Conversion helpers ──

    def _action_to_commands(self, action: dict[str, Any]) -> list:
        commands = []
        if "joints" in action:
            joints = np.asarray(action["joints"], dtype=np.float64)
            for i, idx in enumerate(self._joint_indices):
                robot_id, act_id = self._actuator_ids[idx]
                commands.append(
                    ActuationCommand(
                        ref=ActuatorRef(robot_id=robot_id, actuator_id=act_id),
                        set_position=SetPosition(value=float(joints[i])),
                    )
                )
        if "gripper" in action:
            grippers = np.asarray(action["gripper"], dtype=np.float64)
            for i, idx in enumerate(self._gripper_indices):
                robot_id, act_id = self._actuator_ids[idx]
                lo, hi = self._actuator_limits[idx]
                rad_value = float(np.clip(grippers[i], 0.0, 1.0)) * (hi - lo) + lo
                commands.append(
                    ActuationCommand(
                        ref=ActuatorRef(robot_id=robot_id, actuator_id=act_id),
                        set_position=SetPosition(value=rad_value),
                    )
                )
        return commands

    def _snapshot_to_obs(
        self, snapshot: WorldSnapshot
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        # Build a lookup: (robot_id, actuator_id) → ActuatorState
        state_map = {}
        for a in snapshot.actuators:
            if a.ref is not None:
                state_map[(a.ref.robot_id, a.ref.actuator_id)] = a

        obs: dict[str, Any] = {}
        if self._joint_indices:
            joints = np.zeros(len(self._joint_indices), dtype=np.float64)
            for i, idx in enumerate(self._joint_indices):
                key = self._actuator_ids[idx]
                if key in state_map:
                    joints[i] = state_map[key].position_value
            obs["joints"] = joints

        if self._gripper_indices:
            grippers = np.zeros(len(self._gripper_indices), dtype=np.float64)
            for i, idx in enumerate(self._gripper_indices):
                key = self._actuator_ids[idx]
                if key in state_map:
                    lo, hi = self._actuator_limits[idx]
                    rng = hi - lo
                    if rng > 0:
                        grippers[i] = (state_map[key].position_value - lo) / rng
                    else:
                        grippers[i] = 0.0
            obs["gripper"] = grippers

        # Extract camera frames from sensors
        if snapshot.sensors:
            for sensor in snapshot.sensors:
                if sensor.camera_frame is not None and sensor.ref is not None:
                    cf = sensor.camera_frame
                    cam_name = sensor.ref.sensor_id
                    pixels = np.frombuffer(cf.data, dtype=np.uint8).reshape(
                        cf.height, cf.width, 3
                    )
                    obs[f"camera.{cam_name}"] = pixels

        info = {}
        if snapshot.clock is not None:
            info["world_tick"] = snapshot.clock.world_tick
            info["sim_time_ns"] = snapshot.clock.sim_time_ns
        return obs, info
