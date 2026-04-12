"""`SteppingScheduler` — step-on-demand for Gymnasium integration.

Unlike `RealTimeScheduler`, this scheduler does NOT run a continuous
loop.  Each call to `step(n)` advances physics by exactly *n* ticks
and returns a `WorldSnapshot` synchronously.  The caller (IPC session)
drives the pace — there is no wall-clock pacing or background thread.

This scheduler is selected with ``norma_sim --mode stepping``.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from ..cameras import CameraConfig, DEFAULT_CAMERAS  # noqa: F401 — re-exported
from ..ipc.codec import WorldClock
from ..world.actuation import ActuationApplier
from ..world.model import MuJoCoWorld
from ..world.snapshot import SnapshotBuilder

_log = logging.getLogger("norma_sim.scheduler.stepping")


class SteppingScheduler:
    """Synchronous step-on-demand scheduler."""

    def __init__(
        self,
        world: MuJoCoWorld,
        applier: ActuationApplier,
        builder: SnapshotBuilder,
        physics_hz: int = 500,
        on_render: Optional[Callable[[], None]] = None,
        cameras: dict[str, CameraConfig] | None = None,
    ) -> None:
        self.world = world
        self.applier = applier
        self.builder = builder
        self.physics_hz = physics_hz
        self._on_render = on_render
        self._tick = 0

        # Camera rendering
        self._cameras = cameras or {}
        self._renderers: dict[str, object] = {}
        if self._cameras:
            self._init_renderers()

    @property
    def tick(self) -> int:
        return self._tick

    def _make_clock(self):
        return WorldClock(
            world_tick=self._tick,
            sim_time_ns=int(self._tick * (1e9 / self.physics_hz)),
            wall_time_ns=0,
        )

    def step(self, n_ticks: int = 1):
        """Advance physics by *n_ticks* steps, return snapshot."""
        with self.world.lock:
            for _ in range(n_ticks):
                self.world.step()
                self._tick += 1
        if self._on_render is not None:
            self._on_render()
        snap = self.builder.build(clock=self._make_clock())
        if self._cameras:
            snap = self._attach_cameras(snap)
        return snap

    def reset(self, seed=None):
        """Reset MuJoCo state and return initial snapshot.

        *seed* is reserved for future domain randomization.  MuJoCo
        itself is deterministic, so the same model always resets to
        the same state regardless of seed.
        """
        with self.world.lock:
            self.world.reset()
        self._tick = 0
        if self._on_render is not None:
            self._on_render()
        snap = self.builder.build(clock=self._make_clock())
        if self._cameras:
            snap = self._attach_cameras(snap)
        return snap

    def _attach_cameras(self, snap):
        """Render cameras and attach as SensorSample entries."""
        from ..world._proto import world_pb
        frames = self.render_cameras()
        sensors = list(snap.sensors) if snap.sensors else []
        for cam_name, pixels in frames.items():
            sensors.append(
                world_pb.SensorSample(
                    ref=world_pb.SensorRef(robot_id="", sensor_id=cam_name),
                    camera_frame=world_pb.CameraFrame(
                        width=pixels.shape[1],
                        height=pixels.shape[0],
                        encoding="rgb8",
                        data=pixels.tobytes(),
                        capture_tick=self._tick,
                    ),
                )
            )
        snap.sensors = sensors
        return snap

    def run_forever(self) -> None:
        """Not used — stepping scheduler is driven by IPC requests."""
        raise NotImplementedError(
            "SteppingScheduler is request-driven; use step()/reset() instead"
        )

    def stop(self) -> None:
        """Clean up renderers."""
        for r in self._renderers.values():
            try:
                r.close()
            except Exception:
                pass
        self._renderers.clear()

    # ── Camera rendering ──

    def _init_renderers(self) -> None:
        import mujoco
        for cam_name, cfg in self._cameras.items():
            renderer = mujoco.Renderer(
                self.world.model, height=cfg.height, width=cfg.width
            )
            self._renderers[cam_name] = renderer
            _log.info(
                "camera initialized",
                extra={"extra_fields": {"name": cam_name, "res": f"{cfg.width}x{cfg.height}"}},
            )

    def render_cameras(self) -> dict[str, np.ndarray]:
        """Render all configured cameras, return {name: (H, W, 3) uint8}."""
        import mujoco
        frames: dict[str, np.ndarray] = {}
        for cam_name, cfg in self._cameras.items():
            renderer = self._renderers.get(cam_name)
            if renderer is None:
                continue
            # Prefer MJCF-defined camera by name; fall back to free camera
            mjcf_cam_id = mujoco.mj_name2id(
                self.world.model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name
            )
            if mjcf_cam_id >= 0:
                renderer.update_scene(self.world.data, camera=cam_name)
            else:
                cam = mujoco.MjvCamera()
                cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                cam.lookat[:] = cfg.lookat
                cam.distance = cfg.distance
                cam.azimuth = cfg.azimuth
                cam.elevation = cfg.elevation
                renderer.update_scene(self.world.data, camera=cam)
            frames[cam_name] = renderer.render().copy()
        return frames
