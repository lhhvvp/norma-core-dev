"""Tiled grid visualization for parallel simulation states.

Renders N environment states into a single tiled image for debugging
and monitoring parallel training/data generation.

Usage::

    from norma_sim.viz_grid import render_grid, save_grid_video

    # Snapshot: render current states as a grid image
    canvas = render_grid(mj_model, batch_qpos_numpy, grid_cols=8, cam_size=128)
    # canvas.shape = (rows*128, 8*128, 3)

    # Video: accumulate frames over training
    frames = []
    for step in range(1000):
        batch_data = mjx_step(batch_data)
        if step % 10 == 0:
            qpos_np = jax.device_get(batch_data.qpos)
            frames.append(render_grid(mj_model, qpos_np))
    save_grid_video(frames, "training_grid.mp4", fps=10)
"""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np


def render_grid(
    mj_model: mujoco.MjModel,
    batch_qpos: np.ndarray,
    grid_cols: int = 8,
    cam_size: int = 128,
    camera_name: str | None = None,
    lookat: tuple[float, float, float] = (0.0, 0.05, 0.1),
    distance: float = 0.6,
    azimuth: float = 90.0,
    elevation: float = -60.0,
) -> np.ndarray:
    """Render N environment states as a tiled grid image.

    Args:
        mj_model: MuJoCo model (CPU).
        batch_qpos: (N, nq) array of joint positions for each environment.
        grid_cols: Number of columns in the grid.
        cam_size: Width and height of each cell (square).
        camera_name: MJCF camera name (None = free camera with lookat params).

    Returns:
        (H, W, 3) uint8 canvas with all environments tiled.
    """
    n_envs = batch_qpos.shape[0]
    grid_rows = (n_envs + grid_cols - 1) // grid_cols

    renderer = mujoco.Renderer(mj_model, height=cam_size, width=cam_size)
    mj_data = mujoco.MjData(mj_model)
    canvas = np.zeros((grid_rows * cam_size, grid_cols * cam_size, 3), dtype=np.uint8)

    # Set up camera
    if camera_name:
        cam_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        use_named = cam_id >= 0
    else:
        use_named = False

    if not use_named:
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = lookat
        cam.distance = distance
        cam.azimuth = azimuth
        cam.elevation = elevation

    for i in range(n_envs):
        nq = min(batch_qpos.shape[1], mj_model.nq)
        mj_data.qpos[:nq] = batch_qpos[i, :nq]
        mujoco.mj_forward(mj_model, mj_data)

        if use_named:
            renderer.update_scene(mj_data, camera=camera_name)
        else:
            renderer.update_scene(mj_data, camera=cam)

        frame = renderer.render()
        row, col = divmod(i, grid_cols)
        canvas[row * cam_size:(row + 1) * cam_size,
               col * cam_size:(col + 1) * cam_size] = frame

    renderer.close()
    return canvas


def save_grid_video(
    frames: list[np.ndarray],
    path: str | Path,
    fps: int = 10,
) -> None:
    """Save a list of grid frames as MP4 video.

    Args:
        frames: List of (H, W, 3) uint8 arrays from render_grid().
        path: Output video path.
        fps: Frames per second.
    """
    import imageio.v3 as iio

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(str(path), np.stack(frames), fps=fps, codec="h264")
