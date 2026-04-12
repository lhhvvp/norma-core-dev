"""`python -m norma_sim --manifest <path>` entry point.

The `--manifest` flag accepts the MVP-2 `.scene.yaml` schema
(see docs/superpowers/specs/2026-04-11-mvp2-menagerie-walking-skeleton-design.md
section 8.1). MVP-1's `.world.yaml` schema is no longer supported.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os
import signal
import sys
import threading
from pathlib import Path
from typing import Optional

from .ipc.codec import WorldClock
from .ipc.server import IpcServer
from .logging_setup import configure_logging
from .scheduler.realtime import RealTimeScheduler
from .scheduler.stepping import SteppingScheduler
from .world.actuation import ActuationApplier
from .world.descriptor import build_world_descriptor
from .world.manifest import load_manifest
from .world.model import MuJoCoWorld
from .world.snapshot import SnapshotBuilder


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog="norma_sim")
    ap.add_argument(
        "--manifest",
        type=Path,
        required=True,
        help=(
            "Path to the sim scene config yaml (MVP-2 .scene.yaml schema; "
            "see spec 2026-04-11-mvp2-menagerie-walking-skeleton-design.md "
            "section 8.1)."
        ),
    )
    ap.add_argument(
        "--socket",
        type=Path,
        default=None,
        help="UDS bind path (defaults to $NORMA_SIM_SOCKET_PATH)",
    )
    ap.add_argument("--physics-hz", type=int, default=500)
    ap.add_argument("--publish-hz", type=int, default=100)
    ap.add_argument(
        "--mode",
        choices=["realtime", "stepping"],
        default="realtime",
        help="Scheduler mode: 'realtime' (wall-clock paced, default) or "
             "'stepping' (step-on-demand for Gymnasium integration)",
    )
    ap.add_argument(
        "--render-port",
        type=int,
        default=0,
        help="If set, start mjviser web viewer on this port (e.g. 8012). "
             "Only effective in stepping mode.",
    )
    ap.add_argument(
        "--cameras",
        nargs="*",
        default=None,
        help="Camera names to render (e.g. 'top wrist.top'). "
             "Uses built-in presets. Only effective in stepping mode.",
    )
    ap.add_argument("--log-level", default="INFO")
    return ap.parse_args(argv)


async def _async_main(args: argparse.Namespace) -> int:
    log = logging.getLogger("norma_sim.cli")

    socket_path: Optional[Path] = args.socket
    if socket_path is None:
        env = os.environ.get("NORMA_SIM_SOCKET_PATH")
        if not env:
            log.critical("no --socket and no $NORMA_SIM_SOCKET_PATH set")
            return 1
        socket_path = Path(env)
    assert socket_path is not None

    manifest = load_manifest(args.manifest)
    log.info(
        "manifest loaded",
        extra={
            "extra_fields": {
                "world": manifest.world_name,
                "robots": len(manifest.robots),
                "mjcf": str(manifest.mjcf_path),
            }
        },
    )

    world = MuJoCoWorld(manifest)
    descriptor = build_world_descriptor(
        manifest, world=world, publish_hz=args.publish_hz, physics_hz=args.physics_hz
    )
    applier = ActuationApplier(world)
    builder = SnapshotBuilder(world)

    loop = asyncio.get_running_loop()

    def on_actuation(batch) -> None:
        applier.drain_and_apply(batch)

    # ── Stepping mode: IPC-driven, no background thread ──
    if args.mode == "stepping":
        # Optional mjviser web viewer
        on_render = None
        if args.render_port > 0:
            try:
                import viser
                from mjviser import ViserMujocoScene
                viser_server = viser.ViserServer(port=args.render_port)
                mjv_scene = ViserMujocoScene(viser_server, world.model, num_envs=1)
                log.info(
                    "mjviser started",
                    extra={"extra_fields": {"port": args.render_port}},
                )

                _gui_cam_handles: dict[str, object] = {}

                def on_render() -> None:
                    mjv_scene.update_from_mjdata(world.data)
                    # Update camera feed panels in GUI sidebar
                    if stepping and stepping._cameras and viser_server:
                        frames = stepping.render_cameras()
                        for cam_name, pixels in frames.items():
                            if cam_name not in _gui_cam_handles:
                                _gui_cam_handles[cam_name] = viser_server.gui.add_image(
                                    pixels, label=f"cam: {cam_name}",
                                    format="jpeg", jpeg_quality=80,
                                )
                            else:
                                _gui_cam_handles[cam_name].image = pixels

            except ImportError:
                viser_server = None
                _gui_cam_handles = {}
                log.warning("mjviser not installed; --render-port ignored")

        # Camera config
        cam_configs = None
        if args.cameras:
            from .scheduler.stepping import DEFAULT_CAMERAS
            cam_configs = {}
            for name in args.cameras:
                if name in DEFAULT_CAMERAS:
                    cam_configs[name] = DEFAULT_CAMERAS[name]
                else:
                    log.warning(f"unknown camera preset '{name}', skipping")

        stepping = SteppingScheduler(
            world, applier=applier, builder=builder,
            physics_hz=args.physics_hz, on_render=on_render,
            cameras=cam_configs,
        )
        server = IpcServer(
            socket_path=socket_path,
            manifest=manifest,
            descriptor=descriptor,
            on_actuation=on_actuation,
            on_step=stepping.step,
            on_reset=stepping.reset,
        )
    else:
        server = IpcServer(
            socket_path=socket_path,
            manifest=manifest,
            descriptor=descriptor,
            on_actuation=on_actuation,
        )

    # Install the asyncio signal handlers BEFORE binding the UDS so
    # that a SIGTERM arriving between `server.start()` and the
    # `_request_stop_event.wait()` below cannot kill the process
    # before `server.stop()` has a chance to unlink the socket. The
    # physics thread hasn't been spawned yet either; the handler
    # safely sets a flag and the cleanup path runs on the main loop.
    stopping = threading.Event()
    loop_stopped = threading.Event()
    _request_stop_event = asyncio.Event()

    def _handle_signal(*_: object) -> None:
        log.info("received shutdown signal")
        stopping.set()
        try:
            loop.call_soon_threadsafe(_request_stop_event.set)
        except RuntimeError:
            pass

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, _handle_signal)
            except NotImplementedError:
                # Windows — fallback to default handler
                pass
    except RuntimeError:
        pass

    await server.start()

    # ── Realtime mode: background physics thread ──
    if args.mode == "realtime":
        # Optional mjviser for realtime mode.
        # IMPORTANT: ViserMujocoScene init takes 3-5s and would block the
        # asyncio event loop, causing Station's IPC handshake to timeout.
        # We defer it to a background thread so handshake completes first.
        mjv_scene_holder = [None]  # mutable holder for closure
        if args.render_port > 0:
            try:
                import viser
                from mjviser import ViserMujocoScene

                _rt_viser_server = [None]
                _rt_gui_handles: dict[str, object] = {}
                _rt_renderers: dict[str, object] = {}
                _rt_cam_names = list(args.cameras) if args.cameras else []

                def _init_mjviser():
                    vs = viser.ViserServer(port=args.render_port)
                    scene = ViserMujocoScene(vs, world.model, num_envs=1)
                    mjv_scene_holder[0] = scene
                    _rt_viser_server[0] = vs
                    log.info(
                        "mjviser started (realtime)",
                        extra={"extra_fields": {"port": args.render_port}},
                    )

                threading.Thread(target=_init_mjviser, name="mjviser-init", daemon=True).start()
            except ImportError:
                log.warning("mjviser not installed; --render-port ignored")

        def publish_cb(tick: int) -> None:
            clock = WorldClock(
                world_tick=tick,
                sim_time_ns=int(tick * (1e9 / args.physics_hz)),
                wall_time_ns=0,
            )
            snap = builder.build(clock=clock)
            try:
                loop.call_soon_threadsafe(server.broadcast_snapshot, snap)
            except RuntimeError:
                pass
            # Push to mjviser (same thread as physics, no lock needed)
            if mjv_scene_holder[0] is not None:
                mjv_scene_holder[0].update_from_mjdata(world.data)
                # Render camera feeds to GUI sidebar at ~10Hz (not every publish)
                vs = _rt_viser_server[0]
                if vs is not None and _rt_cam_names and tick % 10 == 0:
                    import mujoco as _mj
                    # Lazy init renderers on first call (from physics thread — no race)
                    if not _rt_renderers:
                        from .scheduler.stepping import DEFAULT_CAMERAS
                        for name in _rt_cam_names:
                            if name in DEFAULT_CAMERAS:
                                cfg = DEFAULT_CAMERAS[name]
                                _rt_renderers[name] = {
                                    "renderer": _mj.Renderer(world.model, height=cfg.height, width=cfg.width),
                                    "cfg": cfg,
                                }
                    for cam_name, r in _rt_renderers.items():
                        mjcf_cam_id = _mj.mj_name2id(
                            world.model, _mj.mjtObj.mjOBJ_CAMERA, cam_name
                        )
                        if mjcf_cam_id >= 0:
                            r["renderer"].update_scene(world.data, camera=cam_name)
                        else:
                            cam = _mj.MjvCamera()
                            cam.type = _mj.mjtCamera.mjCAMERA_FREE
                            cam.lookat[:] = r["cfg"].lookat
                            cam.distance = r["cfg"].distance
                            cam.azimuth = r["cfg"].azimuth
                            cam.elevation = r["cfg"].elevation
                            r["renderer"].update_scene(world.data, camera=cam)
                        pixels = r["renderer"].render()
                        if cam_name not in _rt_gui_handles:
                            _rt_gui_handles[cam_name] = vs.gui.add_image(
                                pixels, label=f"cam: {cam_name}",
                                format="jpeg", jpeg_quality=80,
                            )
                        else:
                            _rt_gui_handles[cam_name].image = pixels

        scheduler = RealTimeScheduler(
            world,
            physics_hz=args.physics_hz,
            publish_hz=args.publish_hz,
            on_publish=publish_cb,
        )

        def physics_thread() -> None:
            try:
                scheduler.run_forever()
            finally:
                loop_stopped.set()
                try:
                    loop.call_soon_threadsafe(_request_stop_event.set)
                except RuntimeError:
                    pass

        t = threading.Thread(target=physics_thread, name="sim-physics", daemon=True)
        t.start()
    else:
        scheduler = None  # type: ignore[assignment]
        t = None
        log.info("stepping mode: physics driven by IPC requests")

    try:
        await _request_stop_event.wait()
    finally:
        if scheduler is not None:
            scheduler.stop()
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        await server.stop()

    log.info("norma_sim shut down cleanly")
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)
    configure_logging(args.log_level)
    try:
        return asyncio.run(_async_main(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
