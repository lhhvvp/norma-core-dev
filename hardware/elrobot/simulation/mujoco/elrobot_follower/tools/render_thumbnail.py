#!/usr/bin/env python3
"""Render elrobot_follower.png thumbnail for Menagerie-convention packaging.

Usage (from any directory):
    python3 tools/render_thumbnail.py

Outputs elrobot_follower.png in the package root directory.
Requires: mujoco >= 3.0, Pillow.
"""
from pathlib import Path
import sys

try:
    import mujoco
except ImportError:
    sys.exit("mujoco not installed. Install with: pip install mujoco")

try:
    from PIL import Image
except ImportError:
    sys.exit("Pillow not installed. Install with: pip install Pillow")

# Resolve paths relative to this script's location (tools/ -> package root)
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
SCENE_XML = PACKAGE_ROOT / "scene.xml"
OUTPUT_PNG = PACKAGE_ROOT / "elrobot_follower.png"

# Image dimensions (standard Menagerie thumbnail size)
WIDTH = 640
HEIGHT = 480

# Camera: elevated front-left view, arm centered in frame at home pose.
# Tuned for the ElRobot follower arm with floor visible.
CAMERA_AZIMUTH = -120.0
CAMERA_ELEVATION = -25.0
CAMERA_DISTANCE = 1.0
CAMERA_LOOKAT = (0.0, 0.0, 0.2)


def main():
    if not SCENE_XML.exists():
        sys.exit(f"scene.xml not found at {SCENE_XML}")

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    cam = mujoco.MjvCamera()
    cam.azimuth = CAMERA_AZIMUTH
    cam.elevation = CAMERA_ELEVATION
    cam.distance = CAMERA_DISTANCE
    cam.lookat[:] = CAMERA_LOOKAT

    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        renderer.update_scene(data, camera=cam)
        pixels = renderer.render()

    img = Image.fromarray(pixels)
    img.save(str(OUTPUT_PNG))
    print(f"Saved {OUTPUT_PNG} ({WIDTH}x{HEIGHT})")


if __name__ == "__main__":
    main()
