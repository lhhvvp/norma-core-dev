"""Smoke test for scene.xml — Menagerie-style wrapper around elrobot_follower.xml.

This test loads the scene wrapper via mujoco.MjModel.from_xml_path and verifies
name-based invariants: actuator count must match the included main MJCF, the
floor geom must exist by name, and the directional light must exist by name.

Per spec Section 4 risk #3 / codex iter-1 reframe: this test uses
mj_name2id-based assertions, NOT count-based assertions like `m.ngeom == N`.
Count-based assertions are fragile to future geometry additions; name-based
assertions are stable across structural changes that don't affect the named
elements.
"""
from __future__ import annotations

from pathlib import Path

import mujoco
import pytest


@pytest.fixture
def scene_xml_path() -> Path:
    """Path to scene.xml inside this package."""
    p = Path(__file__).resolve().parent.parent / "scene.xml"
    assert p.exists(), (
        f"scene.xml not found at {p}. scene.xml is mandatory content of "
        f"this package after MVP-3 Chunk 2."
    )
    return p


def test_scene_xml_compiles_and_includes_main_mjcf(scene_xml_path: Path):
    """scene.xml must compile via from_xml_path and the <include> namespace
    merge must bring in the main MJCF's actuators (nu == 8)."""
    m = mujoco.MjModel.from_xml_path(str(scene_xml_path))
    assert m.nu == 8, (
        f"scene.xml's <include> should bring in the main MJCF's 8 actuators, "
        f"got nu={m.nu}. Check that <include file=\"elrobot_follower.xml\"/> "
        f"is present and the include path resolves correctly."
    )


def test_scene_xml_floor_geom_exists_by_name(scene_xml_path: Path):
    """The floor geom added by scene.xml must be discoverable by name. This
    is the name-based equivalent of asserting on geom count, which would be
    fragile to future geometry additions."""
    m = mujoco.MjModel.from_xml_path(str(scene_xml_path))
    floor_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    assert floor_id >= 0, (
        f"scene.xml should declare a <geom name=\"floor\"> in its <worldbody>, "
        f"got mj_name2id result {floor_id}. Check that the floor geom is "
        f"present and named 'floor'."
    )


def test_scene_xml_floor_light_exists_by_name(scene_xml_path: Path):
    """The directional light added by scene.xml must be discoverable by name."""
    m = mujoco.MjModel.from_xml_path(str(scene_xml_path))
    light_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, "floor_light")
    assert light_id >= 0, (
        f"scene.xml should declare a <light name=\"floor_light\"> in its "
        f"<worldbody>, got mj_name2id result {light_id}. Check that the "
        f"directional light is present and named 'floor_light'."
    )
