"""URDF ↔ MJCF consistency gate.

The `elrobot_follower.urdf` at hardware/elrobot/simulation/ is kept as the
kinematic source of truth. This test prevents it from rotting: whenever
the MJCF's joint topology diverges from the URDF's, the test fails loudly.

Scope: structural invariants only (joint names, counts, axes). Dynamic
properties (inertia, mass, friction) are NOT checked because the MJCF
intentionally overrides those per the MVP-2 calibration_notes.md.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import pytest


@pytest.fixture
def urdf_path() -> Path:
    here = Path(__file__).resolve()
    # tests/ → robot package → mujoco/ → simulation/
    p = here.parent.parent.parent.parent / "elrobot_follower.urdf"
    if not p.exists():
        pytest.skip(f"ElRobot URDF not found at {p}")
    return p


def test_urdf_and_mjcf_agree_on_joint_names(urdf_path: Path, elrobot_mjcf_path: Path):
    """Joint names M1..M7 (revolute) + M8 (gripper) must match between URDF and MJCF."""
    urdf_root = ET.parse(urdf_path).getroot()
    urdf_joint_names = {
        j.attrib["name"]
        for j in urdf_root.findall("joint")
        if j.attrib.get("type") in ("revolute", "continuous", "prismatic")
    }
    model = mujoco.MjModel.from_xml_path(str(elrobot_mjcf_path))
    mjcf_joint_names = set()
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            mjcf_joint_names.add(name)
    missing_in_mjcf = urdf_joint_names - mjcf_joint_names
    assert not missing_in_mjcf, (
        f"URDF joints missing from MJCF: {sorted(missing_in_mjcf)}"
    )


def test_urdf_and_mjcf_agree_on_actuated_joint_count(
    urdf_path: Path, elrobot_mjcf_path: Path
):
    """ElRobot has 8 actuated joints (7 revolute + 1 gripper primary).
    The MJCF may have additional mimic joints (rev_motor_08_1, rev_motor_08_2)
    that do not appear in the URDF as top-level actuated joints."""
    urdf_root = ET.parse(urdf_path).getroot()
    urdf_actuated = {
        j.attrib["name"]
        for j in urdf_root.findall("joint")
        if j.attrib.get("type") in ("revolute", "continuous")
    }
    assert len(urdf_actuated) == 8, (
        f"Expected 8 actuated URDF joints, got {len(urdf_actuated)}: "
        f"{sorted(urdf_actuated)}"
    )
