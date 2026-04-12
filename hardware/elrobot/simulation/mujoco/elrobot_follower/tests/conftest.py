"""Shared pytest fixtures for the ElRobot follower MuJoCo package.

These fixtures are deliberately minimal. The tests in this directory
exercise the MJCF via direct `mujoco.MjModel.from_xml_path(...)` calls
and must NOT import `norma_sim` — the goal is an engine-level test suite
that can run from a fresh checkout without any NormaCore application
code on PYTHONPATH.

If you need a test that uses `norma_sim.world.MuJoCoWorld` or any other
application-layer helper, put it in `software/sim-server/tests/` instead.
"""
from pathlib import Path

import mujoco
import pytest


@pytest.fixture
def elrobot_mjcf_path() -> Path:
    """Path to the MJCF in this package."""
    p = Path(__file__).resolve().parent.parent / "elrobot_follower.xml"
    if not p.exists():
        pytest.skip(f"ElRobot MJCF not found at {p}")
    return p


@pytest.fixture
def elrobot_sim(elrobot_mjcf_path):
    """Fresh MjModel + MjData pair.  Function-scoped (default) so each
    test gets clean state — no leakage between stress test and step
    response."""
    model = mujoco.MjModel.from_xml_path(str(elrobot_mjcf_path))
    data = mujoco.MjData(model)
    return model, data
