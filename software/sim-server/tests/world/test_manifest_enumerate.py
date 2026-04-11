"""Tests for manifest._enumerate_mjcf_actuators — the MJCF → actuator list
helper used by the MVP-2 scene.yaml loader. Tests run against Menagerie's
vendored trs_so_arm100 MJCF (Chunk 1 dependency)."""
from pathlib import Path

import pytest

from norma_sim.world.manifest import _enumerate_mjcf_actuators


@pytest.fixture
def menagerie_scene_xml() -> Path:
    """Locate the Chunk 1 vendored Menagerie MJCF without relying on
    conftest.py (which is migrated in Chunk 3)."""
    here = Path(__file__).resolve()
    # parents: [0]=tests/world, [1]=tests, [2]=sim-server, [3]=software, [4]=repo
    repo_root = here.parents[4]
    p = repo_root / "hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml"
    if not p.exists():
        pytest.skip(f"Menagerie vendor not found at {p}; Chunk 1 has not been run yet")
    return p


def test_enumerate_menagerie_returns_nonempty_list(menagerie_scene_xml: Path):
    actuators = _enumerate_mjcf_actuators(menagerie_scene_xml)
    assert len(actuators) >= 5, (
        f"Menagerie trs_so_arm100 should have >= 5 actuators, got {len(actuators)}"
    )


def test_enumerate_returns_three_tuple_name_joint_type(menagerie_scene_xml: Path):
    actuators = _enumerate_mjcf_actuators(menagerie_scene_xml)
    for entry in actuators:
        assert len(entry) == 3, f"expected 3-tuple, got {entry}"
        name, joint, type_tag = entry
        assert isinstance(name, str) and name
        assert isinstance(joint, str) and joint
        assert type_tag in ("position", "motor", "general", "velocity"), (
            f"unexpected actuator type: {type_tag}"
        )


def test_enumerate_joint_name_resolves_in_mjcf(menagerie_scene_xml: Path):
    """Verify each returned joint name actually exists in the MJCF
    (so MuJoCoWorld._build_lookups won't fail when it constructs
    from a synthesized ActuatorManifest)."""
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(menagerie_scene_xml))
    actuators = _enumerate_mjcf_actuators(menagerie_scene_xml)
    for name, joint, _ in actuators:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        assert joint_id >= 0, (
            f"enumerator returned joint '{joint}' for actuator '{name}' "
            f"but MJCF has no such joint"
        )


def test_enumerate_raises_on_nonexistent_file(tmp_path: Path):
    with pytest.raises((FileNotFoundError, ValueError)):
        _enumerate_mjcf_actuators(tmp_path / "does_not_exist.xml")
