"""Tests for the MVP-2 scene.yaml schema loader. Uses tmp_path to
build test fixtures on the fly, referencing the Menagerie vendored
MJCF for `mjcf_path`."""
from pathlib import Path

import pytest

from norma_sim.world.manifest import _enumerate_mjcf_actuators, load_manifest


@pytest.fixture
def menagerie_mjcf_path() -> Path:
    here = Path(__file__).resolve()
    # parents: [0]=tests/world, [1]=tests, [2]=sim-server, [3]=software, [4]=repo
    repo_root = here.parents[4]
    p = repo_root / "hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml"
    if not p.exists():
        pytest.skip(f"Menagerie vendor not found at {p}; run Chunk 1 first")
    return p


def _write_minimal_scene(tmp_path: Path, mjcf_path: Path) -> Path:
    scene_yaml = tmp_path / "minimal.scene.yaml"
    scene_yaml.write_text(
        f"world_name: test_world\n"
        f"mjcf_path: {mjcf_path}\n"
    )
    return scene_yaml


def test_minimal_scene_yaml_loads(tmp_path: Path, menagerie_mjcf_path: Path):
    """Simplest valid scene.yaml: world_name + mjcf_path."""
    scene_yaml = _write_minimal_scene(tmp_path, menagerie_mjcf_path)
    manifest = load_manifest(scene_yaml)
    assert manifest.world_name == "test_world"
    assert manifest.mjcf_path == menagerie_mjcf_path.resolve()
    assert len(manifest.robots) == 1
    assert len(manifest.robots[0].actuators) >= 5


def test_scene_yaml_synthesizes_revolute_actuators_with_mjcf_joint(
    tmp_path: Path, menagerie_mjcf_path: Path
):
    """Revolute <position> actuators should become REVOLUTE_POSITION
    ActuatorManifest entries with `mjcf_joint` populated from the MJCF."""
    scene_yaml = _write_minimal_scene(tmp_path, menagerie_mjcf_path)
    manifest = load_manifest(scene_yaml)
    revolute = [
        a for r in manifest.robots for a in r.actuators
        if a.capability.kind == "REVOLUTE_POSITION"
    ]
    assert len(revolute) >= 4
    for act in revolute:
        assert act.mjcf_joint, f"mjcf_joint empty on {act.actuator_id}"
        assert act.mjcf_actuator, f"mjcf_actuator empty on {act.actuator_id}"


def test_scene_yaml_annotation_overrides_capability(
    tmp_path: Path, menagerie_mjcf_path: Path
):
    """An actuator_annotation with kind=GRIPPER_PARALLEL overrides the
    default REVOLUTE_POSITION synthesis."""
    actuators = _enumerate_mjcf_actuators(menagerie_mjcf_path)
    assert len(actuators) > 0
    # Pick the last actuator as the "stand-in gripper" for test purposes
    target_mjcf_name, target_joint, _ = actuators[-1]

    scene_yaml = tmp_path / "with_annotation.scene.yaml"
    scene_yaml.write_text(
        f"world_name: test_world\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
        f"actuator_annotations:\n"
        f"  - mjcf_actuator: {target_mjcf_name}\n"
        f"    actuator_id: test_gripper\n"
        f"    display_name: Test Gripper\n"
        f"    capability:\n"
        f"      kind: GRIPPER_PARALLEL\n"
        f"      normalized_range: [0.0, 1.0]\n"
        f"    gripper:\n"
        f"      primary_joint_range_rad: [0.0, 1.0]\n"
        f"      mimic_joints: []\n"
    )
    manifest = load_manifest(scene_yaml)
    gripper_acts = [
        a for r in manifest.robots for a in r.actuators
        if a.capability.kind == "GRIPPER_PARALLEL"
    ]
    assert len(gripper_acts) == 1
    g = gripper_acts[0]
    assert g.actuator_id == "test_gripper"
    assert g.display_name == "Test Gripper"
    assert g.mjcf_joint == target_joint  # populated from MJCF via annotation
    assert g.gripper is not None
    assert g.gripper.normalized_range == (0.0, 1.0)
    assert g.gripper.primary_joint_range_rad == (0.0, 1.0)


def test_scene_yaml_missing_mjcf_path_raises(tmp_path: Path):
    scene_yaml = tmp_path / "bad.scene.yaml"
    scene_yaml.write_text("world_name: test\n")
    with pytest.raises((ValueError, KeyError)):
        load_manifest(scene_yaml)


def test_scene_yaml_annotation_for_nonexistent_actuator_raises(
    tmp_path: Path, menagerie_mjcf_path: Path
):
    scene_yaml = tmp_path / "bad_annotation.scene.yaml"
    scene_yaml.write_text(
        f"world_name: test_world\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
        f"actuator_annotations:\n"
        f"  - mjcf_actuator: actuator_that_does_not_exist\n"
        f"    actuator_id: fake\n"
        f"    display_name: Fake\n"
        f"    capability:\n"
        f"      kind: GRIPPER_PARALLEL\n"
        f"      normalized_range: [0.0, 1.0]\n"
        f"    gripper:\n"
        f"      primary_joint_range_rad: [0.0, 1.0]\n"
        f"      mimic_joints: []\n"
    )
    with pytest.raises(ValueError, match="no such actuator|not found"):
        load_manifest(scene_yaml)
