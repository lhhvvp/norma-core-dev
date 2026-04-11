"""Tests for world.manifest loader."""
import pytest

from norma_sim.world.manifest import load_manifest


def test_manifest_load_happy(tmp_path, menagerie_mjcf_path):
    scene_yaml = tmp_path / "test.scene.yaml"
    scene_yaml.write_text(
        f"world_name: happy_world\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
    )
    manifest = load_manifest(scene_yaml)
    assert manifest.world_name == "happy_world"
    assert manifest.mjcf_path == menagerie_mjcf_path.resolve()
    assert len(manifest.robots) == 1
    assert len(manifest.robots[0].actuators) >= 5


def test_manifest_scene_config(tmp_path, menagerie_mjcf_path):
    scene_yaml = tmp_path / "test.scene.yaml"
    scene_yaml.write_text(
        f"world_name: test\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
        f"scene_overrides:\n"
        f"  timestep: 0.001\n"
        f"  gravity: [0, 0, -5]\n"
        f"  iterations: 100\n"
    )
    manifest = load_manifest(scene_yaml)
    assert manifest.scene.timestep == 0.001
    assert manifest.scene.gravity == (0, 0, -5)
    assert manifest.scene.iterations == 100


def test_manifest_missing_gripper_fields_raises(tmp_path, menagerie_mjcf_path):
    """GRIPPER_PARALLEL annotation missing the 'gripper:' block raises."""
    from norma_sim.world.manifest import _enumerate_mjcf_actuators
    actuators = _enumerate_mjcf_actuators(menagerie_mjcf_path)
    target_name = actuators[0][0]

    scene_yaml = tmp_path / "bad.scene.yaml"
    scene_yaml.write_text(
        f"world_name: test\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
        f"actuator_annotations:\n"
        f"  - mjcf_actuator: {target_name}\n"
        f"    actuator_id: bad_gripper\n"
        f"    display_name: Bad\n"
        f"    capability:\n"
        f"      kind: GRIPPER_PARALLEL\n"
        f"      normalized_range: [0.0, 1.0]\n"
        f"    # missing: gripper: block\n"
    )
    with pytest.raises(ValueError, match="gripper"):
        load_manifest(scene_yaml)
