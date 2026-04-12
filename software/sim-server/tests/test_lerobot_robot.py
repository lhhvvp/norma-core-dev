"""Tests for the LeRobot Robot adapter."""
import pytest

try:
    from norma_sim.lerobot_robot import NormaSimRobot, NormaSimRobotConfig
    _OK = True
    _ERR = ""
except Exception as e:
    _OK = False
    _ERR = str(e)

pytestmark = pytest.mark.skipif(not _OK, reason=f"lerobot_robot imports failed: {_ERR}")


def _get_manifest():
    from pathlib import Path
    repo = Path(__file__).resolve().parents[3]
    candidates = [
        repo / "hardware/elrobot/simulation/manifests/norma/therobotstudio_so101.scene.yaml",
        repo / "hardware/elrobot/simulation/manifests/norma/menagerie_so_arm100.scene.yaml",
    ]
    for c in candidates:
        if c.exists():
            return str(c)
    pytest.skip("No scene.yaml found")


def test_observation_features():
    config = NormaSimRobotConfig(manifest_path=_get_manifest())
    robot = NormaSimRobot(config)
    features = robot.observation_features
    assert "shoulder_pan.pos" in features
    assert "gripper.pos" in features
    assert len(features) == 6  # 5 joints + 1 gripper


def test_action_features():
    config = NormaSimRobotConfig(manifest_path=_get_manifest())
    robot = NormaSimRobot(config)
    features = robot.action_features
    assert set(features.keys()) == set(robot.observation_features.keys())


def test_connect_disconnect():
    config = NormaSimRobotConfig(manifest_path=_get_manifest())
    robot = NormaSimRobot(config)
    assert not robot.is_connected
    robot.connect()
    assert robot.is_connected
    robot.disconnect()
    assert not robot.is_connected


def test_get_observation():
    config = NormaSimRobotConfig(manifest_path=_get_manifest())
    with NormaSimRobot(config) as robot:
        obs = robot.get_observation()
        assert "shoulder_pan.pos" in obs
        assert "gripper.pos" in obs
        assert isinstance(obs["shoulder_pan.pos"], float)


def test_send_action_and_observe():
    config = NormaSimRobotConfig(manifest_path=_get_manifest())
    with NormaSimRobot(config) as robot:
        obs_before = robot.get_observation()

        # Send non-zero action
        action = {f"{name}.pos": 0.5 for name in robot.JOINT_NAMES}
        action["gripper.pos"] = 50.0  # LeRobot 0-100 scale
        sent = robot.send_action(action)
        assert sent == action

        # After stepping, obs should change
        obs_after = robot.get_observation()
        # At least one joint should have moved
        diffs = [
            abs(obs_after[f"{n}.pos"] - obs_before[f"{n}.pos"])
            for n in robot.JOINT_NAMES
        ]
        assert max(diffs) > 0.001, f"No joint moved: {diffs}"


def test_context_manager():
    config = NormaSimRobotConfig(manifest_path=_get_manifest())
    with NormaSimRobot(config) as robot:
        assert robot.is_connected
        robot.send_action({
            "shoulder_pan.pos": 0.3,
            "shoulder_lift.pos": 0.0,
            "elbow_flex.pos": 0.0,
            "wrist_flex.pos": 0.0,
            "wrist_roll.pos": 0.0,
            "gripper.pos": 50.0,
        })
    assert not robot.is_connected


def test_multiple_steps():
    config = NormaSimRobotConfig(manifest_path=_get_manifest())
    with NormaSimRobot(config) as robot:
        for i in range(10):
            action = {f"{n}.pos": 0.1 * i for n in robot.JOINT_NAMES}
            action["gripper.pos"] = 50.0
            robot.send_action(action)
            obs = robot.get_observation()
            assert "shoulder_pan.pos" in obs
