"""★ P0: pytest version of the Chunk 1 Task 1.7 MJCF demo.

This test is the canary for the whole capability-keyed gripper
story. If the tendon-based equality in `gen.py` regresses (e.g.
someone reverts to joint-joint polycoef), this test will fail long
before Chunk 7's bridge reports anything wrong.
"""
import mujoco


def test_mimic_gripper_equality_works(elrobot_mjcf_path):
    """Driving rev_motor_08 to 1.0 rad should move both mimic joints
    by approximately ±0.0115 (in metres — the mimic joints are
    prismatic and `0.0115 m/rad` is the ElRobot mimic multiplier).
    """
    model = mujoco.MjModel.from_xml_path(str(elrobot_mjcf_path))
    data = mujoco.MjData(model)

    act8 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_motor_08")
    j08 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rev_motor_08")
    j08_1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rev_motor_08_1")
    j08_2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rev_motor_08_2")

    assert act8 >= 0, "act_motor_08 not found"
    assert j08 >= 0 and j08_1 >= 0 and j08_2 >= 0

    data.ctrl[act8] = 1.0
    # The gripper actuator uses kp=10 which settles slowly; 5000
    # steps (10 s sim @ 500 Hz) is the empirically measured window
    # in Chunk 1 — plan's 500 is too short. See commit a76b2fe.
    for _ in range(5000):
        mujoco.mj_step(model, data)

    q08 = data.qpos[model.jnt_qposadr[j08]]
    q08_1 = data.qpos[model.jnt_qposadr[j08_1]]
    q08_2 = data.qpos[model.jnt_qposadr[j08_2]]

    # Motor 8 itself reaches (close to) the 1.0 rad target.
    assert abs(q08 - 1.0) < 0.1, f"motor 8 qpos={q08}"

    # Mimic joints track the ratio 0.0115 m/rad relative to motor 8's
    # actual position. Expected at q08 ≈ 0.94: ±(0.94 × 0.0115) ≈
    # ±0.0108 m. Tolerance 0.002 covers both the kp settling delta
    # and the solver's finite stiffness on the equality tendon.
    expected_1 = -0.0115 * q08
    expected_2 = 0.0115 * q08
    assert abs(q08_1 - expected_1) < 0.002, (
        f"rev_motor_08_1 qpos={q08_1}, expected ~{expected_1}"
    )
    assert abs(q08_2 - expected_2) < 0.002, (
        f"rev_motor_08_2 qpos={q08_2}, expected ~{expected_2}"
    )


def test_mimic_gripper_zero_setpoint_holds_zero(elrobot_mjcf_path):
    """Sanity: with ctrl=0 the mimic joints stay near 0."""
    model = mujoco.MjModel.from_xml_path(str(elrobot_mjcf_path))
    data = mujoco.MjData(model)

    act8 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_motor_08")
    j08_1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rev_motor_08_1")
    j08_2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rev_motor_08_2")

    data.ctrl[act8] = 0.0
    for _ in range(500):
        mujoco.mj_step(model, data)

    q08_1 = data.qpos[model.jnt_qposadr[j08_1]]
    q08_2 = data.qpos[model.jnt_qposadr[j08_2]]
    # Small tolerance because gravity may nudge the finger.
    assert abs(q08_1) < 0.002
    assert abs(q08_2) < 0.002
