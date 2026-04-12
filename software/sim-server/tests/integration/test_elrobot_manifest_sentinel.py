"""Manifest-pipeline sentinel for the ElRobot engine package.

The engine-tier acceptance tests (in mujoco/elrobot_follower/tests/) use
raw mujoco.MjModel.from_xml_path(...) and intentionally skip the manifest
layer. This sentinel exercises the full pipeline:

    scene.yaml -> load_manifest -> MuJoCoWorld -> mj_step

It catches manifest-layer regressions that the engine-tier suite cannot
see — specifically: scene.yaml parsing, world_name binding,
actuator_annotations consistency check, and the GRIPPER_PARALLEL
capability assignment for act_motor_08.
"""
from __future__ import annotations

import pytest

try:
    import mujoco
    from norma_sim.world.model import MuJoCoWorld
    _OK = True
    _ERR = ""
except Exception as e:  # pragma: no cover
    _OK = False
    _ERR = str(e)


pytestmark = pytest.mark.skipif(not _OK, reason=f"imports not OK: {_ERR}")


def test_elrobot_manifest_pipeline_sentinel(elrobot_scene_yaml):
    """Smoke: scene.yaml -> load_manifest -> MuJoCoWorld pipeline still
    works end-to-end for the elrobot package.  Catches manifest-layer
    regressions that the engine-tier acceptance suite (now in
    mujoco/elrobot_follower/tests/) cannot see — specifically:
    scene.yaml parsing, world_name binding, actuator_annotations
    consistency check, and the GRIPPER_PARALLEL capability assignment
    for act_motor_08."""
    world = MuJoCoWorld.from_manifest_path(elrobot_scene_yaml)
    mujoco.mj_step(world.model, world.data)

    # Manifest parsing actually happened (not just MJCF load)
    assert world.manifest.world_name == "elrobot_follower"

    # MJCF compile + lookup cache built correctly
    assert world.model.nu == 8

    # actuator_annotations were applied — without these explicit
    # annotations, load_manifest auto-synthesizes act_motor_08 as a
    # plain REVOLUTE_POSITION (manifest.py:187), so this assertion
    # fails if the gripper annotation is silently dropped from the
    # scene.yaml or from load_manifest's annotation merge.
    gripper = world.actuator_by_mjcf_name("act_motor_08")
    assert gripper is not None
    assert gripper.capability.kind == "GRIPPER_PARALLEL"
    assert gripper.gripper is not None
