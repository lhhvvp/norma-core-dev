"""MJX forward-pass smoke gate (placeholder).

This test reserves a CI slot for verifying the MJCF compiles under
MuJoCo's JAX backend (`mujoco.mjx`). MJX is the GPU-accelerated / batched
rollout / differentiable sim path that matters for policy training
(RL, IL, domain randomization).

Status: placeholder. If `mujoco.mjx` is importable, run a minimal
`mjx.put_model` + `mjx.forward` pass. Otherwise skip.

When MVP-3 moves into policy training, this test expands into a real
MJX compatibility gate with full forward+backward pass verification.
"""
import pytest


def test_mjx_forward_pass_compiles(elrobot_mjcf_path):
    """Minimal smoke test: MJX must be able to compile this MJCF and
    run a single forward pass without errors."""
    mjx = pytest.importorskip("mujoco.mjx")
    import mujoco

    mj_model = mujoco.MjModel.from_xml_path(str(elrobot_mjcf_path))
    mjx_model = mjx.put_model(mj_model)
    mjx_data = mjx.make_data(mjx_model)
    mjx_data = mjx.forward(mjx_model, mjx_data)
    # Verify basic invariants post-forward:
    assert mjx_model.nu == 8, (
        f"expected nu=8 in MJX model, got {mjx_model.nu}"
    )
    assert mjx_model.nv == 10, (
        f"expected nv=10 in MJX model, got {mjx_model.nv}"
    )
