"""Shared pytest fixtures for norma_sim tests (MVP-2 layout)."""
from pathlib import Path

import pytest


@pytest.fixture
def repo_root() -> Path:
    # tests/conftest.py → sim-server/ → software/ → repo root
    return Path(__file__).resolve().parents[3]


# --- Menagerie fixtures: immediately available after Chunk 1 ---

@pytest.fixture
def menagerie_mjcf_path(repo_root: Path) -> Path:
    """Path to the vendored Menagerie trs_so_arm100 scene.xml.
    Chunk 1 dependency — skipped if the vendor directory is absent."""
    p = repo_root / "hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml"
    if not p.exists():
        pytest.skip(f"Menagerie vendor not found at {p}; run Chunk 1 first")
    return p


@pytest.fixture
def menagerie_scene_yaml(tmp_path: Path, menagerie_mjcf_path: Path) -> Path:
    """Minimal scene.yaml pointing at the Menagerie MJCF, generated in tmp_path.
    Tests needing annotations should write their own yaml instead."""
    scene_yaml = tmp_path / "menagerie.scene.yaml"
    scene_yaml.write_text(
        f"world_name: menagerie_test\n"
        f"mjcf_path: {menagerie_mjcf_path}\n"
    )
    return scene_yaml


# --- ElRobot fixtures: skipped until Chunk 5 lands the hand-written MJCF ---

@pytest.fixture
def elrobot_mjcf_path(repo_root: Path) -> Path:
    """Path to the hand-written ElRobot MJCF (Chunk 5 artifact).
    Skipped during Chunks 2-4."""
    p = repo_root / "hardware/elrobot/simulation/elrobot_follower.xml"
    if not p.exists():
        pytest.skip(f"ElRobot MJCF not found at {p}; run Chunk 5 first")
    return p


@pytest.fixture
def elrobot_scene_yaml(repo_root: Path) -> Path:
    """Path to the hand-written ElRobot scene.yaml (Chunk 5 artifact).
    Skipped during Chunks 2-4."""
    p = repo_root / "hardware/elrobot/simulation/elrobot_follower.scene.yaml"
    if not p.exists():
        pytest.skip(f"ElRobot scene.yaml not found at {p}; run Chunk 5 first")
    return p
