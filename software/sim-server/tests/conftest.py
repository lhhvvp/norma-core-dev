"""Shared pytest fixtures for norma_sim tests."""
from pathlib import Path

import pytest


@pytest.fixture
def repo_root() -> Path:
    # tests/conftest.py → sim-server/ → software/ → repo root
    return Path(__file__).resolve().parents[3]


@pytest.fixture
def world_yaml_path(repo_root: Path) -> Path:
    return repo_root / "hardware/elrobot/simulation/worlds/elrobot_follower.world.yaml"


@pytest.fixture
def mjcf_path(repo_root: Path) -> Path:
    return repo_root / "hardware/elrobot/simulation/worlds/elrobot_follower.xml"
