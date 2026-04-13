"""Experiment configuration — one file drives data gen + training + eval.

An experiment.yaml describes the full pipeline:
  - Which robot (manifest)
  - Which sim backend and settings
  - Which cameras at what resolution
  - Which task with what randomization
  - Dataset output location
  - Training hyperparameters

Usage::

    from norma_sim.experiment import ExperimentConfig

    config = ExperimentConfig.load("experiments/pick_v1.yaml")
    config.robot.manifest  # path to scene.yaml
    config.sim.backend     # "cpu" or "mjx"
    config.task.name       # "pick_and_place"

Example YAML::

    robot:
      manifest: hardware/elrobot/simulation/manifests/norma/so101_tabletop.scene.yaml

    sim:
      backend: cpu
      physics_hz: 500
      action_hz: 30
      gl_env:
        DISPLAY: ":0"
        GALLIUM_DRIVER: d3d12
        MUJOCO_GL: glx

    cameras:
      top: [224, 224]

    task:
      name: pick_and_place
      episodes: 200
      action_noise: 0.02
      seed: 0

    dataset:
      repo_id: norma/sim_pick_v1
      root: datasets/norma_sim_pick_v1
      use_videos: true

    training:
      policy: act
      batch_size: 8
      steps: 50000
      save_freq: 10000
      log_freq: 100
      num_workers: 4
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class RobotConfig:
    manifest: str = ""


@dataclass
class SimToRealSection:
    """Sim-to-real degradation settings in experiment config."""
    enabled: bool = False
    preset: str = "mild"  # "off", "mild", "aggressive", or "custom"
    # Custom overrides (only used if preset="custom")
    joint_noise_std: float = 0.02
    action_delay_steps: int = 1
    calibration_offset_std: float = 0.05
    camera_latency_frames: int = 1
    image_noise_std: float = 5.0

    def to_config(self):
        """Convert to SimToRealConfig (or None if disabled)."""
        if not self.enabled:
            return None
        from .sim_to_real import SimToRealConfig
        if self.preset == "off":
            return SimToRealConfig.off()
        elif self.preset == "mild":
            return SimToRealConfig.mild()
        elif self.preset == "aggressive":
            return SimToRealConfig.aggressive()
        else:
            return SimToRealConfig(
                joint_noise_std=self.joint_noise_std,
                action_delay_steps=self.action_delay_steps,
                calibration_offset_std=self.calibration_offset_std,
                camera_latency_frames=self.camera_latency_frames,
                image_noise_std=self.image_noise_std,
            )


@dataclass
class SimConfig:
    backend: str = "fast"  # "fast" (in-process MuJoCo) or "ipc" (subprocess, for real-time/mjviser)
    physics_hz: int = 500
    action_hz: int = 30
    gl_env: dict[str, str] = field(default_factory=lambda: {
        "DISPLAY": ":0",
        "GALLIUM_DRIVER": "d3d12",
        "MUJOCO_GL": "glx",
    })
    sim_to_real: SimToRealSection = field(default_factory=SimToRealSection)


@dataclass
class TaskConfig:
    name: str = "pick_and_place"
    episodes: int = 200
    action_noise: float = 0.02
    seed: int = 0


@dataclass
class DatasetConfig:
    repo_id: str = "norma/sim_pick_v1"
    root: str = "datasets/norma_sim_pick_v1"
    use_videos: bool = True


@dataclass
class TrainingConfig:
    policy: str = "act"
    batch_size: int = 8
    steps: int = 50000
    save_freq: int = 10000
    log_freq: int = 100
    num_workers: int = 4
    output_dir: str = "outputs/act_pick_v1"


@dataclass
class ExperimentConfig:
    """Complete experiment specification."""

    robot: RobotConfig = field(default_factory=RobotConfig)
    sim: SimConfig = field(default_factory=SimConfig)
    cameras: dict[str, list[int]] = field(default_factory=lambda: {"top": [224, 224]})
    task: TaskConfig = field(default_factory=TaskConfig)
    dataset: DatasetConfig = field(default_factory=DatasetConfig)
    training: TrainingConfig = field(default_factory=TrainingConfig)

    @classmethod
    def load(cls, path: str | Path) -> "ExperimentConfig":
        """Load from YAML file."""
        with open(path) as f:
            raw = yaml.safe_load(f)
        return cls._from_dict(raw or {})

    @classmethod
    def _from_dict(cls, d: dict[str, Any]) -> "ExperimentConfig":
        sim_raw = dict(d.get("sim", {}))
        s2r_raw = sim_raw.pop("sim_to_real", {})
        sim_config = SimConfig(
            **sim_raw,
            sim_to_real=SimToRealSection(**s2r_raw) if s2r_raw else SimToRealSection(),
        )
        return cls(
            robot=RobotConfig(**d.get("robot", {})),
            sim=sim_config,
            cameras=d.get("cameras", {"top": [224, 224]}),
            task=TaskConfig(**d.get("task", {})),
            dataset=DatasetConfig(**d.get("dataset", {})),
            training=TrainingConfig(**d.get("training", {})),
        )

    def save(self, path: str | Path) -> None:
        """Save to YAML file."""
        import dataclasses
        d = {}
        for section_name in ["robot", "sim", "task", "dataset", "training"]:
            section = getattr(self, section_name)
            d[section_name] = dataclasses.asdict(section)
        d["cameras"] = self.cameras
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            yaml.dump(d, f, default_flow_style=False, sort_keys=False)

    @property
    def camera_configs(self) -> dict[str, tuple[int, int]]:
        """Cameras as {name: (height, width)} tuples."""
        return {name: tuple(hw) for name, hw in self.cameras.items()}
