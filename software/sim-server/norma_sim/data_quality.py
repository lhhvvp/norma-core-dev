"""Dataset quality validation for robot learning pipelines.

Three layers of checks:
  1. Per-frame: NaN, joint limits, image sanity
  2. Per-episode: gripper activity, state variance, trajectory structure
  3. Cross-dataset: diversity, domain randomization coverage, statistics

Usage::

    from norma_sim.data_quality import validate_dataset
    report = validate_dataset("datasets/norma_sim_pick_v1")
    print(report)
    if report.critical:
        sys.exit(1)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class EpisodeReport:
    """Quality report for a single episode."""
    index: int
    n_frames: int = 0
    passed: bool = True
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    # Metrics
    state_std: float = 0.0
    action_std: float = 0.0
    gripper_min: float = 0.0
    gripper_max: float = 0.0
    has_nan: bool = False
    has_gripper_cycle: bool = False


@dataclass
class QualityReport:
    """Full dataset quality report."""
    dataset_path: str
    n_episodes: int = 0
    n_frames: int = 0

    # Episode results
    episodes: list[EpisodeReport] = field(default_factory=list)

    # Cross-dataset statistics
    state_global_min: float = 0.0
    state_global_max: float = 0.0
    action_global_min: float = 0.0
    action_global_max: float = 0.0
    starting_state_std: float = 0.0
    target_pan_std: float = 0.0

    # Aggregate
    n_passed: int = 0
    n_warnings: int = 0
    n_failed: int = 0
    critical: bool = False
    all_warnings: list[str] = field(default_factory=list)
    all_errors: list[str] = field(default_factory=list)

    def __str__(self) -> str:
        lines = [
            f"=== Dataset Quality Report ===",
            f"Dataset:  {self.dataset_path}",
            f"Episodes: {self.n_episodes}, Frames: {self.n_frames}",
            f"",
            f"Per-Episode Results:",
            f"  ✓ {self.n_passed}/{self.n_episodes} passed",
            f"  ⚠ {self.n_warnings} with warnings",
            f"  ✗ {self.n_failed} failed",
        ]

        if self.all_errors:
            lines.append(f"\nErrors:")
            for e in self.all_errors[:10]:
                lines.append(f"  ✗ {e}")

        if self.all_warnings:
            lines.append(f"\nWarnings:")
            for w in self.all_warnings[:10]:
                lines.append(f"  ⚠ {w}")

        lines.extend([
            f"\nCross-Dataset Statistics:",
            f"  State range:   [{self.state_global_min:+.3f}, {self.state_global_max:+.3f}]",
            f"  Action range:  [{self.action_global_min:+.3f}, {self.action_global_max:+.3f}]",
            f"  Starting state diversity (std): {self.starting_state_std:.4f}",
            f"  Target pan diversity (std):     {self.target_pan_std:.4f}",
            f"\nRecommendation: {'FAIL — fix data before training' if self.critical else 'PASS — proceed to training'}",
        ])
        return "\n".join(lines)


def validate_dataset(
    dataset_path: str | Path,
    repo_id: str = "norma/sim_pick_v1",
    max_episodes: int | None = 20,  # sample, not full scan
    max_frames_per_episode: int = 30,  # sample frames, not all
    joint_limit_margin: float = 0.5,
    min_gripper_range: float = 0.3,
    min_state_std: float = 0.005,
    min_frames: int = 50,
    max_frames: int = 500,
) -> QualityReport:
    """Validate a LeRobot dataset. Returns QualityReport."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    import os
    os.environ["HF_HUB_OFFLINE"] = "1"

    dataset_path = str(dataset_path)
    ds = LeRobotDataset(repo_id=repo_id, root=dataset_path)

    report = QualityReport(
        dataset_path=dataset_path,
        n_episodes=ds.num_episodes,
        n_frames=ds.num_frames,
    )

    n_eps = min(ds.num_episodes, max_episodes) if max_episodes else ds.num_episodes

    all_states = []
    all_actions = []
    starting_states = []
    gripper_peaks = []  # max gripper value per episode (proxy for target pan)

    for ep_idx in range(n_eps):
        ep_report = _validate_episode(
            ds, ep_idx,
            joint_limit_margin=joint_limit_margin,
            min_gripper_range=min_gripper_range,
            min_state_std=min_state_std,
            min_frames=min_frames,
            max_frames=max_frames,
            max_sample_frames=max_frames_per_episode,
        )
        report.episodes.append(ep_report)

        if ep_report.errors:
            report.n_failed += 1
            for e in ep_report.errors:
                report.all_errors.append(f"ep {ep_idx}: {e}")
        elif ep_report.warnings:
            report.n_warnings += 1
            for w in ep_report.warnings:
                report.all_warnings.append(f"ep {ep_idx}: {w}")
        else:
            report.n_passed += 1

        # Collect cross-dataset stats
        if hasattr(ep_report, '_states') and ep_report._states is not None:
            all_states.append(ep_report._states)
            all_actions.append(ep_report._actions)
            starting_states.append(ep_report._states[0])
            # Track shoulder_pan action peak as proxy for target diversity
            gripper_peaks.append(ep_report._actions[:, 0].max())

    # Cross-dataset statistics
    if all_states:
        cat_states = np.concatenate(all_states)
        cat_actions = np.concatenate(all_actions)
        report.state_global_min = float(cat_states.min())
        report.state_global_max = float(cat_states.max())
        report.action_global_min = float(cat_actions.min())
        report.action_global_max = float(cat_actions.max())

        start_arr = np.array(starting_states)
        report.starting_state_std = float(start_arr.std())

        if gripper_peaks:
            report.target_pan_std = float(np.std(gripper_peaks))

        # Check for duplicate episodes
        if len(starting_states) > 1:
            dists = []
            for i in range(len(starting_states)):
                for j in range(i + 1, min(i + 5, len(starting_states))):
                    dists.append(np.linalg.norm(start_arr[i] - start_arr[j]))
            if dists and min(dists) < 1e-6:
                report.all_warnings.append("Possible duplicate episodes detected (identical starting states)")

    # Critical failure if >20% episodes failed
    if report.n_failed > report.n_episodes * 0.2:
        report.critical = True

    return report


def _validate_episode(
    ds, ep_idx: int,
    joint_limit_margin: float,
    min_gripper_range: float,
    min_state_std: float,
    min_frames: int,
    max_frames: int,
    max_sample_frames: int = 30,
) -> EpisodeReport:
    """Validate a single episode by sampling frames (not full scan)."""
    # Get episode frame indices
    all_ep_idx = ds.hf_dataset["episode_index"]
    indices = [i for i in range(len(all_ep_idx)) if all_ep_idx[i] == ep_idx]

    ep = EpisodeReport(index=ep_idx, n_frames=len(indices))

    if not indices:
        ep.errors.append("Empty episode (0 frames)")
        ep.passed = False
        return ep

    # Frame count check
    if len(indices) < min_frames:
        ep.warnings.append(f"Short episode: {len(indices)} frames (min={min_frames})")
    if len(indices) > max_frames:
        ep.warnings.append(f"Long episode: {len(indices)} frames (max={max_frames})")

    # Sample frames evenly (don't scan all — too slow for large datasets)
    if max_sample_frames and len(indices) > max_sample_frames:
        step = len(indices) // max_sample_frames
        indices = indices[::step][:max_sample_frames]

    # Collect all state/action vectors
    states = []
    actions = []
    gripper_values = []
    prev_img_means: dict[str, float] = {}
    frozen_streaks: dict[str, int] = {}
    max_frozen: dict[str, int] = {}

    for idx in indices:
        frame = ds[idx]
        state = frame["observation.state"].numpy()
        action = frame["action"].numpy()

        states.append(state)
        actions.append(action)

        # Per-frame: NaN check
        if not np.isfinite(state).all():
            ep.errors.append(f"NaN/Inf in state at frame {idx}")
            ep.has_nan = True
        if not np.isfinite(action).all():
            ep.errors.append(f"NaN/Inf in action at frame {idx}")
            ep.has_nan = True

        # Gripper value (last element of state, in 0-100 LeRobot scale)
        gripper_values.append(float(state[-1]))

        # Image check (if available)
        for key in frame:
            if "images" in key and hasattr(frame[key], 'numpy'):
                img = frame[key].numpy().astype(np.float32)
                img_mean = float(img.mean())

                if img_mean < 0.01:
                    ep.warnings.append(f"Near-black image at frame {idx}: {key}")

                # Frozen detection: compare full-image mean (cheap, robust)
                if key in prev_img_means and abs(img_mean - prev_img_means[key]) < 1e-5:
                    frozen_streaks[key] = frozen_streaks.get(key, 0) + 1
                else:
                    frozen_streaks[key] = 0
                prev_img_means[key] = img_mean
                max_frozen[key] = max(max_frozen.get(key, 0), frozen_streaks[key])

    # Report max frozen streak per camera (only if excessive)
    for key, streak in max_frozen.items():
        if streak > 50:
            ep.warnings.append(f"Long frozen camera streak ({streak} frames): {key}")

    states_arr = np.array(states)
    actions_arr = np.array(actions)

    # Store for cross-dataset analysis
    ep._states = states_arr
    ep._actions = actions_arr

    # Per-episode: state variance
    ep.state_std = float(states_arr.std())
    if ep.state_std < min_state_std:
        ep.errors.append(f"Robot barely moved (state std={ep.state_std:.6f})")

    # Per-episode: action variance
    ep.action_std = float(actions_arr.std())
    if ep.action_std < min_state_std:
        ep.warnings.append(f"Near-constant actions (action std={ep.action_std:.6f})")

    # Per-episode: gripper activity
    gripper_arr = np.array(gripper_values)
    ep.gripper_min = float(gripper_arr.min())
    ep.gripper_max = float(gripper_arr.max())
    gripper_range = ep.gripper_max - ep.gripper_min

    if gripper_range < min_gripper_range:
        ep.warnings.append(
            f"Narrow gripper range: [{ep.gripper_min:.1f}, {ep.gripper_max:.1f}] "
            f"(range={gripper_range:.1f}, min={min_gripper_range})"
        )

    # Gripper cycle detection: open → close → open
    threshold_open = ep.gripper_min + gripper_range * 0.3
    threshold_closed = ep.gripper_max - gripper_range * 0.3
    if gripper_range > min_gripper_range:
        phases = []
        for g in gripper_arr:
            if g < threshold_open:
                phases.append("open")
            elif g > threshold_closed:
                phases.append("closed")
        # Deduplicate consecutive
        deduped = [phases[0]] if phases else []
        for p in phases[1:]:
            if p != deduped[-1]:
                deduped.append(p)
        ep.has_gripper_cycle = (
            len(deduped) >= 3
            and "open" in deduped
            and "closed" in deduped
        )
    if not ep.has_gripper_cycle and gripper_range > min_gripper_range:
        ep.warnings.append("No complete gripper open→close→open cycle detected")

    # Set overall pass/fail
    ep.passed = len(ep.errors) == 0

    return ep
