"""Checks that do not need the simulator.

Everything the environment does that is not a ROS call is arithmetic, and
arithmetic is worth pinning down before spending hours of wall clock on it. The
bench is injected, so these run against a stub instead of Gazebo.
"""
import pathlib
import re

import numpy as np
import pytest

from vision_arm_rl.bench import RESIDUAL_LIMIT, X_RANGE, Y_RANGE
from vision_arm_rl.residual_env import ResidualGraspEnv

TASK_SERVER = (pathlib.Path(__file__).resolve().parents[2]
               / "vision_arm_tasks" / "nodes" / "task_server.py")


class FakeBench:
    """A bench that records what it was asked to do and never talks to ROS."""

    def __init__(self, outcome=None, detected=(0.1, -0.45, 0.04)):
        self.colour = "red"
        self.seen = {"red": detected}
        self.asked = None
        self.placements = 0
        self.outcome = outcome or {
            "verdict": "done: ok", "success": True, "planning_failed": False,
            "start": (0.1, -0.45, 0.775), "final": (-0.2, -0.42, 0.775),
            "progress": 1.0,
        }

    def park_others(self):
        pass

    def new_placement(self):
        self.placements += 1
        return self.seen["red"][:2]

    def attempt(self, residual=(0.0, 0.0, 0.0)):
        self.asked = residual
        return dict(self.outcome)

    def close(self):
        pass


def env(**kwargs):
    return ResidualGraspEnv(bench=FakeBench(**kwargs))


def test_spaces_are_three_dimensional():
    e = env()
    assert e.observation_space.shape == (3,)
    assert e.action_space.shape == (3,)


def test_normalise_maps_the_sampling_box_onto_minus_one_to_one():
    low = ResidualGraspEnv.normalise((X_RANGE[0], Y_RANGE[0], -0.05))
    high = ResidualGraspEnv.normalise((X_RANGE[1], Y_RANGE[1], 0.15))
    assert np.allclose(low, -1.0)
    assert np.allclose(high, 1.0)


def test_normalise_clips_outside_the_box():
    assert np.all(ResidualGraspEnv.normalise((10.0, 10.0, 10.0)) <= 1.0)
    assert np.all(ResidualGraspEnv.normalise((-10.0, -10.0, -10.0)) >= -1.0)


def test_zero_action_asks_for_no_correction():
    """The baseline has to be the scripted grasp exactly, or it is not a baseline."""
    e = env()
    e.reset()
    e.step(np.zeros(3, dtype=np.float32))
    assert e.bench.asked == (0.0, 0.0, 0.0)


def test_action_is_scaled_and_clipped_to_the_envelope():
    e = env()
    e.reset()
    e.step(np.array([5.0, -5.0, 0.5], dtype=np.float32))
    assert e.bench.asked == pytest.approx(
        (RESIDUAL_LIMIT, -RESIDUAL_LIMIT, 0.5 * RESIDUAL_LIMIT))


def test_every_episode_is_one_step():
    e = env()
    e.reset()
    _, _, terminated, truncated, _ = e.step(np.zeros(3, dtype=np.float32))
    assert terminated and not truncated


def test_success_scores_above_a_miss():
    win = env()
    win.reset()
    _, good, _, _, _ = win.step(np.zeros(3, dtype=np.float32))

    miss = env(outcome={
        "verdict": "failed: closed on nothing", "success": False,
        "planning_failed": False, "start": (0.1, -0.45, 0.775),
        "final": (0.1, -0.45, 0.775), "progress": 0.0})
    miss.reset()
    _, bad, _, _, _ = miss.step(np.zeros(3, dtype=np.float32))
    assert good > bad


def test_unreachable_correction_scores_below_a_miss():
    """A correction the arm cannot reach wastes the attempt entirely."""
    miss = env(outcome={
        "verdict": "failed: closed on nothing", "success": False,
        "planning_failed": False, "start": (0.1, -0.45, 0.775),
        "final": (0.1, -0.45, 0.775), "progress": 0.0})
    miss.reset()
    _, bad, _, _, _ = miss.step(np.zeros(3, dtype=np.float32))

    unreachable = env(outcome={
        "verdict": "failed: approach: motion failed (-2)", "success": False,
        "planning_failed": True, "start": (0.1, -0.45, 0.775),
        "final": (0.1, -0.45, 0.775), "progress": 0.0})
    unreachable.reset()
    _, worse, _, _, _ = unreachable.step(np.zeros(3, dtype=np.float32))
    assert worse < bad


def test_effort_is_penalised_so_zero_stays_the_default():
    """Two identical successes: the one that moved the grasp less scores higher."""
    lazy = env()
    lazy.reset()
    _, still, _, _, _ = lazy.step(np.zeros(3, dtype=np.float32))

    busy = env()
    busy.reset()
    _, moved, _, _, _ = busy.step(np.ones(3, dtype=np.float32))
    assert still > moved


def test_residual_limit_matches_the_task_server():
    """The task server clips whatever arrives; drifting apart wastes action range."""
    source = TASK_SERVER.read_text()
    found = re.search(r"^RESIDUAL_LIMIT = ([0-9.]+)", source, re.M)
    assert found, f"no RESIDUAL_LIMIT in {TASK_SERVER}"
    assert float(found.group(1)) == RESIDUAL_LIMIT
