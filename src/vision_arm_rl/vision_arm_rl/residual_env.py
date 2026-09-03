"""One grasp attempt as one Gymnasium step.

The correction is applied open loop, before the arm moves, and the reward
arrives when the cycle ends. There is no state carried from one attempt to the
next, so this is a contextual bandit with an episode length of one rather than a
policy with a planning horizon. That is deliberate: an attempt costs the better
part of a minute of simulated execution, which affords a few thousand samples
and not the hundreds of thousands a step by step controller would need.

Observation: where the detector says the cube is, in base_link.
Action: how far to move the grasp point, per axis, in [-1, 1].
"""
import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .bench import RESIDUAL_LIMIT, X_RANGE, Y_RANGE, GraspBench

# Observations are normalised to roughly [-1, 1] before they reach the network.
# Raw metres would hand the policy a y around -0.45 and a z around 0.04, two
# orders of magnitude apart, which is a needlessly hard scaling problem.
OBS_LOW = np.array([X_RANGE[0], Y_RANGE[0], -0.05], dtype=np.float32)
OBS_HIGH = np.array([X_RANGE[1], Y_RANGE[1], 0.15], dtype=np.float32)


class ResidualGraspEnv(gym.Env):
    """Wraps a live GraspBench. One reset and one step per grasp."""

    metadata = {"render_modes": []}

    def __init__(self, bench=None, plan_penalty=0.25, effort_penalty=0.05, **kwargs):
        super().__init__()
        self.bench = bench or GraspBench(**kwargs)
        self.plan_penalty = plan_penalty
        self.effort_penalty = effort_penalty
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self._obs = np.zeros(3, dtype=np.float32)
        self.bench.park_others()

    @staticmethod
    def normalise(xyz):
        span = OBS_HIGH - OBS_LOW
        return np.clip(
            2.0 * (np.asarray(xyz, dtype=np.float32) - OBS_LOW) / span - 1.0,
            -1.0, 1.0).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.bench.new_placement()
        # Observe what the DETECTOR reports, not the truth we just commanded.
        # The whole point of the residual is to correct a detection that is
        # systematically wrong, so training on ground truth would remove the
        # error the policy exists to cancel.
        detected = self.bench.seen[self.bench.colour]
        self._obs = self.normalise(detected)
        return self._obs, {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        residual = action * RESIDUAL_LIMIT
        outcome = self.bench.attempt(tuple(float(v) for v in residual))

        reward = float(outcome["progress"])
        if outcome["success"]:
            reward += 1.0
        if outcome["planning_failed"]:
            # A correction that cannot be reached is not merely unrewarded, it
            # is worse than doing nothing: the arm never moves and the attempt
            # is spent.
            reward -= self.plan_penalty
        # Keep the correction near zero wherever the scripted grasp is already
        # right, so the policy acts only where it is actually biased.
        reward -= self.effort_penalty * float(np.mean(np.abs(action)))

        outcome["residual_m"] = tuple(float(v) for v in residual)
        return self._obs, reward, True, False, outcome

    def close(self):
        self.bench.close()
