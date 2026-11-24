"""Tests of the training environment. Run with: python -m pytest -q"""

import numpy as np
import pytest

from env import N_RAYS, MiniEnv


@pytest.fixture(scope="module", params=["mini13", "mini16", "mini14"])
def env(request):
    return MiniEnv(terrain="mix", robot=request.param, seed=0)


def test_spaces_and_observation(env):
    obs, info = env.reset(seed=1, options={"terrain": ("flat", 0)})
    assert env.observation_space.shape == obs.shape
    assert env.action_space.shape == (env.n_ctrl,)
    assert np.all(np.isfinite(obs))


def test_zero_action_stands_on_flat(env):
    """With zero action it is the scripted gait: with random friction, mass and servos it must not
    fall within 5 s. (Forward progress is not required: with the variations, the scripted gait of the
    real Mini16 barely moves forward; improving that is the RL policy's job.)"""
    env.reset(seed=2, options={"terrain": ("flat", 0)})
    for _ in range(250):
        obs, r, term, trunc, info = env.step(np.zeros(env.n_ctrl))
        assert not term


def test_distance_sensor_sees_step(env):
    obs, _ = env.reset(seed=3, options={"terrain": ("step", 0.02)})
    rays = obs[-N_RAYS:]
    # The first rays see the floor; the last ones, the step (closer to the torso).
    assert rays[0] > rays[-1] + 0.2


def test_servo_speed_limit(env):
    env.reset(seed=4, options={"terrain": ("flat", 0)})
    n = env.n_ctrl
    prev = env.d.ctrl[:n].copy()
    env.step(np.ones(n))
    assert np.all(np.abs(env.d.ctrl[:n] - prev) <= env.rb.servo_speed / 50 + 1e-9)
