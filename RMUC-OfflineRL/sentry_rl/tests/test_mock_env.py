import numpy as np
import pytest

from sentry_rl.config import load_config, obs_dim
from sentry_rl.envs.mock import MockSentryEnv


def test_reset_and_step():
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    obs = env.reset()
    assert obs.shape == (obs_dim(cfg),)
    assert env.action_mask().any()
    obs2, reward, done, info = env.step_macro(0)
    assert obs2.shape == obs.shape
    assert isinstance(reward, float)
    assert isinstance(done, bool)
    assert info["decisions"] == 1


def test_invalid_action_rejected():
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    env.reset()
    with pytest.raises(ValueError):
        env.step_macro(999)


def test_episode_terminates():
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    env.reset()
    done = False
    steps = 0
    while not done:
        obs, reward, done, info = env.step_macro(0)
        steps += 1
        assert steps <= int(cfg["decision"]["max_decisions"])
    assert done
