import numpy as np

from sentry_rl.config import load_config, obs_dim
from sentry_rl.envs.mock import MockSentryEnv
from sentry_rl.obs import build_obs, obs_names


def test_obs_shape_and_finiteness():
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    obs = env.reset()
    assert obs.shape == (obs_dim(cfg),)
    assert obs.dtype == np.float32
    assert np.isfinite(obs).all()
    assert len(obs_names(cfg)) == obs_dim(cfg)


def test_obs_without_radar_is_still_finite():
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    env.reset()
    obs = build_obs(env.observation, None, cfg)
    assert obs.shape == (obs_dim(cfg),)
    assert np.isfinite(obs).all()
    # radar link bit is third from the end: [..., link, age, time]
    assert obs[-3] == 0.0
