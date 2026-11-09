import numpy as np
import pytest

torch = pytest.importorskip("torch")

from sentry_rl.config import load_config, obs_dim
from sentry_rl.models.policy import build_policy


def test_forward_shapes_and_mask():
    cfg = load_config()
    policy = build_policy(cfg, obs_dim(cfg))
    obs = torch.zeros(2, obs_dim(cfg))
    mask = torch.ones(2, 7, dtype=torch.bool)
    mask[:, 3] = False
    dist, value = policy(obs, mask)
    assert dist.logits.shape == (2, 7)
    assert value.shape == (2,)
    assert torch.all(dist.logits[:, 3] < -1e8)


def test_deterministic_act():
    cfg = load_config()
    policy = build_policy(cfg, obs_dim(cfg))
    obs = torch.zeros(1, obs_dim(cfg))
    mask = torch.ones(1, 7, dtype=torch.bool)
    action, logp, value = policy.act(obs, mask, deterministic=True)
    assert action.shape == (1,)
    assert logp.shape == (1,)
    assert value.shape == (1,)
