import numpy as np

from sentry_rl.actions import Intent, compute_mask, masked_softmax
from sentry_rl.config import load_config
from sentry_rl.envs.mock import MockSentryEnv


def test_mask_at_start():
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    env.reset()
    mask = compute_mask(env.observation, cfg)
    assert mask[Intent.HOLD_OUTPOST]
    assert mask[Intent.TAKE_OUTPOST]
    assert not mask[Intent.TAKE_BASE]
    assert not mask[Intent.RESUPPLY]


def test_take_base_after_180s_and_control():
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    env.reset()
    env.time = 200.0
    env.positions["blue_outpost"]["controller"] = "red"
    env._refresh()
    mask = compute_mask(env.observation, cfg)
    assert mask[Intent.TAKE_BASE]


def test_low_hp_adds_retreat_and_resupply():
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    env.reset()
    env.own["hp"] = 100.0
    env.own["ammo"] = 20
    env.enemies[0]["x"] = env.own["x"] + 3.0
    env.enemies[0]["y"] = env.own["y"]
    env._refresh()
    mask = compute_mask(env.observation, cfg)
    assert mask[Intent.RETREAT]
    assert mask[Intent.RESUPPLY]


def test_masked_softmax_never_selects_masked():
    logits = np.array([0.0, 1.0, 2.0], dtype=np.float32)
    mask = np.array([True, False, True])
    probs = masked_softmax(logits, mask)
    assert probs.shape == (3,)
    assert probs[1] == 0.0
    assert abs(float(probs.sum()) - 1.0) < 1e-6
