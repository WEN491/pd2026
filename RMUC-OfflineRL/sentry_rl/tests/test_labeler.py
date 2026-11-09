from sentry_rl.config import load_config
from sentry_rl.data.labeler import infer_intent
from sentry_rl.envs.mock import MockSentryEnv


def test_hold_outpost_label():
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    env.reset()
    env.own["x"], env.own["y"] = -3.02, 3.90
    env._refresh()
    assert infer_intent(env.observation, cfg) == 0


def test_take_outpost_label():
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    env.reset()
    env.own["x"], env.own["y"] = 3.02, -3.90
    env._refresh()
    assert infer_intent(env.observation, cfg) == 2


def test_retreat_label():
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    env.reset()
    env.own["hp"] = 60.0
    env.own["x"], env.own["y"] = -12.0, 5.2
    env._refresh()
    assert infer_intent(env.observation, cfg) == 6
