import numpy as np

from sentry_rl.config import load_config
from sentry_rl.data.collect import collect
from sentry_rl.envs.mock import MockSentryEnv


def test_collect_writes_npz(tmp_path):
    cfg = load_config()
    env = MockSentryEnv(cfg, seed=0)
    out = tmp_path / "bc.npz"
    info = collect(env, episodes=2, out_path=out, seed=0)
    assert info["samples"] == sum(info["counts"])
    data = np.load(out)
    assert data["obs"].ndim == 2
    assert data["mask"].shape[1] == 7
    assert data["action"].shape[0] == data["obs"].shape[0]
