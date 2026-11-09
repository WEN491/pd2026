from pathlib import Path

from sentry_rl.config import goal, load_config, obs_dim, resolve_path


def test_obs_dim_and_goals():
    cfg = load_config("configs/sentry_2027.yaml")
    assert obs_dim(cfg) == 59
    assert len(goal(cfg, "hold_outpost")) == 2


def test_paths_resolve_from_workspace_root():
    cfg = load_config("configs/sentry_2027.yaml")
    path = resolve_path(cfg, "runs/data/x.npz")
    assert path == Path(cfg["_workspace_dir"]) / "runs/data/x.npz"
