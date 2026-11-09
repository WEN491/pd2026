"""Configuration loading and small helpers.

The YAML in ``configs/`` is the single source of truth.  Keep the defaults here
minimal: they only exist so tests and the mock environment can run without a
config file.
"""
from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml

DEFAULTS: dict[str, Any] = {
    "seed": 42,
    "device": "cpu",
    "team": "red",
    "own_slot": "7",
    "enemy_slots": ["1", "3", "4", "7"],
    "field": {"width": 28.0, "height": 15.0, "center_x": 14.0, "center_y": 7.5},
    "decision": {"interval_s": 5.0, "engine_dt": 0.05, "steps_per_decision": 100, "max_decisions": 84},
    "lineup": {"1": 2711, "3": 2731, "4": 2732, "6": 2761, "7": 2771},
    "goals": {
        "hold_outpost": [-3.02, 3.90],
        "hold_base": [-7.43, 0.00],
        "take_outpost": [3.02, -3.90],
        "take_base": [7.43, 0.00],
        "resupply": [-12.60, 5.50],
        "retreat": [-12.60, 5.50],
    },
    "arena": {"map": "rmuc2027", "duration_s": 420, "python_path": "../Arena-1", "sim_binary": ""},
    "radar": {"mode": "synthetic", "rate_hz": 5.0, "sigma_m": 0.7, "latency_s": 0.15,
              "dropout": 0.10, "timeout_s": 1.0},
    "reward": {"control": 1.0, "structure": 0.001, "damage_dealt": 0.01,
               "damage_taken": 0.01, "hit": 0.05, "potential": 1.0,
               "win": 10.0, "lose": 10.0, "base_destroyed": 5.0},
    "bc": {"epochs": 30, "batch_size": 256, "lr": 1e-3, "label_smoothing": 0.05,
           "class_weights": "auto", "seed": 42},
    "ppo": {"total_steps": 500_000, "n_envs": 16, "rollout_decisions": 84,
            "batch_size": 4096, "epochs": 4, "lr": 3e-4, "clip": 0.2,
            "entropy": 0.01, "value_coef": 0.5, "max_grad_norm": 0.5,
            "gamma": 0.99, "lam": 0.95, "bc_anchor": 0.0},
    "eval": {"seeds": 20, "swap_sides": True, "max_decisions": 84},
    "paths": {"data": "data", "runs": "runs", "checkpoints": "checkpoints"},
}


def _deep_merge(base: dict, extra: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in (extra or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_config(path: str | Path | None = None) -> dict:
    """Load YAML on top of :data:`DEFAULTS`; remember the config path."""
    cfg = copy.deepcopy(DEFAULTS)
    if path is not None:
        path = Path(path).expanduser().resolve()
        with path.open("r", encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream) or {}
        cfg = _deep_merge(cfg, loaded)
        cfg["_config_path"] = str(path)
        cfg["_config_dir"] = str(path.parent)
        cfg["_workspace_dir"] = str(path.parent.parent)
    else:
        cfg["_config_path"] = ""
        cfg["_config_dir"] = str(Path.cwd())
        cfg["_workspace_dir"] = str(Path.cwd())
    return cfg


def resolve_path(cfg: dict, value: str | Path) -> Path:
    """Resolve a config-relative path; absolute paths pass through."""
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    base = Path(cfg.get("_workspace_dir") or cfg.get("_config_dir") or Path.cwd())
    return (base / path).resolve()


def goal(cfg: dict, name: str) -> list[float]:
    """Return a named field goal as ``[x, y]``."""
    goals = cfg["goals"]
    if name not in goals:
        raise KeyError(f"unknown goal {name!r}; have {sorted(goals)}")
    return [float(goals[name][0]), float(goals[name][1])]


def obs_dim(cfg: dict) -> int:
    """Observation size for the configured enemy slots."""
    return 5 * len(cfg["enemy_slots"]) + 12 + 12 + 12 + 2 + 1
