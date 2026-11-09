"""Evaluation: deterministic policy, fixed seeds, control-degree metrics."""
from __future__ import annotations

import argparse
import json
import statistics

import numpy as np
import torch

from ..config import load_config, obs_dim, resolve_path
from ..envs.arena1 import Arena1SentryEnv, ModulePolicy
from ..envs.mock import MockSentryEnv
from ..models.policy import load_policy


def evaluate(policy, env, seeds: list[int], max_decisions: int | None = None) -> dict:
    device = next(policy.parameters()).device
    rows = []
    for seed in seeds:
        obs = env.reset(seed)
        done = False
        total = 0.0
        decisions = 0
        info = {"control": 0.0, "winner": ""}
        while not done:
            mask = env.action_mask()
            obs_t = torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
            mask_t = torch.as_tensor(mask, dtype=torch.bool, device=device).unsqueeze(0)
            action, _, _ = policy.act(obs_t, mask_t, deterministic=True)
            obs, reward, done, info = env.step_macro(int(action.item()))
            total += float(reward)
            decisions += 1
            if max_decisions and decisions >= max_decisions:
                break
        rows.append({"seed": seed, "return": total, "decisions": decisions,
                     "control": info.get("control", 0.0), "winner": info.get("winner", "")})
    controls = [row["control"] for row in rows]
    returns = [row["return"] for row in rows]
    wins = [1.0 if row["winner"] == getattr(env, "cfg", {}).get("team", "red") else 0.0 for row in rows]
    return {
        "episodes": len(rows),
        "control_mean": statistics.fmean(controls) if controls else 0.0,
        "control_std": statistics.pstdev(controls) if len(controls) > 1 else 0.0,
        "return_mean": statistics.fmean(returns) if returns else 0.0,
        "win_rate": statistics.fmean(wins) if wins else 0.0,
        "rows": rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/sentry_2027.yaml")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--seeds", type=int, default=0)
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    cfg = load_config(args.config)
    n_seeds = args.seeds or int(cfg["eval"]["seeds"])
    seeds = list(range(int(cfg["seed"]), int(cfg["seed"]) + n_seeds))
    policy = load_policy(resolve_path(cfg, args.checkpoint), cfg, obs_dim(cfg))
    if args.mock:
        env = MockSentryEnv(cfg, seed=seeds[0])
    else:
        env = Arena1SentryEnv(cfg, seed=seeds[0])
    result = evaluate(policy, env, seeds, int(cfg["eval"]["max_decisions"]))
    text = json.dumps({key: value for key, value in result.items() if key != "rows"},
                      ensure_ascii=False, indent=2)
    print(text)
    if args.out:
        out = resolve_path(cfg, args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
