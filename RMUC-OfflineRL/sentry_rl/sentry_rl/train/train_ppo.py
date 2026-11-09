"""CLI: PPO fine-tuning of the 7-intent sentry policy."""
from __future__ import annotations

import argparse

from ..algos.ppo import train_ppo
from ..config import load_config, obs_dim, resolve_path
from ..envs.arena1 import Arena1SentryEnv, ModulePolicy
from ..envs.mock import MockSentryEnv


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/sentry_2027.yaml")
    parser.add_argument("--init", default="")
    parser.add_argument("--out", default="checkpoints/ppo.pt")
    parser.add_argument("--metrics", default="runs/ppo_metrics.jsonl")
    parser.add_argument("--mock", action="store_true", help="use MockSentryEnv (no Arena-1)")
    parser.add_argument("--red-teammates", default="", help="Arena-1 strategy dir for slots 1/3/4/6")
    parser.add_argument("--blue", default="", help="Arena-1 strategy dir for the opponent")
    parser.add_argument("--n-envs", type=int, default=0)
    parser.add_argument("--total-steps", type=int, default=0)
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.n_envs:
        cfg["ppo"]["n_envs"] = args.n_envs
    if args.total_steps:
        cfg["ppo"]["total_steps"] = args.total_steps
    cfg["_obs_dim"] = obs_dim(cfg)

    if args.mock:
        def factory(i: int):
            return MockSentryEnv(cfg, seed=int(cfg["seed"]) + i)
    else:
        red_path, blue_path = args.red_teammates, args.blue

        def factory(i: int):
            return Arena1SentryEnv(
                cfg, seed=int(cfg["seed"]) + i,
                red_teammates=ModulePolicy(red_path) if red_path else None,
                blue_policy=ModulePolicy(blue_path) if blue_path else None,
            )

    train_ppo(cfg, factory,
              resolve_path(cfg, args.out),
              init_path=resolve_path(cfg, args.init) if args.init else None,
              metrics_path=resolve_path(cfg, args.metrics) if args.metrics else None,
              obs_dim=obs_dim(cfg))


if __name__ == "__main__":
    main()
