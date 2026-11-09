"""Run one short real Arena-1 match with example strategies.

Usage (from sentry_rl/):
    python scripts/run_arena_smoke.py --duration 30
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sentry_rl.config import load_config, obs_dim
from sentry_rl.envs.arena1 import Arena1SentryEnv, ModulePolicy
from sentry_rl.models.policy import load_policy

REPO = ROOT.parent
DEFAULT_RED = REPO / "Arena-1/examples/claude_opus55_xhigh_2027_team"
DEFAULT_BLUE = REPO / "Arena-1/examples/claude_opus55_xhigh_2027_hold"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs/sentry_2027.yaml"))
    parser.add_argument("--duration", type=int, default=30)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--red", default=str(DEFAULT_RED))
    parser.add_argument("--blue", default=str(DEFAULT_BLUE))
    parser.add_argument("--checkpoint", default="", help="optional BC/PPO checkpoint")
    args = parser.parse_args()

    cfg = load_config(args.config)
    cfg["arena"]["duration_s"] = args.duration
    env = Arena1SentryEnv(
        cfg, seed=args.seed,
        red_teammates=ModulePolicy(args.red),
        blue_policy=ModulePolicy(args.blue),
    )
    policy = None
    if args.checkpoint:
        policy = load_policy(ROOT / args.checkpoint, cfg, obs_dim(cfg))
    import torch
    try:
        obs = env.reset()
        print("obs", obs.shape, "mask", env.action_mask().tolist())
        done = False
        while not done:
            mask = env.action_mask()
            if policy is not None:
                obs_t = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)
                mask_t = torch.as_tensor(mask, dtype=torch.bool).unsqueeze(0)
                action = int(policy.act(obs_t, mask_t, deterministic=True)[0].item())
                if not mask[action]:
                    action = int(np.flatnonzero(mask)[0])
            else:
                action = int(np.flatnonzero(mask)[0])
            obs, reward, done, info = env.step_macro(action)
            print(json.dumps({"action": action, "reward": round(float(reward), 3), **info},
                             ensure_ascii=False))
        print("final", env.info())
    finally:
        env.close()


if __name__ == "__main__":
    main()
