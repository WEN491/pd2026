"""CLI: collect BC data by running a scripted sentry.

For a real Arena-1 run the sentry is driven by ``--sentry`` (an Arena-1
strategy directory); for a mock run the macro executor is used.  Each decision
stores the restricted observation, the action mask and an inferred intent.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from ..actions import N_ACTIONS
from ..config import load_config, resolve_path
from ..envs.arena1 import Arena1SentryEnv, ModulePolicy
from ..envs.mock import MockSentryEnv
from .labeler import infer_intent


def collect(env, episodes: int, out_path: str | Path, seed: int = 0) -> dict:
    obs_rows, mask_rows, act_rows = [], [], []
    for episode in range(episodes):
        obs = env.reset(seed + episode)
        done = False
        while not done:
            mask = env.action_mask()
            intent = infer_intent(env.observation, env.cfg)
            if not mask[intent]:
                intent = int(np.flatnonzero(mask)[0])
            obs_rows.append(np.asarray(obs, dtype=np.float32))
            mask_rows.append(np.asarray(mask, dtype=bool))
            act_rows.append(int(intent))
            obs, _reward, done, _info = env.step_macro(intent)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out_path,
                        obs=np.asarray(obs_rows, dtype=np.float32),
                        mask=np.asarray(mask_rows, dtype=bool),
                        action=np.asarray(act_rows, dtype=np.int64))
    close = getattr(env, "close", None)
    if callable(close):
        close()
    counts = np.bincount(np.asarray(act_rows, dtype=np.int64), minlength=N_ACTIONS)
    return {"samples": len(act_rows), "counts": counts.tolist(), "out": str(out_path)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/sentry_2027.yaml")
    parser.add_argument("--out", default="runs/data/bc.npz")
    parser.add_argument("--episodes", type=int, default=8)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--sentry", default="", help="Arena-1 strategy dir for slot 7")
    parser.add_argument("--red-teammates", default="", help="Arena-1 strategy dir for slots 1/3/4/6")
    parser.add_argument("--blue", default="", help="Arena-1 strategy dir for the opponent")
    parser.add_argument("--duration", type=int, default=0, help="override arena.duration_s (short smoke match)")
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.duration:
        cfg["arena"]["duration_s"] = args.duration
    if args.mock:
        env = MockSentryEnv(cfg, seed=int(cfg["seed"]))
    else:
        env = Arena1SentryEnv(
            cfg,
            seed=int(cfg["seed"]),
            sentry_policy=ModulePolicy(args.sentry) if args.sentry else None,
            red_teammates=ModulePolicy(args.red_teammates) if args.red_teammates else None,
            blue_policy=ModulePolicy(args.blue) if args.blue else None,
        )
    info = collect(env, args.episodes, resolve_path(cfg, args.out), seed=int(cfg["seed"]))
    print(info)


if __name__ == "__main__":
    main()
