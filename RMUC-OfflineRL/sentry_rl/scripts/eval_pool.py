"""Fixed-schedule pool exam for one sentry checkpoint.

The exam is a *contract*: the same opponents, seeds, sides, teammate
composition and checkpoint hash must reproduce the same numbers.  Held-out
opponents (a deterministic split of the example pool, see
``sentry_rl.pools.split_held_out``) measure generalisation rather than the
opponents the policy was tuned against.

Usage (from ``sentry_rl/``):

    python scripts/eval_pool.py --checkpoint checkpoints/ppo.pt \
        --opponents 4 --seeds 1 --out runs/eval/pool_exam.json

    python scripts/eval_pool.py --held-out --no-swap --duration 120   # smoke

One 420 s match takes roughly 2.5 minutes of wall clock with in-process
example strategies, so an 4-opponent x 1-seed x 2-side exam is ~20 minutes.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import random
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sentry_rl.config import load_config, obs_dim, resolve_path
from sentry_rl.envs.arena1 import Arena1SentryEnv
from sentry_rl.models.policy import load_policy
from sentry_rl.pools import (
    OpponentPool,
    TeammatePool,
    discover,
    entries_digest,
    save_pool,
    split_held_out,
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _select_opponents(pool_entries: list, held_out: list, args) -> list:
    candidates = held_out if args.held_out else pool_entries
    if args.opponent:
        wanted = set(args.opponent)
        candidates = [e for e in candidates if e.name in wanted]
        missing = wanted - {e.name for e in candidates}
        if missing:
            raise SystemExit(f"unknown or held-out opponent(s): {sorted(missing)}")
    chosen = sorted(candidates, key=lambda e: e.key)[: int(args.opponents)]
    if not chosen:
        raise SystemExit("no opponents selected")
    return chosen


def _teammate_entries(pool: TeammatePool, lineup: dict, mix: str, seed: int) -> list:
    rng = random.Random(seed)
    picked = pool.sample(lineup, rng, mix=mix)
    return sorted(picked.items())


def _run_cell(cfg: dict, checkpoint, opponent, teammate_entries, seed: int,
              swapped: bool, args) -> dict:
    cfg = copy.deepcopy(cfg)
    cfg["team"] = "blue" if swapped else "red"
    cfg["arena"]["duration_s"] = int(args.duration)
    cfg["decision"]["max_decisions"] = int(args.max_decisions) if args.max_decisions else (
        int(args.duration) // int(cfg["decision"]["interval_s"])
    )
    env = Arena1SentryEnv(
        cfg,
        seed=seed,
        teammate_entries=teammate_entries,
        opponent_entry=opponent,
        pool_seed=seed,
    )
    counts = [0] * 7
    started = time.time()
    try:
        obs = env.reset(seed)
        done = False
        while not done:
            mask = env.action_mask()
            if checkpoint is None:
                action = int(next(i for i, ok in enumerate(mask) if ok))
            else:
                import torch
                obs_t = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)
                mask_t = torch.as_tensor(mask, dtype=torch.bool).unsqueeze(0)
                action = int(checkpoint.act(obs_t, mask_t, deterministic=True)[0].item())
                if not mask[action]:
                    action = int(next(i for i, ok in enumerate(mask) if ok))
            obs, _reward, done, _info = env.step_macro(action)
            counts[action] += 1
        info = env.info()
        result = {
            "opponent": opponent.key,
            "seed": seed,
            "side": env.team,
            "swapped": bool(swapped),
            "winner": info["winner"],
            "won": info["winner"] == env.team,
            "draw": info["winner"] not in (env.team, env.opponent),
            "control": info["control"],
            "decisions": info["decisions"],
            "time": info["time"],
            "sentry_hp": info["sentry_hp"],
            "reason": info.get("reason", ""),
            "actions": counts,
            "composition": env.composition,
            "wall_s": round(time.time() - started, 1),
        }
    finally:
        env.close()
    return result


def _aggregate(cells: list) -> dict:
    wins = sum(1 for c in cells if c["won"])
    draws = sum(1 for c in cells if c["draw"])
    by_side: dict[str, list] = {}
    by_opponent: dict[str, list] = {}
    for cell in cells:
        by_side.setdefault(cell["side"], []).append(cell)
        by_opponent.setdefault(cell["opponent"], []).append(cell)

    def block(group: list) -> dict:
        return {
            "matches": len(group),
            "mean_time": round(statistics.fmean(c["time"] for c in group), 1),
            "early_ends": sum(1 for c in group if c.get("reason") == "base_destroyed"),
            "wins": sum(1 for c in group if c["won"]),
            "draws": sum(1 for c in group if c["draw"]),
            "win_rate": round(sum(1 for c in group if c["won"]) / len(group), 3),
            "control_mean": round(statistics.fmean(c["control"] for c in group), 3),
        }

    return {
        "matches": len(cells),
        "wins": wins,
        "draws": draws,
        "win_rate": round(wins / len(cells), 3) if cells else 0.0,
        "control_mean": round(statistics.fmean(c["control"] for c in cells), 3) if cells else 0.0,
        "by_side": {side: block(group) for side, group in sorted(by_side.items())},
        "by_opponent": {name: block(group) for name, group in sorted(by_opponent.items())},
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs/sentry_2027.yaml"))
    parser.add_argument("--checkpoint", default="")
    parser.add_argument("--opponents", type=int, default=4)
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--seed-base", type=int, default=1000)
    parser.add_argument("--duration", type=int, default=420)
    parser.add_argument("--max-decisions", type=int, default=0)
    parser.add_argument("--mix", choices=["hetero", "homo"], default="hetero")
    parser.add_argument("--held-out", action="store_true")
    parser.add_argument("--no-swap", action="store_true")
    parser.add_argument("--opponent", action="append", default=[],
                        help="restrict to these example names (repeatable)")
    parser.add_argument("--out", default=str(ROOT / "runs/eval/pool_exam.json"))
    parser.add_argument("--pool-out", default=str(ROOT / "runs/pools/frozen_pool.json"))
    args = parser.parse_args()

    cfg = load_config(args.config)
    arena_root = resolve_path(cfg, cfg["arena"]["python_path"])
    entries = discover(arena_root)
    if not entries:
        raise SystemExit(f"no Arena-1 example strategies under {arena_root}")

    opponents_all = [e for e in entries if e.mode == "team"]
    adaptive_all = [e for e in entries if e.mode == "adaptive"]
    pool_entries, held_out = split_held_out(opponents_all, fraction=0.25)
    teammate_pool = TeammatePool(adaptive_all)
    if not len(teammate_pool):
        raise SystemExit("no adaptive teammate strategies found")

    opponents = _select_opponents(pool_entries, held_out, args)
    teammates = _teammate_entries(teammate_pool, cfg["lineup"], args.mix, args.seed_base)

    checkpoint = None
    if args.checkpoint:
        checkpoint = load_policy(ROOT / args.checkpoint if not Path(args.checkpoint).is_absolute()
                                 else args.checkpoint, cfg, obs_dim(cfg))

    save_pool(
        args.pool_out,
        teammates=[entry for _, entry in teammates],
        opponents=opponents,
        held_out=held_out,
        arena_root=arena_root,
        meta={"mix": args.mix, "held_out_mode": bool(args.held_out)},
    )

    sides = [False] if args.no_swap else [False, True]
    started = time.time()
    cells: list[dict] = []
    for opponent in opponents:
        for offset in range(int(args.seeds)):
            seed = int(args.seed_base) + offset
            for swapped in sides:
                cell = _run_cell(cfg, checkpoint, opponent, teammates, seed, swapped, args)
                cells.append(cell)
                print(json.dumps({
                    "opponent": cell["opponent"],
                    "side": cell["side"],
                    "seed": cell["seed"],
                    "winner": cell["winner"],
                    "control": round(cell["control"], 2),
                    "wall_s": cell["wall_s"],
                }, ensure_ascii=False), flush=True)

    report = {
        "format": "sentry-rl-pool-exam-v1",
        "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "checkpoint": str(args.checkpoint),
        "checkpoint_sha256": _sha256(Path(args.checkpoint)) if args.checkpoint else "",
        "duration_s": int(args.duration),
        "seed_base": int(args.seed_base),
        "mix": args.mix,
        "held_out_mode": bool(args.held_out),
        "pool_digest": entries_digest([entry for _, entry in teammates] + opponents + held_out),
        "teammates": {str(slot): entry.key for slot, entry in teammates},
        "opponents": [e.key for e in opponents],
        "held_out": [e.key for e in held_out],
        "cells": cells,
        "aggregate": _aggregate(cells),
        "wall_s": round(time.time() - started, 1),
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    agg = report["aggregate"]
    print(json.dumps({
        "matches": agg["matches"],
        "win_rate": agg["win_rate"],
        "control_mean": agg["control_mean"],
        "by_side": {side: b["win_rate"] for side, b in agg["by_side"].items()},
        "report": str(out),
        "wall_s": report["wall_s"],
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
