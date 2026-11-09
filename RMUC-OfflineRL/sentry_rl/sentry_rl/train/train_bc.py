"""CLI: behaviour cloning on a collected ``.npz`` dataset."""
from __future__ import annotations

import argparse
from pathlib import Path

from ..algos.bc import train_bc
from ..config import load_config, obs_dim, resolve_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/sentry_2027.yaml")
    parser.add_argument("--data", required=True)
    parser.add_argument("--out", default="checkpoints/bc.pt")
    parser.add_argument("--metrics", default="runs/bc_metrics.jsonl")
    parser.add_argument("--epochs", type=int, default=0, help="override bc.epochs")
    parser.add_argument("--batch-size", type=int, default=0, help="override bc.batch_size")
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.epochs:
        cfg["bc"]["epochs"] = args.epochs
    if args.batch_size:
        cfg["bc"]["batch_size"] = args.batch_size
    cfg["_obs_dim"] = obs_dim(cfg)
    train_bc(cfg,
             resolve_path(cfg, args.data),
             resolve_path(cfg, args.out),
             resolve_path(cfg, args.metrics) if args.metrics else None)


if __name__ == "__main__":
    main()
