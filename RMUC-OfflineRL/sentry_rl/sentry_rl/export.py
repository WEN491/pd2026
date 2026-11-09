"""Export the policy for deployment (TorchScript; ONNX optional)."""
from __future__ import annotations

import argparse
from pathlib import Path

import torch
import torch.nn as nn

from .config import load_config, obs_dim, resolve_path
from .models.policy import load_policy


class ExportWrapper(nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, obs: torch.Tensor, mask: torch.Tensor):
        h = self.policy.trunk(obs)
        logits = self.policy.pi(h).masked_fill(~mask, -1.0e9)
        return logits, self.policy.v(h).squeeze(-1)


def export(cfg: dict, checkpoint: str, out: str, onnx: bool = False) -> None:
    policy = load_policy(resolve_path(cfg, checkpoint), cfg, obs_dim(cfg))
    wrapper = ExportWrapper(policy).eval()
    dummy_obs = torch.zeros(1, obs_dim(cfg), dtype=torch.float32)
    dummy_mask = torch.ones(1, 7, dtype=torch.bool)
    traced = torch.jit.trace(wrapper, (dummy_obs, dummy_mask))
    out_path = resolve_path(cfg, out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    traced.save(str(out_path))
    print(f"saved TorchScript: {out_path}")
    if onnx:
        try:
            torch.onnx.export(wrapper, (dummy_obs, dummy_mask), str(out_path.with_suffix(".onnx")),
                              input_names=["obs", "mask"], output_names=["logits", "value"],
                              opset_version=17)
            print(f"saved ONNX: {out_path.with_suffix('.onnx')}")
        except Exception as exc:  # onnx is optional
            print(f"ONNX export skipped: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/sentry_2027.yaml")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--out", default="checkpoints/sentry_policy.ts")
    parser.add_argument("--onnx", action="store_true")
    args = parser.parse_args()
    cfg = load_config(args.config)
    export(cfg, args.checkpoint, args.out, args.onnx)


if __name__ == "__main__":
    main()
