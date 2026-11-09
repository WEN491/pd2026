"""Policy / value network: a small MLP over the 7 macro intents."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import torch
import torch.nn as nn
from torch.distributions import Categorical

NEG_INF = -1.0e9


class SentryPolicy(nn.Module):
    def __init__(self, obs_dim: int, n_actions: int = 7, hidden: int = 256, depth: int = 2):
        super().__init__()
        layers: list[nn.Module] = []
        last = obs_dim
        for _ in range(max(1, depth)):
            layers += [nn.Linear(last, hidden), nn.ReLU()]
            last = hidden
        self.trunk = nn.Sequential(*layers)
        self.pi = nn.Linear(last, n_actions)
        self.v = nn.Linear(last, 1)
        self.obs_dim = int(obs_dim)
        self.n_actions = int(n_actions)

    def forward(self, obs: torch.Tensor, mask: torch.Tensor | None = None):
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)
        h = self.trunk(obs)
        logits = self.pi(h)
        if mask is not None:
            mask = mask.to(device=logits.device, dtype=torch.bool)
            if mask.dim() == 1:
                mask = mask.unsqueeze(0)
            logits = logits.masked_fill(~mask, NEG_INF)
        return Categorical(logits=logits), self.v(h).squeeze(-1)

    @torch.no_grad()
    def act(self, obs: torch.Tensor, mask: torch.Tensor | None = None,
            deterministic: bool = False):
        dist, value = self.forward(obs, mask)
        action = torch.argmax(dist.probs, dim=-1) if deterministic else dist.sample()
        return action, dist.log_prob(action), value

    def n_parameters(self) -> int:
        return sum(p.numel() for p in self.parameters())


def build_policy(cfg: dict, obs_dim: int) -> SentryPolicy:
    model_cfg = cfg.get("model", {})
    return SentryPolicy(obs_dim, n_actions=7,
                        hidden=int(model_cfg.get("hidden", 256)),
                        depth=int(model_cfg.get("depth", 2)))


def save_policy(policy: SentryPolicy, path: str | Path, extra: dict | None = None) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"model": policy.state_dict(), "obs_dim": policy.obs_dim,
                "n_actions": policy.n_actions, "extra": extra or {}}, path)


def load_policy(path: str | Path, cfg: dict | None = None, obs_dim: int | None = None) -> SentryPolicy:
    payload = torch.load(Path(path), map_location="cpu")
    obs_dim = int(obs_dim or payload.get("obs_dim") or 59)
    policy = build_policy(cfg or {}, obs_dim)
    policy.load_state_dict(payload["model"])
    policy.eval()
    return policy
