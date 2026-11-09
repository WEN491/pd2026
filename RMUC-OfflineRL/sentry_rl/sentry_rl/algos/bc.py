"""Behaviour cloning over the 7 macro intents.

Dataset format (``.npz``)::

    obs     float32 [N, obs_dim]
    mask    bool    [N, 7]
    action  int64   [N]

The dataset is produced by ``sentry_rl.data.collect``; labels are inferred from
the scripted sentry's state (see ``data/labeler.py``).
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from ..config import obs_dim as configured_obs_dim
from ..models.policy import SentryPolicy, build_policy
from ..utils.logging import MetricLogger
from ..utils.seeding import seed_everything


def _device(cfg: dict) -> torch.device:
    requested = str(cfg.get("device", "cpu"))
    if requested.startswith("cuda") and not torch.cuda.is_available():
        return torch.device("cpu")
    return torch.device(requested)


def load_npz(path: str | Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = np.load(Path(path))
    obs = np.asarray(data["obs"], dtype=np.float32)
    actions = np.asarray(data["action"], dtype=np.int64)
    if "mask" in data:
        mask = np.asarray(data["mask"], dtype=bool)
    else:
        mask = np.ones((obs.shape[0], 7), dtype=bool)
    if obs.ndim != 2 or mask.shape != (obs.shape[0], 7) or actions.shape != (obs.shape[0],):
        raise ValueError("bad dataset shapes")
    return obs, mask, actions


def class_weights(actions: np.ndarray, n_actions: int = 7) -> np.ndarray:
    counts = np.bincount(actions, minlength=n_actions).astype(np.float64)
    total = max(1.0, counts.sum())
    weights = np.where(counts > 0, total / (n_actions * np.maximum(counts, 1.0)), 0.0)
    return weights.astype(np.float32)


def train_bc(cfg: dict, data_path: str | Path, out_path: str | Path,
             metrics_path: str | Path | None = None) -> SentryPolicy:
    seed_everything(int(cfg.get("bc", {}).get("seed", cfg.get("seed", 42))))
    device = _device(cfg)
    obs, mask, actions = load_npz(data_path)
    expected = configured_obs_dim(cfg)
    if obs.shape[1] != expected:
        raise ValueError(f"dataset obs_dim {obs.shape[1]} != config {expected}")

    rng = np.random.default_rng(int(cfg.get("seed", 42)))
    order = rng.permutation(obs.shape[0])
    split = max(1, int(0.9 * len(order)))
    train_idx, val_idx = order[:split], order[split:]
    if len(val_idx) == 0:
        val_idx = train_idx[:1]

    weights = class_weights(actions, 7)
    if cfg.get("bc", {}).get("class_weights") == "none":
        weights = None
    weight_t = None if weights is None else torch.tensor(weights, device=device)
    # Masked logits use -1e9.  Label smoothing would put mass on those masked
    # classes and blow the loss up by ~1e9 * smoothing / n_classes.  Disable
    # smoothing whenever a mask is present; the mask already regularises.
    smoothing = float(cfg["bc"].get("label_smoothing", 0.0))
    if smoothing > 0.0:
        smoothing = 0.0
    loss_fn = nn.CrossEntropyLoss(weight=weight_t, label_smoothing=smoothing)

    def loader(indices: np.ndarray, shuffle: bool) -> DataLoader:
        ds = TensorDataset(torch.from_numpy(obs[indices]), torch.from_numpy(mask[indices]),
                           torch.from_numpy(actions[indices]))
        return DataLoader(ds, batch_size=int(cfg["bc"]["batch_size"]), shuffle=shuffle)

    train_loader = loader(train_idx, True)
    val_loader = loader(val_idx, False)
    policy = build_policy(cfg, obs.shape[1]).to(device)
    optim = torch.optim.AdamW(policy.parameters(), lr=float(cfg["bc"]["lr"]))
    epochs = int(cfg["bc"]["epochs"])
    logger = MetricLogger(metrics_path) if metrics_path else None
    best_acc, best_state = -1.0, None

    for epoch in range(1, epochs + 1):
        policy.train()
        total_loss, seen = 0.0, 0
        for batch_obs, batch_mask, batch_action in train_loader:
            batch_obs = batch_obs.to(device)
            batch_mask = batch_mask.to(device)
            batch_action = batch_action.to(device)
            logits = policy.trunk(batch_obs)
            logits = policy.pi(logits).masked_fill(~batch_mask, -1.0e9)
            loss = loss_fn(logits, batch_action)
            optim.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
            optim.step()
            total_loss += float(loss) * batch_obs.shape[0]
            seen += batch_obs.shape[0]

        policy.eval()
        correct, total = 0, 0
        with torch.no_grad():
            for batch_obs, batch_mask, batch_action in val_loader:
                batch_obs = batch_obs.to(device)
                batch_mask = batch_mask.to(device)
                logits = policy.pi(policy.trunk(batch_obs)).masked_fill(~batch_mask, -1.0e9)
                pred = torch.argmax(logits, dim=-1)
                correct += int((pred == batch_action.to(device)).sum())
                total += int(batch_action.numel())
        acc = correct / max(1, total)
        if logger:
            logger.log(epoch, bc_loss=total_loss / max(1, seen), val_acc=acc)
        print(f"epoch {epoch:3d}  loss={total_loss / max(1, seen):.4f}  val_acc={acc:.4f}")
        if acc >= best_acc:
            best_acc = acc
            best_state = {key: value.detach().cpu().clone() for key, value in policy.state_dict().items()}

    if best_state is not None:
        policy.load_state_dict(best_state)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"model": policy.state_dict(), "obs_dim": obs.shape[1], "n_actions": 7,
                "extra": {"val_acc": best_acc, "class_weights": None if weights is None else weights.tolist()}},
               out_path)
    print(f"saved {out_path} (val_acc={best_acc:.4f})")
    return policy
