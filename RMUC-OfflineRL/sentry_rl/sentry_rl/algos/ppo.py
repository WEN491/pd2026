"""PPO over the 7 macro intents (masked categorical, 5 s decisions).

Kept deliberately small: a CleanRL-style loop with GAE, minibatch epochs and
optional BC initialisation.  The environment interface is::

    obs = env.reset()
    obs, reward, done, info = env.step_macro(action)
    mask = env.action_mask()
"""
from __future__ import annotations

from pathlib import Path
from typing import Callable

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Categorical

from ..models.policy import SentryPolicy, build_policy, load_policy
from ..utils.logging import MetricLogger
from ..utils.seeding import seed_everything


def _device(cfg: dict) -> torch.device:
    requested = str(cfg.get("device", "cpu"))
    if requested.startswith("cuda") and not torch.cuda.is_available():
        return torch.device("cpu")
    return torch.device(requested)


def _gae(rewards: list[float], values: list[float], dones: list[bool],
         last_value: float, gamma: float, lam: float) -> tuple[np.ndarray, np.ndarray]:
    advantages = np.zeros(len(rewards), dtype=np.float32)
    gae = 0.0
    for t in reversed(range(len(rewards))):
        nonterminal = 0.0 if dones[t] else 1.0
        next_value = last_value if t == len(rewards) - 1 else values[t + 1]
        delta = rewards[t] + gamma * next_value * nonterminal - values[t]
        gae = delta + gamma * lam * nonterminal * gae
        advantages[t] = gae
    returns = advantages + np.asarray(values, dtype=np.float32)
    return advantages, returns


class PPO:
    def __init__(self, cfg: dict, policy: SentryPolicy, device: torch.device):
        self.cfg = cfg
        self.policy = policy.to(device)
        self.device = device
        p = cfg["ppo"]
        self.optim = torch.optim.AdamW(policy.parameters(), lr=float(p["lr"]))
        self.gamma = float(p["gamma"])
        self.lam = float(p["lam"])
        self.clip = float(p["clip"])
        self.entropy_coef = float(p["entropy"])
        self.value_coef = float(p["value_coef"])
        self.max_grad = float(p["max_grad_norm"])
        self.epochs = int(p["epochs"])
        self.batch_size = int(p["batch_size"])

    # ------------------------------------------------------------------ rollout
    def collect(self, envs: list, obs: list[np.ndarray], rollout: int) -> dict:
        n = len(envs)
        data = {key: [[] for _ in range(n)] for key in
                ("obs", "mask", "act", "logp", "val", "rew", "done")}
        for _ in range(rollout):
            for i, env in enumerate(envs):
                mask_np = env.action_mask()
                obs_t = torch.as_tensor(obs[i], dtype=torch.float32, device=self.device).unsqueeze(0)
                mask_t = torch.as_tensor(mask_np, dtype=torch.bool, device=self.device).unsqueeze(0)
                with torch.no_grad():
                    dist, value = self.policy(obs_t, mask_t)
                    action = dist.sample()
                    logp = dist.log_prob(action)
                next_obs, reward, done, _ = env.step_macro(int(action.item()))
                data["obs"][i].append(obs[i])
                data["mask"][i].append(mask_np)
                data["act"][i].append(int(action.item()))
                data["logp"][i].append(float(logp.item()))
                data["val"][i].append(float(value.item()))
                data["rew"][i].append(float(reward))
                data["done"][i].append(bool(done))
                obs[i] = next_obs if not done else env.reset()

        all_obs, all_mask, all_act, all_logp = [], [], [], []
        all_adv, all_ret, all_val, all_rew = [], [], [], []
        for i, env in enumerate(envs):
            last_obs = torch.as_tensor(obs[i], dtype=torch.float32, device=self.device).unsqueeze(0)
            last_mask = torch.as_tensor(env.action_mask(), dtype=torch.bool, device=self.device).unsqueeze(0)
            with torch.no_grad():
                _, last_value = self.policy(last_obs, last_mask)
            adv, ret = _gae(data["rew"][i], data["val"][i], data["done"][i],
                            float(last_value.item()), self.gamma, self.lam)
            all_obs.extend(data["obs"][i])
            all_mask.extend(data["mask"][i])
            all_act.extend(data["act"][i])
            all_logp.extend(data["logp"][i])
            all_adv.extend(adv.tolist())
            all_ret.extend(ret.tolist())
            all_val.extend(data["val"][i])
            all_rew.extend(data["rew"][i])
        return {"obs": np.asarray(all_obs, dtype=np.float32),
                "mask": np.asarray(all_mask, dtype=bool),
                "act": np.asarray(all_act, dtype=np.int64),
                "logp": np.asarray(all_logp, dtype=np.float32),
                "adv": np.asarray(all_adv, dtype=np.float32),
                "ret": np.asarray(all_ret, dtype=np.float32),
                "val": np.asarray(all_val, dtype=np.float32),
                "rew": np.asarray(all_rew, dtype=np.float32)}

    # ------------------------------------------------------------------- update
    def update(self, batch: dict) -> dict[str, float]:
        obs = torch.as_tensor(batch["obs"], dtype=torch.float32, device=self.device)
        mask = torch.as_tensor(batch["mask"], dtype=torch.bool, device=self.device)
        act = torch.as_tensor(batch["act"], dtype=torch.int64, device=self.device)
        old_logp = torch.as_tensor(batch["logp"], dtype=torch.float32, device=self.device)
        adv = torch.as_tensor(batch["adv"], dtype=torch.float32, device=self.device)
        ret = torch.as_tensor(batch["ret"], dtype=torch.float32, device=self.device)
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)

        n = obs.shape[0]
        stats = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0, "approx_kl": 0.0}
        count = 0
        for _ in range(self.epochs):
            order = torch.randperm(n, device=self.device)
            for start in range(0, n, self.batch_size):
                idx = order[start:start + self.batch_size]
                logits = self.policy.pi(self.policy.trunk(obs[idx])).masked_fill(~mask[idx], -1.0e9)
                dist = Categorical(logits=logits)
                new_logp = dist.log_prob(act[idx])
                log_ratio = new_logp - old_logp[idx]
                ratio = torch.exp(log_ratio)
                surr1 = ratio * adv[idx]
                surr2 = torch.clamp(ratio, 1.0 - self.clip, 1.0 + self.clip) * adv[idx]
                policy_loss = -torch.min(surr1, surr2).mean()
                value_pred = self.policy.v(self.policy.trunk(obs[idx])).squeeze(-1)
                value_loss = nn.functional.mse_loss(value_pred, ret[idx])
                entropy = dist.entropy().mean()
                loss = policy_loss + self.value_coef * value_loss - self.entropy_coef * entropy
                self.optim.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad)
                self.optim.step()

                stats["policy_loss"] += float(policy_loss.item())
                stats["value_loss"] += float(value_loss.item())
                stats["entropy"] += float(entropy.item())
                stats["approx_kl"] += float((old_logp[idx] - new_logp).mean().item())
                count += 1
        return {key: value / max(1, count) for key, value in stats.items()}


def train_ppo(cfg: dict, env_factory: Callable[[int], object], out_path: str | Path,
              init_path: str | Path | None = None, metrics_path: str | Path | None = None,
              obs_dim: int | None = None) -> SentryPolicy:
    seed_everything(int(cfg.get("seed", 42)))
    device = _device(cfg)
    if obs_dim is None:
        from ..config import obs_dim as configured_obs_dim
        obs_dim = configured_obs_dim(cfg)
    policy = load_policy(init_path, cfg, obs_dim) if init_path else build_policy(cfg, obs_dim)
    p = cfg["ppo"]
    n_envs = int(p["n_envs"])
    rollout = int(p["rollout_decisions"])
    total_steps = int(p["total_steps"])
    updates = max(1, total_steps // max(1, n_envs * rollout))

    envs = [env_factory(i) for i in range(n_envs)]
    obs = [env.reset() for env in envs]
    trainer = PPO(cfg, policy, device)
    logger = MetricLogger(metrics_path) if metrics_path else None
    step = 0

    for update in range(1, updates + 1):
        batch = trainer.collect(envs, obs, rollout)
        stats = trainer.update(batch)
        step += n_envs * rollout
        mean_ret = float(np.mean([np.sum(batch["rew"][i:i + rollout]) for i in range(0, len(batch["rew"]), rollout)])) if len(batch["rew"]) else 0.0
        if logger:
            logger.log(step, **stats, rollout_return=mean_ret)
        if update % 5 == 0 or update == updates:
            print(f"update {update:4d}  step {step:8d}  " +
                  "  ".join(f"{k}={v:.4f}" for k, v in stats.items()))

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"model": policy.state_dict(), "obs_dim": obs_dim, "n_actions": 7,
                "extra": {"updates": updates, "steps": step}}, out_path)
    print(f"saved {out_path}")
    return policy
