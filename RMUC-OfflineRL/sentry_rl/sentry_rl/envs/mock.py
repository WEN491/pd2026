"""A tiny synthetic environment that exercises the whole pipeline.

It is not a simulator: it exists so the observation builder, action mask,
reward tracker, BC/PPO loops and evaluation code can be tested without Rust,
ROS or torch.  ``Arena1SentryEnv`` is the real adapter.
"""
from __future__ import annotations

import random
from typing import Any

import numpy as np

from .. import parse
from ..actions import N_ACTIONS, Intent, compute_mask
from ..config import load_config
from ..obs import build_obs
from ..radar import RadarChannel
from ..reward import RewardTracker


class MockSentryEnv:
    def __init__(self, cfg: dict | None = None, seed: int = 0):
        self.cfg = cfg or load_config()
        self.seed = int(seed)
        self.random = random.Random(seed)
        self.radar = RadarChannel(self.cfg["radar"], self.cfg["team"], seed)
        self.reward_tracker = RewardTracker(self.cfg)
        self.max_decisions = int(self.cfg["decision"]["max_decisions"])
        self.reset(seed)

    # ------------------------------------------------------------------ state
    def _make_own(self) -> dict:
        return {
            "id": "red_7", "slot": 7, "team": "red", "role": "sentry",
            "x": -3.0, "y": 3.9, "z": 0.2, "hp": 400.0, "max_hp": 400.0,
            "ammo": 300, "heat": 0.0, "posture": "move", "posture_time": [0.0, 0.0, 0.0],
            "posture_enhanced_time": [0.0, 0.0, 0.0], "posture_ready_at": 0.0,
            "heading": 0.0, "alive": True, "weak": False, "invulnerable": False,
        }

    def _make_enemies(self) -> list[dict]:
        slots = [1, 3, 4, 7]
        out = []
        for i, slot in enumerate(slots):
            out.append({
                "id": f"blue_{slot}", "slot": slot, "team": "blue",
                "role": "sentry" if slot == 7 else ("heavy" if slot == 1 else "infantry"),
                "x": 1.0 + 2.0 * i, "y": -3.0 + 0.8 * i, "z": 0.2,
                "hp": 400.0, "max_hp": 400.0, "alive": True,
            })
        return out

    def _make_structures(self) -> tuple[dict, dict]:
        bases = {
            "red": {"hp": 10000.0, "max_hp": 10000.0, "owner": "red", "armor_open": False},
            "blue": {"hp": 10000.0, "max_hp": 10000.0, "owner": "blue", "armor_open": False},
        }
        outposts = {
            "red": {"hp": 1000.0, "max_hp": 1000.0, "owner": "red"},
            "blue": {"hp": 1000.0, "max_hp": 1000.0, "owner": "blue"},
        }
        return bases, outposts

    def _make_positions(self) -> dict:
        return {
            "red_outpost": {"controller": "red", "control": "full", "progress": 0.0},
            "blue_outpost": {"controller": "blue", "control": "full", "progress": 0.0},
            "red_base": {"controller": "red", "control": "full", "progress": 0.0},
            "blue_base": {"controller": "blue", "control": "full", "progress": 0.0},
        }

    def _refresh(self) -> None:
        self._observation = {
            "team": "red", "time": self.time, "remaining": max(0.0, 420.0 - self.time),
            "robots": [self.own] + self.enemies,  # mock: all own robots are the sentry
            "enemies": self.enemies,
            "referee": {"robots": [self.own] + self.enemies},
            "bases": self.bases, "outposts": self.outposts, "positions": self.positions,
            "teams": {"red": {"control": self.control}, "blue": {"control": self.enemy_control}},
            "arena": {},
        }
        self._frame = {
            "time": self.time, "status": "finished" if self.done_flag else "running",
            "winner": self.winner, "robots": [self.own] + self.enemies,
            "bases": self.bases, "outposts": self.outposts, "positions": self.positions,
            "teams": self._observation["teams"],
        }

    # ------------------------------------------------------------------- reset
    def reset(self, seed: int | None = None) -> np.ndarray:
        if seed is not None:
            self.seed = int(seed)
            self.random = random.Random(seed)
            self.radar = RadarChannel(self.cfg["radar"], self.cfg["team"], seed)
        self.time = 0.0
        self.control = 0.0
        self.enemy_control = 0.0
        self.decisions = 0
        self.done_flag = False
        self.winner = ""
        self.own = self._make_own()
        self.enemies = self._make_enemies()
        self.bases, self.outposts = self._make_structures()
        self.positions = self._make_positions()
        self._refresh()
        self.reward_tracker.reset(self._frame)
        self.radar.update(self._frame, self.time)
        return self.obs()

    # -------------------------------------------------------------------- obs
    def obs(self) -> np.ndarray:
        self.radar.update(self._frame, self.time)
        return build_obs(self._observation, self.radar.measure(self.time), self.cfg)

    def action_mask(self) -> np.ndarray:
        return compute_mask(self._observation, self.cfg)

    # ------------------------------------------------------------------- step
    def step_macro(self, action: int) -> tuple[np.ndarray, float, bool, dict[str, Any]]:
        action = int(action)
        if not 0 <= action < N_ACTIONS:
            raise ValueError(f"action out of range: {action}")
        if not self.action_mask()[action]:
            raise ValueError(f"masked action {action} ({Intent(action).name})")

        self.decisions += 1
        self.time += 5.0
        for enemy in self.enemies:
            enemy["x"] += self.random.uniform(-0.4, 0.4)
            enemy["y"] += self.random.uniform(-0.4, 0.4)
        self.own["hp"] = max(0.0, self.own["hp"] - self.random.uniform(0.0, 12.0))
        self.own["ammo"] = max(0, self.own["ammo"] - self.random.randint(0, 6))
        self.own["heat"] = max(0.0, self.own["heat"] + self.random.uniform(-2.0, 6.0))
        if self.own["hp"] <= 0.0:
            self.done_flag = True
            self.winner = "blue"

        if action in (Intent.HOLD_OUTPOST, Intent.HOLD_BASE):
            self.control += 0.4
        elif action in (Intent.TAKE_OUTPOST, Intent.TAKE_BASE):
            self.control += 0.2
        if self.time >= 420.0:
            self.done_flag = True
            self.winner = "red" if self.control >= self.enemy_control else "blue"
        self._refresh()
        reward = self.reward_tracker.update(self._frame)
        done = bool(self.done_flag or self.decisions >= self.max_decisions)
        return self.obs(), reward, done, self.info()

    def info(self) -> dict[str, Any]:
        return {
            "time": self.time,
            "decisions": self.decisions,
            "control": self.control,
            "winner": self.winner,
            "own_hp": self.own["hp"],
        }

    @property
    def observation(self) -> dict:
        return self._observation

    @property
    def frame(self) -> dict:
        return self._frame


def _main() -> None:
    env = MockSentryEnv(seed=0)
    obs = env.reset()
    total = 0.0
    done = False
    while not done:
        mask = env.action_mask()
        action = int(np.flatnonzero(mask)[0])
        obs, reward, done, info = env.step_macro(action)
        total += reward
    print({"return": round(total, 3), **info, "obs_dim": int(obs.size)})


if __name__ == "__main__":
    _main()
