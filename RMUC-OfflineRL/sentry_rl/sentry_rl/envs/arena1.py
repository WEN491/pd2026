"""Arena-1 adapter: drive one 2027 sentry through 5 s macro decisions.

The real ``rmuc_arena`` package is optional at import time.  Unit tests use
``MockSentryEnv``; this adapter is exercised once Arena-1 is built and on
``PYTHONPATH`` (see README).

Sides are not hard-coded: ``cfg["team"]`` names the sentry's side and the
opposite side is derived from it, so an evaluation can swap sides.  Teammates
(every slot except the sentry) and the opponent can be single policies or drawn
from :mod:`sentry_rl.pools`.
"""
from __future__ import annotations

import importlib.util
import os
import random
import sys
from pathlib import Path
from typing import Any, Callable

import numpy as np

from ..actions import N_ACTIONS, Intent, compute_mask
from ..config import load_config, resolve_path
from ..obs import build_obs
from ..parse import other_team
from ..radar import RadarChannel
from ..reward import RewardTracker
from .executor import MacroExecutor


class StopPolicy:
    """Empty actions: robots coast/stop.  Smoke tests only."""

    def act(self, observation: dict, state: Any) -> tuple[dict, Any]:
        return {}, state


class ModulePolicy:
    """Load an Arena-1 ``strategy.py`` and call its ``act`` at 10 Hz."""

    def __init__(self, path: str | Path, seed: int = 0, strict: bool = False):
        path = Path(path)
        self.source = path if path.is_file() else path / "strategy.py"
        self.path = self.source.parent
        self.strict = strict
        # Arena-1 example strategies do `from sdk import ...`; the real runner
        # mounts sdk.py next to strategy.py, so replicate that here.
        repo = Path(__file__).resolve().parents[3]
        for candidate in (repo / "Arena-1", repo / "Arena-1" / "rmuc_arena"):
            if candidate.is_dir() and str(candidate) not in sys.path:
                sys.path.insert(0, str(candidate))
        self.state: Any = None
        spec = importlib.util.spec_from_file_location(
            f"arena_strategy_{abs(hash(str(self.source)))}", self.source
        )
        if spec is None or spec.loader is None:
            raise FileNotFoundError(f"no strategy source at {self.source}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        if not callable(getattr(module, "act", None)):
            raise ValueError(f"{self.source} does not define act()")
        self.module = module
        self.seed = seed

    def act(self, observation: dict, state: Any = None) -> tuple[dict, Any]:
        try:
            actions, new_state = self.module.act(observation, self.state)
            if not isinstance(actions, dict):
                raise TypeError("act() must return a dict keyed by slot")
            self.state = new_state
            return actions, new_state
        except Exception:
            if self.strict:
                raise
            return {}, self.state


class Arena1SentryEnv:
    def __init__(
        self,
        cfg: dict | None = None,
        seed: int = 0,
        sentry_policy: Callable[[dict, Any], tuple[dict, Any]] | None = None,
        red_teammates: Callable[[dict, Any], tuple[dict, Any]] | None = None,
        blue_policy: Callable[[dict, Any], tuple[dict, Any]] | None = None,
        radar_mode: str | None = None,
        teammate_pool: Any = None,
        opponent_pool: Any = None,
        teammate_entries: list | None = None,
        opponent_entry: Any = None,
        pool_mix: str = "hetero",
        pool_seed: int = 0,
    ):
        self.cfg = cfg or load_config()
        self.seed = int(seed)
        self.team = str(self.cfg.get("team", "red"))
        self.opponent = other_team(self.team)
        self.own_slot = str(self.cfg.get("own_slot", "7"))
        self.teammate_slots = [int(slot) for slot in self.cfg["lineup"] if str(slot) != self.own_slot]

        self.sentry_policy = sentry_policy
        self.teammate_policy = red_teammates or StopPolicy()
        self.opponent_policy = blue_policy or StopPolicy()
        self.teammate_pool = teammate_pool
        self.opponent_pool = opponent_pool
        self.teammate_entries = list(teammate_entries or [])
        self.opponent_entry = opponent_entry
        self.pool_mix = str(pool_mix)
        self.pool_seed = int(pool_seed)
        self.teammates: dict[int, ModulePolicy] | None = None
        self.composition: dict[str, Any] = {}
        self._episode = -1

        self.radar_mode = radar_mode or self.cfg["radar"]["mode"]
        self.max_decisions = int(self.cfg["decision"]["max_decisions"])
        self._match = None
        self._executor = None
        self._ensure_arena_path()
        self.reset(seed)

    # --------------------------------------------------------------- arena setup
    def _ensure_arena_path(self) -> None:
        raw = self.cfg.get("arena", {}).get("python_path", "")
        if raw:
            path = resolve_path(self.cfg, raw)
            if path.is_dir() and str(path) not in sys.path:
                sys.path.insert(0, str(path))
        binary = self.cfg.get("arena", {}).get("sim_binary", "")
        if binary:
            os.environ["RMUC_SIM_BINARY"] = str(resolve_path(self.cfg, binary))

    def _import_arena(self):
        from rmuc_arena.engine import Match
        from rmuc_arena.models import mirrored_lineups
        return Match, mirrored_lineups

    # ------------------------------------------------------------------- pools
    def _sample_composition(self) -> None:
        """Pick the teammate/opponent strategies for the coming episode."""
        if self.teammate_entries or self.teammate_pool is not None:
            rng = random.Random(self.pool_seed * 1_000_003 + self._episode)
            if self.teammate_entries:
                entries = {int(slot): entry for slot, entry in self.teammate_entries}
            else:
                picked = self.teammate_pool.sample(self.cfg["lineup"], rng, mix=self.pool_mix)
                entries = {int(slot): entry for slot, entry in picked.items()}
            self.teammates = {slot: ModulePolicy(entry.source) for slot, entry in entries.items()}
            self.composition["teammates"] = {
                str(slot): entry.key for slot, entry in sorted(entries.items())
            }
        else:
            self.teammates = None
            self.composition.pop("teammates", None)

        if self.opponent_entry is not None:
            self.opponent_policy = ModulePolicy(self.opponent_entry.source)
            self.composition["opponent"] = self.opponent_entry.key
        elif self.opponent_pool is not None:
            rng = random.Random(self.pool_seed * 7_919 + self._episode)
            entry = self.opponent_pool.sample(rng)
            self.opponent_policy = ModulePolicy(entry.source)
            self.composition["opponent"] = entry.key
        else:
            self.composition.pop("opponent", None)

    # ------------------------------------------------------------------- reset
    def reset(self, seed: int | None = None) -> np.ndarray:
        Match, mirrored_lineups = self._import_arena()
        if self._match is not None:
            self._match.close()
        self.seed = int(seed if seed is not None else self.seed)
        self._episode += 1
        lineup = {str(slot): int(model) for slot, model in self.cfg["lineup"].items()}
        lineups = mirrored_lineups(lineup, "2027")
        self._match = Match(
            seed=self.seed,
            duration=int(self.cfg["arena"]["duration_s"]),
            map_name=str(self.cfg["arena"]["map"]),
            lineups=lineups,
        )
        self._executor = MacroExecutor(self.cfg)
        cfg_radar = dict(self.cfg["radar"])
        cfg_radar["mode"] = self.radar_mode
        self.radar = RadarChannel(cfg_radar, self.team, self.seed)
        self.reward_tracker = RewardTracker(self.cfg)
        self._sample_composition()
        self.decision = 0
        self._last_action = 0
        self.time = 0.0
        self._frame = self._match.snapshot()
        self.reward_tracker.reset(self._frame)
        self.radar.update(self._frame, self.time)
        self._observation = self._match.observation(self.team, [int(self.own_slot)])
        return self.obs()

    # --------------------------------------------------------------------- obs
    def obs(self) -> np.ndarray:
        return build_obs(self._observation, self.radar.measure(self.time), self.cfg)

    def action_mask(self) -> np.ndarray:
        return compute_mask(self._observation, self.cfg)

    # -------------------------------------------------------------------- step
    def _teammate_actions(self) -> dict[str, dict]:
        observation = self._match.observation(self.team, self.teammate_slots)
        if self.teammates:
            merged: dict[str, dict] = {}
            for slot, policy in self.teammates.items():
                actions, _ = policy.act(observation, None)
                command = (actions or {}).get(str(slot)) or (actions or {}).get(slot)
                if command:
                    merged[str(slot)] = command
            return merged
        actions, _ = self.teammate_policy.act(observation, None)
        return dict(actions or {})

    def _opponent_actions(self) -> dict[str, dict]:
        observation = self._match.observation(self.opponent, None)
        actions, _ = self.opponent_policy.act(observation, None)
        return dict(actions or {})

    def _sentry_action(self, tick: int) -> dict:
        if self.sentry_policy is not None:
            observation = self._match.observation(self.team, [int(self.own_slot)])
            actions, _ = self.sentry_policy.act(observation, None)
            actions = actions or {}
            return dict(actions.get(str(self.own_slot), actions.get(int(self.own_slot), {})))
        return self._executor.command(self._last_action, self._observation, tick)

    def step_macro(self, action: int) -> tuple[np.ndarray, float, bool, dict[str, Any]]:
        action = int(action)
        if not 0 <= action < N_ACTIONS:
            raise ValueError(f"action out of range: {action}")
        if not self.action_mask()[action]:
            raise ValueError(f"masked action {action} ({Intent(action).name})")
        self._last_action = action

        commands: dict[str, dict] = {self.team: {}, self.opponent: {}}
        for tick in range(100):  # 100 * 0.05 s = 5 s
            if tick % 2 == 0:  # 10 Hz policy tick
                team_commands = self._teammate_actions()
                team_commands.pop(self.own_slot, None)
                team_commands[self.own_slot] = self._sentry_action(tick)
                commands = {self.team: team_commands, self.opponent: self._opponent_actions()}
            self._frame = self._match.step(commands)
        self.time += float(self.cfg["decision"]["interval_s"])
        self.decision += 1
        self._observation = self._match.observation(self.team, [int(self.own_slot)])
        self.radar.update(self._frame, float(self._frame.get("time", self.time)))
        reward = self.reward_tracker.update(self._frame)
        done = bool(self._match.status == "finished" or self.decision >= self.max_decisions)
        return self.obs(), reward, done, self.info()

    # -------------------------------------------------------------------- info
    def sentry_hp(self) -> float:
        for robot in self._frame.get("robots", []) or []:
            if str(robot.get("slot")) == self.own_slot and robot.get("team") == self.team:
                return float(robot.get("hp", 0.0))
        return 0.0

    def info(self) -> dict[str, Any]:
        teams = (self._frame.get("teams") or {})
        return {
            "time": self._frame.get("time", self.time),
            "decisions": self.decision,
            "control": float((teams.get(self.team) or {}).get("control", 0.0)),
            "winner": self._frame.get("winner", ""),
            "status": self._frame.get("status", ""),
            "reason": self._frame.get("reason", ""),
            "sentry_hp": self.sentry_hp(),
        }

    @property
    def observation(self) -> dict:
        return self._observation

    @property
    def frame(self) -> dict:
        return self._frame

    def close(self) -> None:
        self.teammates = None
        if self._match is not None:
            self._match.close()
            self._match = None
