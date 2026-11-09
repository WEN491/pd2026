"""Per-decision reward for the sentry macro policy.

Reward is accumulated over the 5 s a macro is executed.  The primary term is
the team's control-degree delta; structure damage, damage exchange and a
potential over capture progress give the policy a denser signal without
replacing the objective.
"""
from __future__ import annotations

from . import parse


class RewardTracker:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.weights = cfg["reward"]
        self.team = str(cfg.get("team", "red"))
        self.enemy = parse.other_team(self.team)
        self._prev_control = 0.0
        self._prev_hp: dict[str, float] = {}
        self._prev_struct: dict[tuple[str, str], float] = {}
        self._prev_progress = 0.0
        self._prev_time = 0.0

    @staticmethod
    def _hp_map(frame: dict) -> dict[str, float]:
        out: dict[str, float] = {}
        for robot in frame.get("robots", []) or []:
            rid = str(robot.get("id", robot.get("slot", "")))
            if rid:
                out[rid] = parse.as_float(robot.get("hp"))
        return out

    @staticmethod
    def _struct_map(frame: dict) -> dict[tuple[str, str], float]:
        out: dict[tuple[str, str], float] = {}
        for kind in ("outposts", "bases"):
            for side in ("red", "blue"):
                item = (frame.get(kind) or {}).get(side) or {}
                out[(kind, side)] = parse.as_float(item.get("hp"))
        return out

    @staticmethod
    def _progress(frame: dict) -> float:
        return sum(parse.as_float(item.get("progress"))
                   for item in (frame.get("positions") or {}).values())

    def reset(self, frame: dict) -> None:
        self._prev_control = parse.as_float((frame.get("teams") or {}).get(self.team, {}).get("control"))
        self._prev_hp = self._hp_map(frame)
        self._prev_struct = self._struct_map(frame)
        self._prev_progress = self._progress(frame)
        self._prev_time = parse.as_float(frame.get("time"))

    def update(self, frame: dict) -> float:
        w = self.weights
        control = parse.as_float((frame.get("teams") or {}).get(self.team, {}).get("control"))
        dc = control - self._prev_control

        hp = self._hp_map(frame)
        dealt = sum(max(0.0, self._prev_hp.get(rid, cur) - cur)
                    for rid, cur in hp.items() if self._team_of(frame, rid) == self.enemy)
        taken = sum(max(0.0, self._prev_hp.get(rid, cur) - cur)
                    for rid, cur in hp.items() if self._team_of(frame, rid) == self.team)

        structures = self._struct_map(frame)
        struct_damage = sum(max(0.0, self._prev_struct.get(key, cur) - cur)
                            for key, cur in structures.items()
                            if key[1] == self.enemy)

        progress = self._progress(frame)
        dphi = progress - self._prev_progress

        reward = (
            w["control"] * dc
            + w["structure"] * struct_damage
            + w["damage_dealt"] * dealt
            - w["damage_taken"] * taken
            + w["hit"] * (dealt / 20.0)  # 17 mm hit = 20 HP
            + w["potential"] * dphi
        )

        enemy_base = parse.as_float(((frame.get("bases") or {}).get(self.enemy) or {}).get("hp"))
        own_base = parse.as_float(((frame.get("bases") or {}).get(self.team) or {}).get("hp"))
        if enemy_base <= 0.0:
            reward += w["base_destroyed"]
        if own_base <= 0.0:
            reward -= w["base_destroyed"]
        status = frame.get("status")
        winner = frame.get("winner")
        if status == "finished" and winner:
            reward += w["win"] if winner == self.team else -w["lose"]

        self._prev_control = control
        self._prev_hp = hp
        self._prev_struct = structures
        self._prev_progress = progress
        self._prev_time = parse.as_float(frame.get("time"))
        return float(reward)

    def _team_of(self, frame: dict, rid: str) -> str:
        for robot in frame.get("robots", []) or []:
            if str(robot.get("id", robot.get("slot", ""))) == rid:
                return str(robot.get("team", ""))
        return ""
