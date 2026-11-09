"""Defensive helpers for Arena-1 observation dicts.

Arena-1's observation schema is documented in
``Arena-1/docs/agent_guides/en/2027/robot-7.md``.  Keep every access here so a
schema change is a one-file fix, not a hunt through the training code.
"""
from __future__ import annotations

import math
from typing import Any, Iterable


def other_team(team: str) -> str:
    return "blue" if team == "red" else "red"


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def find_robot(robots: Iterable[dict] | None, slot: str | int) -> dict:
    """Find one robot by slot in a list of Arena-1 robot dicts."""
    want = int(slot)
    for robot in robots or []:
        try:
            if int(robot.get("slot", -1)) == want:
                return robot
        except (TypeError, ValueError):
            continue
    return {}


def own_robot(observation: dict, slot: str | int = "7") -> dict:
    return find_robot(observation.get("robots", []), slot)


def enemy_team(observation: dict) -> str:
    return other_team(str(observation.get("team", "red")))


def hp_ratio(robot: dict) -> float:
    max_hp = as_float(robot.get("max_hp"), 0.0)
    if max_hp <= 0:
        return 0.0
    return max(0.0, min(1.0, as_float(robot.get("hp")) / max_hp))


def ammo_ratio(robot: dict, full: float = 300.0) -> float:
    return max(0.0, min(1.0, as_float(robot.get("ammo")) / full))


def visible_enemies(observation: dict) -> list[dict]:
    return list(observation.get("enemies", []) or [])


def enemy_public(observation: dict) -> dict[str, dict]:
    """Enemy slot -> public referee record (hp/alive/role, no position)."""
    team = enemy_team(observation)
    out: dict[str, dict] = {}
    for robot in observation.get("referee", {}).get("robots", []) or []:
        if robot.get("team") == team:
            out[str(robot.get("slot"))] = robot
    return out


def position(observation: dict, key: str) -> dict:
    return dict((observation.get("positions") or {}).get(key) or {})


def structure(observation: dict, kind: str, team: str) -> dict:
    """``kind`` is ``bases`` or ``outposts``."""
    return dict((observation.get(kind) or {}).get(team) or {})


def distance(a: dict, b: dict) -> float:
    return math.hypot(as_float(a.get("x")) - as_float(b.get("x")),
                      as_float(a.get("y")) - as_float(b.get("y")))


def nearest_visible(observation: dict) -> tuple[dict | None, float]:
    own = own_robot(observation)
    best, best_d = None, float("inf")
    for enemy in visible_enemies(observation):
        d = distance(own, enemy)
        if d < best_d:
            best, best_d = enemy, d
    return best, best_d


def posture_index(robot: dict) -> int:
    return {"attack": 0, "defense": 1, "move": 2}.get(str(robot.get("posture", "move")), 2)


def posture_enhanced_remaining(robot: dict) -> float:
    """Fraction of the 15 s enhancement budget left (0..1).

    Arena-1 publishes ``posture_enhanced_time``; accept either a scalar (seconds
    used) or a per-posture list.
    """
    raw = robot.get("posture_enhanced_time", 0.0)
    idx = posture_index(robot)
    if isinstance(raw, (list, tuple)):
        used = as_float(raw[idx]) if idx < len(raw) else 0.0
    else:
        used = as_float(raw)
    return max(0.0, min(1.0, 1.0 - used / 15.0))
