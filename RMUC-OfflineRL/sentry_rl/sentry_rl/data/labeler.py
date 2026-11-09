"""Heuristic intent labels for behaviour cloning.

A scripted sentry does not expose its intent, so we infer it from where it is
and what it is doing.  This is deliberately simple; inspect the confusion
matrix before trusting it.  The label is only a BC prior, not a ground truth.
"""
from __future__ import annotations

import math

from .. import parse
from ..actions import Intent
from ..config import goal

GOAL_TO_INTENT = {
    "hold_outpost": Intent.HOLD_OUTPOST,
    "hold_base": Intent.HOLD_BASE,
    "take_outpost": Intent.TAKE_OUTPOST,
    "take_base": Intent.TAKE_BASE,
    "resupply": Intent.RESUPPLY,
    "retreat": Intent.RETREAT,
}


def _dist(robot: dict, point: list[float]) -> float:
    return math.hypot(parse.as_float(robot.get("x")) - point[0],
                      parse.as_float(robot.get("y")) - point[1])


def infer_intent(observation: dict, cfg: dict) -> int:
    own = parse.own_robot(observation, cfg.get("own_slot", "7"))
    hp = parse.hp_ratio(own)
    ammo = parse.ammo_ratio(own)
    distances = {name: _dist(own, goal(cfg, name)) for name in GOAL_TO_INTENT}
    nearest_name = min(distances, key=distances.get)
    nearest_dist = distances[nearest_name]

    # Safety first: very low HP heading home.
    if hp < 0.35 and (distances["retreat"] < 6.0 or hp < 0.20):
        return int(Intent.RETREAT)
    # Low ammo / medium HP near supply.
    if (ammo < 0.30 or hp < 0.60) and distances["resupply"] < 8.0:
        return int(Intent.RESUPPLY)
    # Already on a goal card.
    if nearest_dist < 2.5:
        return int(GOAL_TO_INTENT[nearest_name])
    # In a fight away from the card.
    _, enemy_dist = parse.nearest_visible(observation)
    if enemy_dist <= 12.0:
        return int(Intent.ENGAGE)
    return int(GOAL_TO_INTENT[nearest_name])
