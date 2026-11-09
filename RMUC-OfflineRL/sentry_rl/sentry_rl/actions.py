"""The 7 macro intents and their availability mask.

Order is part of the contract: it matches ``/decision_num`` in the behaviour
tree (1..7) and the subtree order in ``rmul_2027_7v7.xml``.
"""
from __future__ import annotations

from enum import IntEnum

import numpy as np

from . import parse

NEG_INF = -1.0e9


class Intent(IntEnum):
    HOLD_OUTPOST = 0
    HOLD_BASE = 1
    TAKE_OUTPOST = 2
    TAKE_BASE = 3
    RESUPPLY = 4
    ENGAGE = 5
    RETREAT = 6


INTENT_NAMES = tuple(item.name for item in Intent)
N_ACTIONS = len(INTENT_NAMES)


def intent_name(index: int) -> str:
    if not 0 <= int(index) < N_ACTIONS:
        raise ValueError(f"intent index out of range: {index}")
    return INTENT_NAMES[int(index)]


def decision_num(index: int) -> int:
    """0-based policy index -> 1-based ``/decision_num`` value."""
    return int(index) + 1


def compute_mask(observation: dict, cfg: dict) -> np.ndarray:
    """Boolean availability mask for one observation.

    The mask is deliberately conservative: it encodes only hard availability,
    not a preference.  ``HOLD_OUTPOST`` is always available as the safe
    fallback, mirroring the BT's ``Fallback -> HoldOutpost``.
    """
    mask = np.zeros(N_ACTIONS, dtype=bool)
    team = str(observation.get("team", cfg.get("team", "red")))
    enemy = parse.other_team(team)
    own = parse.own_robot(observation, cfg.get("own_slot", "7"))
    hp = parse.hp_ratio(own)
    ammo = parse.ammo_ratio(own)
    positions = observation.get("positions") or {}
    own_outpost = positions.get(f"{team}_outpost") or {}
    enemy_outpost = positions.get(f"{enemy}_outpost") or {}
    own_base = parse.structure(observation, "bases", team)
    elapsed = parse.as_float(observation.get("time"))

    own_outpost_full = (
        own_outpost.get("controller") == team
        and own_outpost.get("control") == "full"
        and parse.as_float(own_outpost.get("progress")) <= 0.0
    )
    base_armor_open = bool(own_base.get("armor_open", False))

    mask[Intent.HOLD_OUTPOST] = True
    if (not own_outpost_full) or base_armor_open:
        mask[Intent.HOLD_BASE] = True
    if own_outpost_full:
        mask[Intent.TAKE_OUTPOST] = True
    if elapsed >= 180.0 and enemy_outpost.get("controller") == team:
        mask[Intent.TAKE_BASE] = True
    if ammo < 0.30 or hp < 0.60:
        mask[Intent.RESUPPLY] = True
    _, nearest = parse.nearest_visible(observation)
    if nearest <= 15.0:
        mask[Intent.ENGAGE] = True
    if hp < 0.35 and nearest <= 10.0:
        mask[Intent.RETREAT] = True

    if not mask.any():  # defensive: never hand the policy an all-false mask
        mask[Intent.HOLD_OUTPOST] = True
    return mask


def masked_logits(logits: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Set unavailable logits to a large negative number (in place copy)."""
    out = np.array(logits, dtype=np.float32, copy=True)
    out[~np.asarray(mask, dtype=bool)] = NEG_INF
    return out


def masked_softmax(logits: np.ndarray, mask: np.ndarray) -> np.ndarray:
    z = masked_logits(logits, mask).astype(np.float64)
    z -= np.max(z)
    exp = np.exp(z)
    total = exp.sum()
    if total <= 0 or not np.isfinite(total):
        out = np.zeros_like(exp)
        out[int(np.argmax(mask))] = 1.0
        return out
    return (exp / total).astype(np.float32)
