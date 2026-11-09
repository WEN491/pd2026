"""Observation builder: Arena-1 dict -> fixed-size float vector.

Layout (default 4 ground enemies, 59 dims)::

    enemy block     4 x [rel_x, rel_y, dist, hp_ratio, alive]   = 20
    structures      4 x [hp_ratio, owner_sign, armor_open]      = 12
    control points  4 x [controller_sign, control, progress]    = 12
    self            [x, y, hp, ammo, heat,
                     posture_onehot(3), enhance_left, cooldown,
                     sin(heading), cos(heading)]                 = 12
    radar           [link, age]                                  =  2
    time            [remaining / 420]                            =  1

Positions are normalised by the field, so the same policy works on the real
28x15 field and on Arena-1.  Dead/unknown enemies keep their public HP/alive
bits but contribute a zero position.
"""
from __future__ import annotations

import math

import numpy as np

from . import parse
from .config import obs_dim as configured_obs_dim

FULL_AMMO = 300.0
MAX_HEAT = 260.0


def _norm_xy(x: float, y: float, cfg: dict) -> tuple[float, float]:
    field = cfg["field"]
    return ((x - field["center_x"]) / (field["width"] / 2.0),
            (y - field["center_y"]) / (field["height"] / 2.0))


def _diag(cfg: dict) -> float:
    return math.hypot(cfg["field"]["width"], cfg["field"]["height"])


def _enemy_block(observation: dict, radar: dict | None, cfg: dict) -> list[float]:
    team = str(observation.get("team", cfg.get("team", "red")))
    own = parse.own_robot(observation, cfg.get("own_slot", "7"))
    ox, oy = parse.as_float(own.get("x")), parse.as_float(own.get("y"))
    public = parse.enemy_public(observation)
    visible = {str(item.get("slot")): item for item in parse.visible_enemies(observation)}
    radar_enemies = (radar or {}).get("enemies", {})
    out: list[float] = []
    for slot in cfg["enemy_slots"]:
        slot = str(slot)
        pub = public.get(slot, {})
        max_hp = parse.as_float(pub.get("max_hp"))
        hp = parse.as_float(pub.get("hp")) / max_hp if max_hp > 0 else 0.0
        alive = 1.0 if pub.get("alive") else 0.0

        sample = radar_enemies.get(slot)
        source = None
        if sample and sample.get("known"):
            source = sample
        elif slot in visible:
            source = visible[slot]

        if source is None:
            dx = dy = dist = 0.0
        else:
            ex, ey = parse.as_float(source.get("x")), parse.as_float(source.get("y"))
            nx, ny = _norm_xy(ex, ey, cfg)
            oxn, oyn = _norm_xy(ox, oy, cfg)
            dx, dy = nx - oxn, ny - oyn
            dist = math.hypot(ex - ox, ey - oy) / _diag(cfg)
        out.extend([dx, dy, dist, max(0.0, min(1.0, hp)), alive])
    return out


def _structure_block(observation: dict, cfg: dict) -> list[float]:
    team = str(observation.get("team", cfg.get("team", "red")))
    enemy = parse.other_team(team)
    out: list[float] = []
    for side in ("red", "blue"):
        for kind in ("outposts", "bases"):
            item = parse.structure(observation, kind, side)
            max_hp = parse.as_float(item.get("max_hp"))
            hp = parse.as_float(item.get("hp")) / max_hp if max_hp > 0 else 0.0
            owner = str(item.get("owner", side))
            owner_sign = 1.0 if owner == team else (-1.0 if owner == enemy else 0.0)
            armor_open = 1.0 if (kind == "bases" and item.get("armor_open")) else 0.0
            out.extend([max(0.0, min(1.0, hp)), owner_sign, armor_open])
    return out


def _position_block(observation: dict, cfg: dict) -> list[float]:
    team = str(observation.get("team", cfg.get("team", "red")))
    enemy = parse.other_team(team)
    positions = observation.get("positions") or {}
    out: list[float] = []
    for key in (f"{team}_outpost", f"{enemy}_outpost", f"{team}_base", f"{enemy}_base"):
        item = positions.get(key) or {}
        controller = str(item.get("controller", ""))
        controller_sign = 1.0 if controller == team else (-1.0 if controller == enemy else 0.0)
        control = {"full": 1.0, "temporary": 0.5}.get(str(item.get("control", "")), 0.0)
        progress = max(0.0, min(1.0, parse.as_float(item.get("progress")) / 5.0))
        out.extend([controller_sign, control, progress])
    return out


def _self_block(observation: dict, cfg: dict) -> list[float]:
    own = parse.own_robot(observation, cfg.get("own_slot", "7"))
    ox, oy = _norm_xy(parse.as_float(own.get("x")), parse.as_float(own.get("y")), cfg)
    hp = parse.hp_ratio(own)
    ammo = max(0.0, min(1.0, parse.as_float(own.get("ammo")) / FULL_AMMO))
    heat = max(0.0, min(1.0, parse.as_float(own.get("heat")) / MAX_HEAT))
    posture = parse.posture_index(own)
    posture_onehot = [0.0, 0.0, 0.0]
    posture_onehot[posture] = 1.0
    enhance = parse.posture_enhanced_remaining(own)
    now = parse.as_float(observation.get("time"))
    ready_at = parse.as_float(own.get("posture_ready_at"), now)
    cooldown = max(0.0, min(1.0, (ready_at - now) / 5.0))
    heading = parse.as_float(own.get("heading"))
    return [ox, oy, hp, ammo, heat, *posture_onehot, enhance, cooldown,
            math.sin(heading), math.cos(heading)]


def build_obs(observation: dict, radar: dict | None = None, cfg: dict | None = None) -> np.ndarray:
    """Build the policy observation.  Never pass a raw ``snapshot()`` here."""
    if cfg is None:
        from .config import load_config
        cfg = load_config()
    radar = radar or {}
    remaining = parse.as_float(observation.get("remaining"),
                              max(0.0, 420.0 - parse.as_float(observation.get("time"))))
    link = 1.0 if radar.get("link") else 0.0
    age = max(0.0, min(1.0, parse.as_float(radar.get("age"), 1.0)))
    vector = (
        _enemy_block(observation, radar, cfg)
        + _structure_block(observation, cfg)
        + _position_block(observation, cfg)
        + _self_block(observation, cfg)
        + [link, age]
        + [max(0.0, min(1.0, remaining / 420.0))]
    )
    arr = np.asarray(vector, dtype=np.float32)
    arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
    expected = configured_obs_dim(cfg)
    if arr.size != expected:
        raise ValueError(f"obs size {arr.size} != configured {expected}; check enemy_slots")
    return arr


def obs_names(cfg: dict) -> list[str]:
    """Human-readable feature names, same order as :func:`build_obs`."""
    names: list[str] = []
    for slot in cfg["enemy_slots"]:
        names += [f"e{slot}_dx", f"e{slot}_dy", f"e{slot}_dist", f"e{slot}_hp", f"e{slot}_alive"]
    for side in ("red", "blue"):
        for kind in ("outpost", "base"):
            names += [f"{side}_{kind}_hp", f"{side}_{kind}_owner", f"{side}_{kind}_armor"]
    for key in ("own_outpost", "enemy_outpost", "own_base", "enemy_base"):
        names += [f"{key}_controller", f"{key}_control", f"{key}_progress"]
    names += ["self_x", "self_y", "self_hp", "self_ammo", "self_heat",
              "self_posture_attack", "self_posture_defense", "self_posture_move",
              "self_enhance_left", "self_cooldown", "self_heading_sin", "self_heading_cos"]
    names += ["radar_link", "radar_age", "time_left"]
    return names
