"""Radar wireless 0x0A01-0x0A06 parsing (2027 protocol).

Only 0x0A01-0x0A05 feed the RL observation / suggestion. 0x0A06 is the
jamming key and is kept local to the radar; it is parsed only so callers can
explicitly see it and must not forward it.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import struct
from typing import Dict, Iterable, Optional, Tuple

from .radar_link import LinkError

DEFAULT_RED_ENEMIES = (101, 103, 104, 107)  # blue heavy/3/4/sentry
DEFAULT_BLUE_ENEMIES = (1, 3, 4, 7)         # red heavy/3/4/sentry

SLOT_NAMES = ("heavy", "standard3", "standard4", "sentry")


def _need(data: bytes, size: int, name: str) -> None:
    if len(data) != size:
        raise LinkError(f"{name} must be {size} B, got {len(data)}")


def _enemy_ids(enemy_ids: Iterable[int]) -> Tuple[int, int, int, int]:
    ids = tuple(int(v) for v in enemy_ids)
    if len(ids) != 4 or len(set(ids)) != 4:
        raise LinkError(f"enemy_ids must be 4 unique IDs, got {ids!r}")
    return ids  # type: ignore[return-value]


@dataclass
class RadarTarget:
    robot_id: int
    x: float = 0.0
    y: float = 0.0
    hp: int = 0
    ammo: int = 0
    position_valid: bool = False
    hp_valid: bool = False
    ammo_valid: bool = False
    macro: bytes = b""
    buffs: bytes = b""


@dataclass
class RadarSnapshot:
    timestamp_ms: int = 0
    targets: Dict[int, RadarTarget] = field(default_factory=dict)
    key: Optional[bytes] = None

    def get(self, robot_id: int) -> Optional[RadarTarget]:
        return self.targets.get(robot_id)


def parse_0a01(data: bytes, enemy_ids: Iterable[int] = DEFAULT_RED_ENEMIES) -> Dict[int, Tuple[float, float]]:
    """Enemy ground positions, 24 B, centimetres -> metres."""
    _need(data, 24, "0x0A01")
    ids = _enemy_ids(enemy_ids)
    v = struct.unpack("<12h", data)
    # heavy x/y, reserved x2, 3 x/y, 4 x/y, reserved x2, sentry x/y
    slots = ((v[0], v[1]), (v[4], v[5]), (v[6], v[7]), (v[10], v[11]))
    return {rid: (x / 100.0, y / 100.0) for rid, (x, y) in zip(ids, slots)}


def parse_0a02(data: bytes, enemy_ids: Iterable[int] = DEFAULT_RED_ENEMIES) -> Dict[int, int]:
    """Enemy HP, 12 B: heavy, reserved, 3, 4, reserved, sentry."""
    _need(data, 12, "0x0A02")
    ids = _enemy_ids(enemy_ids)
    v = struct.unpack("<6H", data)
    return {rid: hp for rid, hp in zip(ids, (v[0], v[2], v[3], v[5]))}


def parse_0a03(data: bytes, enemy_ids: Iterable[int] = DEFAULT_RED_ENEMIES) -> Dict[int, int]:
    """Enemy 17mm allowance, 10 B: heavy, 3, 4, reserved, sentry."""
    _need(data, 10, "0x0A03")
    ids = _enemy_ids(enemy_ids)
    v = struct.unpack("<5H", data)
    return {rid: ammo for rid, ammo in zip(ids, (v[0], v[1], v[2], v[4]))}


def parse_0a04(data: bytes) -> bytes:
    """Enemy macro state, 12 B; kept raw for optional observation."""
    _need(data, 12, "0x0A04")
    return bytes(data)


def parse_0a05(data: bytes) -> bytes:
    """Enemy buffs, 47 B; kept raw for optional observation."""
    _need(data, 47, "0x0A05")
    return bytes(data)


def parse_0a06(data: bytes) -> bytes:
    """Jamming key, 6 B ASCII; radar-local only, never forwarded."""
    _need(data, 6, "0x0A06")
    return bytes(data)


def parse_0a(cmd_id: int, data: bytes, enemy_ids: Iterable[int] = DEFAULT_RED_ENEMIES):
    if cmd_id == 0x0A01:
        return parse_0a01(data, enemy_ids)
    if cmd_id == 0x0A02:
        return parse_0a02(data, enemy_ids)
    if cmd_id == 0x0A03:
        return parse_0a03(data, enemy_ids)
    if cmd_id == 0x0A04:
        return parse_0a04(data)
    if cmd_id == 0x0A05:
        return parse_0a05(data)
    if cmd_id == 0x0A06:
        return parse_0a06(data)
    raise LinkError(f"unsupported 0x0A command: 0x{cmd_id:04X}")


class RadarAggregator:
    """Accumulate the 10 Hz 0x0A stream into a snapshot.

    Positions are converted cm -> m. Missing HP/ammo stay at 0 with the
    corresponding ``*_valid`` flag false; callers must not treat 0 as truth.
    """

    def __init__(self, enemy_ids: Iterable[int] = DEFAULT_RED_ENEMIES) -> None:
        self.enemy_ids = _enemy_ids(enemy_ids)
        self.snapshot = RadarSnapshot(
            targets={rid: RadarTarget(rid) for rid in self.enemy_ids}
        )

    def update(self, cmd_id: int, data: bytes, timestamp_ms: int) -> None:
        if cmd_id == 0x0A01:
            for rid, (x, y) in parse_0a01(data, self.enemy_ids).items():
                t = self.snapshot.targets[rid]
                t.x, t.y = x, y
                t.position_valid = True
        elif cmd_id == 0x0A02:
            for rid, hp in parse_0a02(data, self.enemy_ids).items():
                t = self.snapshot.targets[rid]
                t.hp = hp
                t.hp_valid = True
        elif cmd_id == 0x0A03:
            for rid, ammo in parse_0a03(data, self.enemy_ids).items():
                t = self.snapshot.targets[rid]
                t.ammo = ammo
                t.ammo_valid = True
        elif cmd_id == 0x0A04:
            raw = parse_0a04(data)
            for t in self.snapshot.targets.values():
                t.macro = raw
        elif cmd_id == 0x0A05:
            raw = parse_0a05(data)
            for t in self.snapshot.targets.values():
                t.buffs = raw
        elif cmd_id == 0x0A06:
            # Key is radar-local; keep it in the snapshot but mark it clearly.
            self.snapshot.key = parse_0a06(data)
        else:
            raise LinkError(f"unsupported 0x0A command: 0x{cmd_id:04X}")
        self.snapshot.timestamp_ms = timestamp_ms

    def to_snapshot(self) -> RadarSnapshot:
        return self.snapshot


__all__ = [
    "DEFAULT_RED_ENEMIES", "DEFAULT_BLUE_ENEMIES", "SLOT_NAMES",
    "RadarTarget", "RadarSnapshot", "RadarAggregator",
    "parse_0a01", "parse_0a02", "parse_0a03", "parse_0a04",
    "parse_0a05", "parse_0a06", "parse_0a",
]
