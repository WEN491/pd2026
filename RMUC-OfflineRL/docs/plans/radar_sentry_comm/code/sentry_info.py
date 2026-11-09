"""Minimal 0x020D / 0x0208 parsers used by the 1 Hz 0x0220 state."""
from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Tuple

from radar_link import LinkError


@dataclass(frozen=True)
class SentryInfo:
    exchanged_ammo: int
    remote_ammo_exchanges: int
    remote_hp_exchanges: int
    free_revive_available: bool
    instant_revive_available: bool
    instant_revive_cost: int
    out_of_combat: bool
    exchangeable_ammo: int
    posture: int              # 1 attack, 2 defense, 3 move
    rune_activatable: bool
    enhanced: bool
    timers: Tuple[int, int, int, int, int, int]  # attack/def/move decay, enhanced attack/def/move


def parse_0x020d(data: bytes) -> SentryInfo:
    if len(data) != 14:
        raise LinkError(f"0x020D must be 14 B, got {len(data)}")
    word0, word1, word2 = struct.unpack("<IHQ", data)

    exchanged = word0 & 0x7FF
    remote_ammo = (word0 >> 11) & 0xF
    remote_hp = (word0 >> 15) & 0xF
    free_revive = bool((word0 >> 19) & 1)
    instant_revive = bool((word0 >> 20) & 1)
    instant_cost = (word0 >> 21) & 0x3FF

    out_of_combat = bool(word1 & 1)
    exchangeable = (word1 >> 1) & 0x7FF
    posture = (word1 >> 12) & 0x3
    rune = bool((word1 >> 14) & 1)
    enhanced = bool((word1 >> 15) & 1)
    if posture not in (1, 2, 3):
        raise LinkError(f"0x020D posture {posture} not 1/2/3")

    timers = tuple((word2 >> (8 * i)) & 0xFF for i in range(6))
    return SentryInfo(
        exchanged_ammo=exchanged,
        remote_ammo_exchanges=remote_ammo,
        remote_hp_exchanges=remote_hp,
        free_revive_available=free_revive,
        instant_revive_available=instant_revive,
        instant_revive_cost=instant_cost,
        out_of_combat=out_of_combat,
        exchangeable_ammo=exchangeable,
        posture=posture,
        rune_activatable=rune,
        enhanced=enhanced,
        timers=timers,  # type: ignore[arg-type]
    )


@dataclass(frozen=True)
class ProjectileAllowance:
    projectile_17mm: int
    projectile_42mm: int
    coins: int
    fortress_reserve: int


def parse_0x0208(data: bytes) -> ProjectileAllowance:
    if len(data) != 8:
        raise LinkError(f"0x0208 must be 8 B, got {len(data)}")
    a, b, coins, fortress = struct.unpack("<4H", data)
    return ProjectileAllowance(a, b, coins, fortress)


__all__ = ["SentryInfo", "ProjectileAllowance", "parse_0x020d", "parse_0x0208"]
