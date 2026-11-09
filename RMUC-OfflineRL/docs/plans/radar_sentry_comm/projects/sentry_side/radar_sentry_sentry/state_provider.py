"""Sentry state input interface + a deterministic demo provider."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Protocol, Sequence, Tuple


@dataclass(frozen=True)
class StateSample:
    sentry_x: float
    sentry_y: float
    hp: int
    ammo: int
    posture: int                 # 1 attack, 2 defense, 3 move
    enhanced: bool
    timers: Tuple[int, int, int, int, int, int]
    alive: bool = True
    weak: bool = False
    link_ok: bool = True
    team_positions: Sequence[Optional[Tuple[float, float]]] = (None, None, None)
    visible_target_id: Optional[int] = None


class StateProvider(Protocol):
    def sample(self, now_ms: int) -> StateSample:
        ...

    def visible_target_id(self) -> Optional[int]:
        ...


class DemoStateProvider:
    """Static state; replace with 0x0201/0x0208/0x020B/0x020D integration."""

    def __init__(self, sentry_x: float = 1.0, sentry_y: float = 2.0) -> None:
        self.sentry_x = sentry_x
        self.sentry_y = sentry_y

    def sample(self, now_ms: int) -> StateSample:
        return StateSample(
            sentry_x=self.sentry_x,
            sentry_y=self.sentry_y,
            hp=400,
            ammo=300,
            posture=1,
            enhanced=False,
            timers=(0, 0, 0, 0, 0, 0),
            alive=True,
            weak=False,
            link_ok=True,
            team_positions=((1.0, 2.0), (3.0, 4.0), None),
            visible_target_id=None,
        )

    def visible_target_id(self) -> Optional[int]:
        return None


__all__ = ["StateSample", "StateProvider", "DemoStateProvider"]
