"""Suggestion source interface + a deterministic demo source."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol

from .radar_link import SentryState


@dataclass(frozen=True)
class SuggestionInput:
    action_id: int
    target_id: int = 0
    target_x: float = 0.0
    target_y: float = 0.0
    target_age_ms: int = 0
    target_quality: int = 0
    target_alive: bool = False


class SuggestionSource(Protocol):
    def poll(self, now_ms: int, state: Optional[SentryState]) -> Optional[SuggestionInput]:
        ...


class DemoSource:
    """Alternate ENGAGE(101) and HOLD every 5 s; never violates 1 Hz."""

    def __init__(self, period_ms: int = 5000) -> None:
        self.period_ms = period_ms

    def poll(self, now_ms: int, state: Optional[SentryState]) -> Optional[SuggestionInput]:
        phase = (now_ms // self.period_ms) % 2
        if phase == 0:
            return SuggestionInput(
                action_id=6,
                target_id=101,
                target_x=1.0 + (now_ms % 1000) / 10000.0,
                target_y=2.0,
                target_age_ms=50,
                target_quality=2,
                target_alive=True,
            )
        return SuggestionInput(action_id=1)


__all__ = ["SuggestionInput", "SuggestionSource", "DemoSource"]
