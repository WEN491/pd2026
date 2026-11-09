"""Sentry-side suggestion receiver / ENGAGE state machine (reference)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .radar_link import Suggestion

MODE_IDLE = "idle"
MODE_ACTION = "action"       # non-ENGAGE macro
MODE_VISION = "vision"       # ENGAGE with local vision
MODE_APPROACH = "approach"   # ENGAGE, no vision, fresh radar position
MODE_SEARCH = "search"       # ENGAGE, no vision, stale radar position
MODE_FALLBACK = "fallback"   # HOLD_OUTPOST / local safe fallback


@dataclass(frozen=True)
class SentryDecision:
    mode: str
    action_id: Optional[int]
    target_id: int
    action_seq: Optional[int]
    age_ms: int
    reason: str
    recovered: bool = False
    fallback: bool = False


class SentryReceiver:
    """Deterministic reference for the sentry-side state machine.

    It does not publish ROS or touch the BT. It turns validated suggestions
    into a mode the BT adapter can map. Thresholds follow ICD §8.3.
    """

    def __init__(
        self,
        engage_action_id: int = 6,
        link_timeout_ms: int = 1000,
        position_max_age_ms: int = 300,
        target_lost_timeout_ms: int = 2000,
    ) -> None:
        # Reference default only: action 1..8 semantics are owned by the
        # action table, not by the communication layer. Set this from the table.
        self.engage_action_id = engage_action_id
        self.link_timeout_ms = link_timeout_ms
        self.position_max_age_ms = position_max_age_ms
        self.target_lost_timeout_ms = target_lost_timeout_ms

        self.current: Optional[Suggestion] = None
        self.last_suggestion_ms: Optional[int] = None
        self.last_action_seq: Optional[int] = None
        self.last_action_id: Optional[int] = None
        self.last_target_id: int = 0
        self.last_position_ms: Optional[int] = None
        self.fallback_latched = False
        self.fallback_reason = ""
        self.applied_count = 0

    # -- ingress ---------------------------------------------------------
    def ingest(
        self, suggestion: Suggestion, now_ms: int, visible_target_id: Optional[int] = None
    ) -> SentryDecision:
        recovered = self.fallback_latched
        changed = (
            recovered
            or self.last_action_seq is None
            or suggestion.action_seq != self.last_action_seq
        )

        if changed:
            self.last_action_seq = suggestion.action_seq
            self.last_action_id = suggestion.action_id
            self.applied_count += 1
            self.fallback_latched = False
            self.fallback_reason = ""

        if suggestion.target_valid:
            if suggestion.target_id != self.last_target_id:
                self.last_position_ms = None
            self.last_target_id = suggestion.target_id
            if suggestion.position_valid:
                self.last_position_ms = now_ms - suggestion.target_age_ms
        else:
            self.last_target_id = 0
            self.last_position_ms = None

        self.current = suggestion
        self.last_suggestion_ms = now_ms
        return self.evaluate(now_ms, visible_target_id, recovered=recovered)

    def tick(
        self, now_ms: int, visible_target_id: Optional[int] = None
    ) -> SentryDecision:
        return self.evaluate(now_ms, visible_target_id, recovered=False)

    # -- state machine ---------------------------------------------------
    def _age_ms(self, now_ms: int) -> int:
        if self.current is None:
            return 0
        if self.last_position_ms is not None:
            return max(0, now_ms - self.last_position_ms)
        if self.last_suggestion_ms is None:
            return self.current.target_age_ms
        return self.current.target_age_ms + max(0, now_ms - self.last_suggestion_ms)

    def _fallback(self, reason: str, age_ms: int) -> SentryDecision:
        if not self.fallback_latched:
            self.fallback_latched = True
            self.fallback_reason = reason
            self.last_action_seq = None  # forces same-seq re-apply after recovery
        return SentryDecision(
            mode=MODE_FALLBACK,
            action_id=self.current.action_id if self.current else None,
            target_id=self.current.target_id if self.current else 0,
            action_seq=None,
            age_ms=age_ms,
            reason=reason,
            fallback=True,
        )

    def evaluate(
        self,
        now_ms: int,
        visible_target_id: Optional[int] = None,
        *,
        recovered: bool = False,
    ) -> SentryDecision:
        if self.current is None:
            return SentryDecision(
                mode=MODE_IDLE, action_id=None, target_id=0,
                action_seq=None, age_ms=0, reason="no_suggestion",
            )

        s = self.current
        age = self._age_ms(now_ms)
        link_down = (
            self.last_suggestion_ms is None
            or now_ms - self.last_suggestion_ms > self.link_timeout_ms
        )
        if s.target_valid:
            visible = visible_target_id is not None and visible_target_id == s.target_id
        else:
            visible = visible_target_id is not None

        if self.fallback_latched:
            return SentryDecision(
                mode=MODE_FALLBACK, action_id=s.action_id, target_id=s.target_id,
                action_seq=None, age_ms=age, reason=self.fallback_reason,
                fallback=True,
            )

        if link_down:
            if s.action_id == self.engage_action_id and visible:
                return SentryDecision(
                    mode=MODE_VISION, action_id=s.action_id,
                    target_id=int(visible_target_id), action_seq=s.action_seq,
                    age_ms=age, reason="link_down_vision", recovered=recovered,
                )
            return self._fallback("link_lost", age)

        if s.action_id != self.engage_action_id:
            return SentryDecision(
                mode=MODE_ACTION, action_id=s.action_id, target_id=s.target_id,
                action_seq=s.action_seq, age_ms=age, reason="non_engage",
                recovered=recovered,
            )

        if not s.target_valid or not s.target_alive:
            if visible:
                return SentryDecision(
                    mode=MODE_VISION, action_id=s.action_id,
                    target_id=int(visible_target_id), action_seq=s.action_seq,
                    age_ms=age, reason="vision_no_radar_target", recovered=recovered,
                )
            return self._fallback("target_invalid", age)

        if visible:
            return SentryDecision(
                mode=MODE_VISION, action_id=s.action_id, target_id=s.target_id,
                action_seq=s.action_seq, age_ms=age, reason="visible",
                recovered=recovered,
            )

        if s.position_valid and age <= self.position_max_age_ms:
            return SentryDecision(
                mode=MODE_APPROACH, action_id=s.action_id, target_id=s.target_id,
                action_seq=s.action_seq, age_ms=age, reason="radar_position",
                recovered=recovered,
            )

        # stale position: stop approaching, search/hold current point
        if self.last_position_ms is None or (
            now_ms - self.last_position_ms > self.target_lost_timeout_ms
        ):
            return self._fallback("target_lost", age)
        return SentryDecision(
            mode=MODE_SEARCH, action_id=s.action_id, target_id=s.target_id,
            action_seq=s.action_seq, age_ms=age, reason="stale_position",
            recovered=recovered,
        )


__all__ = [
    "MODE_IDLE", "MODE_ACTION", "MODE_VISION", "MODE_APPROACH",
    "MODE_SEARCH", "MODE_FALLBACK", "SentryDecision", "SentryReceiver",
]
