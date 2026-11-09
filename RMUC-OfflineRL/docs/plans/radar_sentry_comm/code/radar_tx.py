"""Radar-side suggestion builder and frame transmitter (reference)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from radar_link import (
    ACTION_IDS, FLAG_ACTION_VALID, FLAG_POSITION_VALID, FLAG_STALE,
    FLAG_TARGET_ALIVE, FLAG_TARGET_VALID, TARGET_IDS, LinkError,
    RadarStatus, Suggestion, encode_radar_status, encode_suggestion,
)


@dataclass
class SuggestionBuilder:
    """Build 0x0210 at 10 Hz while enforcing the 1 Hz action rule.

    The RL side calls ``build`` on every 10 Hz tick. Same action/target keeps
    ``action_seq`` unchanged; a change is rejected if it comes <1 s after the
    previous change.
    """

    action_min_interval_ms: int = 1000
    action_seq: int = 0
    last_action_id: Optional[int] = None
    last_target_id: Optional[int] = None
    last_action_ms: Optional[int] = None

    def build(
        self,
        now_ms: int,
        action_id: int,
        *,
        target_id: int = 0,
        target_x: float = 0.0,
        target_y: float = 0.0,
        target_age_ms: int = 0,
        target_quality: int = 0,
        target_alive: Optional[bool] = None,
    ) -> Suggestion:
        if action_id not in ACTION_IDS:
            raise LinkError(f"action_id {action_id} not in 1..8")
        if target_id not in (0, *TARGET_IDS):
            raise LinkError(f"target_id {target_id} not whitelisted")
        if target_id == 0:
            target_x = target_y = 0.0
            target_age_ms = 0
            target_quality = 0
            target_alive = False
        elif target_alive is None:
            target_alive = True

        changed = (
            action_id != self.last_action_id or target_id != self.last_target_id
        )
        if changed:
            if (
                self.last_action_ms is not None
                and now_ms - self.last_action_ms < self.action_min_interval_ms
            ):
                raise LinkError(
                    f"action change at {now_ms} ms is < {self.action_min_interval_ms} ms"
                )
            self.action_seq = (self.action_seq + 1) & 0xFFFF
            if self.action_seq == 0:
                self.action_seq = 1
            self.last_action_ms = now_ms
            self.last_action_id = action_id
            self.last_target_id = target_id

        flags = FLAG_ACTION_VALID
        if target_id:
            flags |= FLAG_TARGET_VALID | FLAG_POSITION_VALID
            if target_alive:
                flags |= FLAG_TARGET_ALIVE
            if target_age_ms > 300:
                flags |= FLAG_STALE

        s = Suggestion(
            version=1,
            flags=flags,
            action_id=action_id,
            action_seq=self.action_seq,
            target_id=target_id,
            target_x=target_x,
            target_y=target_y,
            target_age_ms=target_age_ms,
            target_quality=target_quality,
            timestamp_ms=now_ms,
        )
        s.validate()
        return s


@dataclass
class RadarStatusBuilder:
    seq: int = 0

    def build(
        self, now_ms: int, *, status: int = 0, tx_count: int = 0, drop_count: int = 0
    ) -> RadarStatus:
        self.seq = (self.seq + 1) & 0xFFFF
        s = RadarStatus(
            version=1,
            status=status,
            seq=self.seq,
            timestamp_ms=now_ms,
            tx_count=tx_count,
            drop_count=drop_count,
        )
        s.validate()
        return s


class RadarTransmitter:
    """Wrap builders with sender/receiver IDs and frame sequence."""

    def __init__(
        self,
        sender_id: int,
        receiver_id: int,
        builder: Optional[SuggestionBuilder] = None,
    ) -> None:
        self.sender_id = sender_id
        self.receiver_id = receiver_id
        self.builder = builder or SuggestionBuilder()
        self.frame_seq = 0

    def suggestion_frame(self, now_ms: int, action_id: int, **kwargs) -> tuple[Suggestion, bytes]:
        s = self.builder.build(now_ms, action_id, **kwargs)
        raw = encode_suggestion(s, self.sender_id, self.receiver_id, self.frame_seq)
        self.frame_seq = (self.frame_seq + 1) & 0xFF
        return s, raw

    def status_frame(self, status: RadarStatus) -> bytes:
        raw = encode_radar_status(status, self.sender_id, self.receiver_id, self.frame_seq)
        self.frame_seq = (self.frame_seq + 1) & 0xFF
        return raw


__all__ = ["SuggestionBuilder", "RadarStatusBuilder", "RadarTransmitter"]
