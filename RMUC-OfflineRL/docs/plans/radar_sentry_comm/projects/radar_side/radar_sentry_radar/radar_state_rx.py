"""Radar-side receiver for 0x0220 sentry state/ack."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .radar_link import LinkError, SentryState, decode_sentry_state


@dataclass
class RadarStateReceiver:
    expected_sender: int
    expected_receiver: int
    timeout_ms: int = 2000
    last_state: Optional[SentryState] = None
    last_rx_ms: Optional[int] = None
    rx_count: int = 0
    crc_errors: int = 0

    def on_frame(self, raw: bytes, now_ms: int) -> Optional[SentryState]:
        try:
            state = decode_sentry_state(raw, self.expected_sender, self.expected_receiver)
        except LinkError:
            self.crc_errors += 1
            return None
        self.last_state = state
        self.last_rx_ms = now_ms
        self.rx_count += 1
        return state

    def state_valid(self, now_ms: int) -> bool:
        return (
            self.last_rx_ms is not None
            and now_ms - self.last_rx_ms <= self.timeout_ms
        )

    def state_lost(self, now_ms: int) -> bool:
        return self.last_rx_ms is not None and not self.state_valid(now_ms)

    def ack(self) -> dict:
        if self.last_state is None:
            return {"last_action_seq": 0, "rx_count": 0, "crc_errors": self.crc_errors}
        return {
            "last_action_seq": self.last_state.last_action_seq,
            "rx_count": self.last_state.rx_count,
            "crc_errors": self.last_state.crc_errors,
        }


__all__ = ["RadarStateReceiver"]
