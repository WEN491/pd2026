"""Build the 1 Hz 0x0220 sentry state/ack packet."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

from .radar_link import LinkError, SentryState

# 0x0220 flags
STATE_ALIVE = 1 << 0
STATE_WEAK = 1 << 1
STATE_LINK_OK = 1 << 2


@dataclass
class SentryStateBuilder:
    """Stateful 0x0220 builder.

    ``team_positions`` is fixed order heavy / standard3 / standard4. A missing
    teammate is ``None``; its valid bit is cleared and coordinates are 0.
    """

    last_action_seq: int = 0
    rx_count: int = 0
    crc_errors: int = 0
    seq: int = 0

    def build(
        self,
        now_ms: int,
        *,
        sentry_x: float,
        sentry_y: float,
        hp: int,
        ammo: int,
        posture: int,
        enhanced: bool,
        timers: Sequence[int],
        alive: bool = True,
        weak: bool = False,
        link_ok: bool = True,
        team_positions: Sequence[Optional[Tuple[float, float]]] = (None, None, None),
        last_action_seq: Optional[int] = None,
        rx_count: Optional[int] = None,
        crc_errors: Optional[int] = None,
    ) -> SentryState:
        if last_action_seq is not None:
            self.last_action_seq = int(last_action_seq)
        if rx_count is not None:
            self.rx_count = int(rx_count)
        if crc_errors is not None:
            self.crc_errors = int(crc_errors)
        self.seq = (self.seq + 1) & 0xFFFF
        if self.seq == 0:
            self.seq = 1

        if len(timers) != 6:
            raise LinkError("timers must be 6 values")
        timer_bytes = bytes(int(v) & 0xFF for v in timers)
        if len(team_positions) != 3:
            raise LinkError("team_positions must be 3 entries")

        flags = 0
        if alive:
            flags |= STATE_ALIVE
        if weak:
            flags |= STATE_WEAK
        if link_ok:
            flags |= STATE_LINK_OK

        team_valid = 0
        team_flat = []
        for i, pos in enumerate(team_positions):
            if pos is None:
                team_flat.extend((0.0, 0.0))
            else:
                x, y = pos
                team_valid |= 1 << i
                team_flat.extend((float(x), float(y)))

        state = SentryState(
            version=1,
            flags=flags,
            seq=self.seq,
            timestamp_ms=int(now_ms),
            last_action_seq=self.last_action_seq,
            rx_count=self.rx_count,
            crc_errors=self.crc_errors,
            sentry_x=float(sentry_x),
            sentry_y=float(sentry_y),
            hp=int(hp),
            ammo=int(ammo),
            posture=int(posture),
            enhanced=1 if enhanced else 0,
            timers=timer_bytes,
            team_valid=team_valid,
            team_pos=tuple(team_flat),
        )
        state.validate()
        return state


__all__ = ["STATE_ALIVE", "STATE_WEAK", "STATE_LINK_OK", "SentryStateBuilder"]
