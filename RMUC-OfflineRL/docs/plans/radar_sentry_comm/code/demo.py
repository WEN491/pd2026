"""Small end-to-end demo: suggestion -> frame -> sentry state -> ack."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import radar_link as r
from radar_tx import RadarTransmitter
from sentry_rx import SentryReceiver
from sentry_state import SentryStateBuilder
from radar_state_rx import RadarStateReceiver


def main() -> None:
    tx = RadarTransmitter(r.RED_RADAR, r.RED_SENTRY)
    suggestion, raw = tx.suggestion_frame(
        0, 6, target_id=101, target_x=1.25, target_y=2.5,
        target_age_ms=50, target_quality=2,
    )
    print(f"0x0210 content={len(suggestion.pack())} B frame={len(raw)} B")
    print(f"frame={raw.hex(' ')}")

    rx = SentryReceiver(engage_action_id=6)
    decision = rx.ingest(
        r.decode_suggestion(raw, r.RED_RADAR, r.RED_SENTRY), 0
    )
    print(f"sentry mode={decision.mode} target={decision.target_id} reason={decision.reason}")

    state = SentryStateBuilder().build(
        0, sentry_x=1.0, sentry_y=2.0, hp=400, ammo=300,
        posture=1, enhanced=False, timers=[0, 0, 0, 0, 0, 0],
        team_positions=[(1.0, 2.0), (3.0, 4.0), None],
        last_action_seq=decision.action_seq or 0, rx_count=1,
    )
    state_raw = r.encode_sentry_state(state, r.RED_SENTRY, r.RED_RADAR, 0)
    print(f"0x0220 content={len(state.pack())} B frame={len(state_raw)} B")
    ack = RadarStateReceiver(r.RED_SENTRY, r.RED_RADAR)
    ack.on_frame(state_raw, 0)
    print(f"radar ack={ack.ack()}")


if __name__ == "__main__":
    main()
