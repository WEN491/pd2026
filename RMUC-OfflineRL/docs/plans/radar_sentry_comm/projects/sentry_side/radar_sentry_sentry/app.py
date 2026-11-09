"""Sentry-side service: receive 0x0210 suggestion, send 1 Hz 0x0220 state/ack."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import logging
import time
from pathlib import Path
from typing import Optional

from .radar_link import (
    FrameStreamParser, LinkError, Suggestion, decode_suggestion,
    encode_sentry_state,
)
from .sentry_rx import SentryDecision, SentryReceiver
from .sentry_state import SentryStateBuilder
from .serial_port import FakeTransport, SerialTransport
from .state_provider import DemoStateProvider, StateProvider

log = logging.getLogger("radar_sentry_sentry")


@dataclass
class SentryConfig:
    device: str = ""
    baud: int = 115200
    sender_id: int = 7
    receiver_id: int = 9
    state_hz: float = 1.0
    engage_action_id: int = 6
    demo: bool = False


class SentryApp:
    def __init__(
        self,
        transport,
        provider: StateProvider,
        *,
        sender_id: int = 7,
        receiver_id: int = 9,
        state_hz: float = 1.0,
        engage_action_id: int = 6,
    ) -> None:
        self.transport = transport
        self.provider = provider
        self.sender_id = sender_id
        self.receiver_id = receiver_id
        self.state_period_ms = max(1, int(round(1000.0 / state_hz)))
        self.rx = SentryReceiver(engage_action_id=engage_action_id)
        self.builder = SentryStateBuilder()
        self.parser = FrameStreamParser()
        self.next_state_ms = 0
        self.suggestions = 0
        self.crc_errors = 0
        self.frame_seq = 0
        self.last_mode: Optional[str] = None

    def _log_decision(self, d: SentryDecision, logs: list[str]) -> None:
        if d.mode != self.last_mode:
            self.last_mode = d.mode
            logs.append(
                f"mode={d.mode} reason={d.reason} action={d.action_id} "
                f"target={d.target_id} age={d.age_ms} fallback={d.fallback}"
            )

    def step(self, now_ms: int, incoming: bytes = b"") -> list[str]:
        logs: list[str] = []
        for frame in self.parser.feed(incoming):
            try:
                s = decode_suggestion(frame.raw, self.receiver_id, self.sender_id)
            except LinkError as exc:
                self.crc_errors += 1
                logs.append(f"drop suggestion: {exc}")
                continue
            self.suggestions += 1
            d = self.rx.ingest(s, now_ms, self.provider.visible_target_id())
            self._log_decision(d, logs)

        d = self.rx.tick(now_ms, self.provider.visible_target_id())
        self._log_decision(d, logs)

        if now_ms >= self.next_state_ms:
            sample = self.provider.sample(now_ms)
            state = self.builder.build(
                now_ms,
                sentry_x=sample.sentry_x,
                sentry_y=sample.sentry_y,
                hp=sample.hp,
                ammo=sample.ammo,
                posture=sample.posture,
                enhanced=sample.enhanced,
                timers=sample.timers,
                alive=sample.alive,
                weak=sample.weak,
                link_ok=sample.link_ok,
                team_positions=sample.team_positions,
                last_action_seq=self.rx.last_action_seq or 0,
                rx_count=self.suggestions & 0xFFFF,
                crc_errors=self.crc_errors & 0xFFFF,
            )
            raw = encode_sentry_state(state, self.sender_id, self.receiver_id, self.frame_seq)
            self.frame_seq = (self.frame_seq + 1) & 0xFF
            self.transport.write(raw)
            logs.append(f"0x0220 seq={state.seq} action_seq={state.last_action_seq} frame={len(raw)}B")
            self.next_state_ms += self.state_period_ms
        return logs

    def run(self, duration_s: float = 0.0) -> None:
        start = time.monotonic()
        while True:
            now_ms = int((time.monotonic() - start) * 1000)
            if duration_s and now_ms >= duration_s * 1000:
                break
            try:
                incoming = self.transport.read()
                for line in self.step(now_ms, incoming):
                    log.info(line)
            except LinkError as exc:
                log.error("protocol error: %s", exc)
            time.sleep(0.01)


def load_config(path: Optional[str]) -> SentryConfig:
    cfg = SentryConfig()
    if path:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        for key, value in data.items():
            if hasattr(cfg, key):
                setattr(cfg, key, value)
    return cfg


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", help="JSON config file")
    parser.add_argument("--device", help="serial device, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int)
    parser.add_argument("--sender", type=int, help="own sentry ID (red 7 / blue 107)")
    parser.add_argument("--receiver", type=int, help="radar ID (red 9 / blue 109)")
    parser.add_argument("--state-hz", type=float, help="0x0220 rate, default 1")
    parser.add_argument("--engage-action-id", type=int, default=6,
                        help="action id treated as ENGAGE by the reference state machine")
    parser.add_argument("--demo", action="store_true", help="use in-memory transport/demo state")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 = forever")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)

    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO),
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = load_config(args.config)
    for attr, val in (("device", args.device), ("baud", args.baud),
                      ("sender_id", args.sender), ("receiver_id", args.receiver),
                      ("state_hz", args.state_hz)):
        if val is not None:
            setattr(cfg, attr, val)
    cfg.demo = cfg.demo or args.demo
    cfg.engage_action_id = args.engage_action_id

    if cfg.demo or not cfg.device:
        log.info("demo/in-memory mode; no serial writes leave the process")
        transport = FakeTransport()
    else:
        transport = SerialTransport(cfg.device, cfg.baud)
    app = SentryApp(
        transport, DemoStateProvider(),
        sender_id=cfg.sender_id, receiver_id=cfg.receiver_id,
        state_hz=cfg.state_hz, engage_action_id=cfg.engage_action_id,
    )
    try:
        app.run(args.duration)
    except KeyboardInterrupt:
        return 130
    finally:
        transport.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
