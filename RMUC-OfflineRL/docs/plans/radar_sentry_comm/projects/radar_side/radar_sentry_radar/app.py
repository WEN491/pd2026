"""Radar-side service: send 0x0210 at 10 Hz, receive 0x0220 at 1 Hz."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import logging
import time
from pathlib import Path
from typing import Optional

from .radar_link import (
    FrameStreamParser, LinkError, SUBCMD_SENTRY_STATE, decode_interaction,
)
from .radar_state_rx import RadarStateReceiver
from .radar_tx import RadarStatusBuilder, RadarTransmitter, SuggestionBuilder
from .serial_port import FakeTransport, SerialTransport
from .source import DemoSource, SuggestionSource

log = logging.getLogger("radar_sentry_radar")


@dataclass
class RadarConfig:
    device: str = ""
    baud: int = 115200
    sender_id: int = 9
    receiver_id: int = 7
    rate_hz: float = 10.0
    status_hz: float = 1.0
    demo: bool = False


class RadarApp:
    def __init__(
        self,
        transport,
        source: SuggestionSource,
        *,
        sender_id: int = 9,
        receiver_id: int = 7,
        rate_hz: float = 10.0,
        status_hz: float = 1.0,
    ) -> None:
        self.transport = transport
        self.source = source
        self.sender_id = sender_id
        self.receiver_id = receiver_id
        self.period_ms = max(1, int(round(1000.0 / rate_hz)))
        self.status_period_ms = max(1, int(round(1000.0 / status_hz)))
        self.tx = RadarTransmitter(sender_id, receiver_id)
        self.status = RadarStatusBuilder()
        self.state_rx = RadarStateReceiver(receiver_id, sender_id)
        self.parser = FrameStreamParser()
        self.next_tick_ms = 0
        self.next_status_ms = 0
        self.tx_count = 0
        self.drop_count = 0

    def step(self, now_ms: int, incoming: bytes = b"") -> list[str]:
        logs: list[str] = []
        for frame in self.parser.feed(incoming):
            try:
                it = decode_interaction(frame.raw, self.receiver_id, self.sender_id)
            except LinkError as exc:
                self.drop_count += 1
                logs.append(f"drop: {exc}")
                continue
            if it.data_cmd_id == SUBCMD_SENTRY_STATE:
                st = self.state_rx.on_frame(frame.raw, now_ms)
                if st is not None:
                    logs.append(
                        f"0x0220 seq={st.seq} action_seq={st.last_action_seq} "
                        f"hp={st.hp} ammo={st.ammo} posture={st.posture}"
                    )
        if now_ms >= self.next_tick_ms:
            spec = self.source.poll(now_ms, self.state_rx.last_state)
            if spec is not None:
                suggestion, raw = self.tx.suggestion_frame(
                    now_ms,
                    spec.action_id,
                    target_id=spec.target_id,
                    target_x=spec.target_x,
                    target_y=spec.target_y,
                    target_age_ms=spec.target_age_ms,
                    target_quality=spec.target_quality,
                    target_alive=spec.target_alive,
                )
                self.transport.write(raw)
                self.tx_count += 1
                logs.append(
                    f"0x0210 action={suggestion.action_id} seq={suggestion.action_seq} "
                    f"target={suggestion.target_id} frame={len(raw)}B"
                )
            self.next_tick_ms += self.period_ms
        if now_ms >= self.next_status_ms:
            status = self.status.build(
                now_ms,
                status=0,
                tx_count=self.tx_count & 0xFFFF,
                drop_count=self.drop_count & 0xFFFF,
            )
            self.transport.write(self.tx.status_frame(status))
            self.next_status_ms += self.status_period_ms
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
            sleep_s = max(0.0, (self.next_tick_ms - now_ms) / 1000.0)
            time.sleep(min(sleep_s, 0.02))


def load_config(path: Optional[str]) -> RadarConfig:
    cfg = RadarConfig()
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
    parser.add_argument("--sender", type=int, help="own radar ID (red 9 / blue 109)")
    parser.add_argument("--receiver", type=int, help="sentry ID (red 7 / blue 107)")
    parser.add_argument("--rate-hz", type=float, help="suggestion rate, default 10")
    parser.add_argument("--status-hz", type=float, help="status rate, default 1")
    parser.add_argument("--demo", action="store_true", help="use in-memory transport/demo source")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 = forever")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)

    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO),
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = load_config(args.config)
    for attr, val in (("device", args.device), ("baud", args.baud),
                      ("sender_id", args.sender), ("receiver_id", args.receiver),
                      ("rate_hz", args.rate_hz), ("status_hz", args.status_hz)):
        if val is not None:
            setattr(cfg, attr, val)
    cfg.demo = cfg.demo or args.demo

    if cfg.demo or not cfg.device:
        log.info("demo/in-memory mode; no serial writes leave the process")
        transport = FakeTransport()
    else:
        transport = SerialTransport(cfg.device, cfg.baud)
    app = RadarApp(
        transport, DemoSource(),
        sender_id=cfg.sender_id, receiver_id=cfg.receiver_id,
        rate_hz=cfg.rate_hz, status_hz=cfg.status_hz,
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
