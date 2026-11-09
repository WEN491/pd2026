"""Synthetic radar downlink.

The real sentry only receives enemy positions through the radar link; its own
camera is a separate channel.  Arena-1 does not model radar, so this module
turns ground truth (from ``Match.snapshot()`` / the full step frame) into a
radar-like measurement with the rule-relevant knobs: 5 Hz, metre-level noise,
latency, dropout and a link timeout.

Modes:
    none       no radar; the observation falls back to visible enemies
    synthetic  noise + latency + dropout (default)
    oracle     perfect positions, no latency/dropout (A/B/C upper bound only)
"""
from __future__ import annotations

import random
from dataclasses import dataclass, fields
from typing import Any


@dataclass
class RadarConfig:
    mode: str = "synthetic"
    rate_hz: float = 5.0
    sigma_m: float = 0.7
    latency_s: float = 0.15
    dropout: float = 0.10
    timeout_s: float = 1.0


class RadarChannel:
    """Accumulate noisy ground-truth samples and expose the latest usable one."""

    def __init__(self, cfg: RadarConfig | dict, team: str, seed: int = 0):
        if isinstance(cfg, dict):
            allowed = {f.name for f in fields(RadarConfig)}
            cfg = RadarConfig(**{k: v for k, v in cfg.items() if k in allowed})
        if cfg.mode not in ("none", "synthetic", "oracle"):
            raise ValueError(f"unknown radar mode {cfg.mode!r}")
        self.cfg = cfg
        self.team = team
        self.random = random.Random(seed)
        self._samples: dict[int, list[tuple[float, float, float]]] = {}
        self._last_emit = -1.0e9

    @property
    def enabled(self) -> bool:
        return self.cfg.mode != "none"

    def _interval(self) -> float:
        if self.cfg.mode == "oracle":
            return 1.0 / max(1.0, self.cfg.rate_hz * 4.0)
        return 1.0 / max(1.0, self.cfg.rate_hz)

    def _latency(self) -> float:
        return 0.0 if self.cfg.mode == "oracle" else max(0.0, self.cfg.latency_s)

    def _dropout(self) -> float:
        return 0.0 if self.cfg.mode == "oracle" else max(0.0, min(1.0, self.cfg.dropout))

    def _sigma(self) -> float:
        return 0.0 if self.cfg.mode == "oracle" else max(0.0, self.cfg.sigma_m)

    def update(self, frame: dict, now: float | None = None) -> None:
        """Sample the full frame (all robots) at the radar rate."""
        if not self.enabled:
            return
        if now is None:
            now = float(frame.get("time", 0.0))
        if now - self._last_emit < self._interval():
            return
        self._last_emit = now
        for robot in frame.get("robots", []) or []:
            if robot.get("team") == self.team:
                continue
            if robot.get("role") == "aerial":  # 2027 radar identifies ground robots only
                continue
            if not robot.get("alive", True):
                continue
            if self.random.random() < self._dropout():
                continue
            x = float(robot.get("x", 0.0)) + self.random.gauss(0.0, self._sigma())
            y = float(robot.get("y", 0.0)) + self.random.gauss(0.0, self._sigma())
            slot = int(robot.get("slot", -1))
            if slot < 0:
                continue
            history = self._samples.setdefault(slot, [])
            history.append((now, x, y))
            del history[:-10]

    def measure(self, now: float) -> dict[str, Any]:
        """Latest sample with ``t <= now - latency``; empty when nothing usable."""
        if not self.enabled:
            return {"enemies": {}, "link": False, "age": 1.0}
        cutoff = now - self._latency()
        enemies: dict[str, dict] = {}
        ages: list[float] = []
        for slot, history in self._samples.items():
            usable = [item for item in history if item[0] <= cutoff]
            if not usable:
                continue
            t, x, y = usable[-1]
            age = now - t
            known = age <= self.cfg.timeout_s
            enemies[str(slot)] = {"x": x, "y": y, "age": age, "known": known}
            if known:
                ages.append(age)
        if not ages:
            return {"enemies": enemies, "link": False, "age": 1.0}
        return {"enemies": enemies, "link": True, "age": min(ages)}
