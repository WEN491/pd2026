"""Tiny JSONL metric logger (no tensorboard dependency required)."""
from __future__ import annotations

import json
import time
from collections import defaultdict
from pathlib import Path
from typing import Any


class MetricLogger:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._sum: dict[str, float] = defaultdict(float)
        self._count: dict[str, int] = defaultdict(int)
        self._t0 = time.time()

    def log(self, step: int, **metrics: Any) -> None:
        row = {"step": int(step), "wall_s": round(time.time() - self._t0, 3), **metrics}
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
        for key, value in metrics.items():
            if isinstance(value, (int, float)):
                self._sum[key] += float(value)
                self._count[key] += 1

    def mean(self, key: str) -> float | None:
        count = self._count.get(key, 0)
        return self._sum[key] / count if count else None
