"""Static pools of Arena-1 example strategies.

The pool is built from ``Arena-1/examples/*/manifest.json``: one entry per
strategy directory, keyed by ``<name>@<version>``.  Entries carry the sha256 of
the actual strategy source, so a frozen pool stays reproducible even if the
example tree is edited afterwards.

Only Arena-1 manifests are read here -- no ``rmuc_arena`` import -- so pools can
be inspected, frozen and unit-tested without the Rust engine.

Two pools exist because the two sides are not interchangeable:

``TeammatePool``
    ``adaptive`` single-robot strategies, selected per slot.  Their ``act()``
    filters on ``obs['controlled_ids']``, so a strategy only commands the slots
    it is handed.

``OpponentPool``
    ``team`` strategies.  They command a whole side, which is what the opponent
    side of a match needs.
"""
from __future__ import annotations

import hashlib
import json
import random
import re
from dataclasses import dataclass
from pathlib import Path

PROFILE_PREFIX = "rmuc-2027"
POOL_FORMAT = "sentry-rl-pool-v1"
TACTICS = ("balanced", "assault", "guard")
TEAMMATE_SLOTS = ("1", "3", "4", "6")

_TACTIC_LINE = re.compile(r"^TACTIC\s*=\s*['\"]([A-Za-z]+)['\"]", re.MULTILINE)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def family_of(name: str) -> str:
    """``gpt61_2027_adaptive_2711_guard`` -> ``gpt61``; ``dsv41_1017_sentry`` -> ``dsv41_1017``."""
    for marker in ("_2027", "_2026"):
        if marker in name:
            return name.split(marker, 1)[0]
    parts = name.split("_")
    return "_".join(parts[:2]) if len(parts) > 2 else name


def tactic_of(name: str, source_text: str = "") -> str:
    """Read ``TACTIC`` from the source, falling back to the directory suffix."""
    match = _TACTIC_LINE.search(source_text or "")
    if match:
        tactic = match.group(1).strip().lower()
        if tactic in TACTICS:
            return tactic
    for tactic in ("assault", "guard"):
        if name.endswith(f"_{tactic}"):
            return tactic
    return "balanced"


@dataclass(frozen=True)
class PoolEntry:
    """One immutable strategy snapshot."""

    name: str
    source: Path
    mode: str
    robot: int | None
    version: str
    digest: str
    tactic: str
    family: str
    profile: str

    @property
    def directory(self) -> Path:
        return self.source.parent

    @property
    def key(self) -> str:
        return f"{self.name}@{self.version}"

    @property
    def label(self) -> str:
        return f"{self.name}[{self.tactic}]"

    def to_json(self) -> dict:
        return {
            "name": self.name,
            "source": str(self.source),
            "mode": self.mode,
            "robot": self.robot,
            "version": self.version,
            "digest": self.digest,
            "tactic": self.tactic,
            "family": self.family,
            "profile": self.profile,
        }

    @classmethod
    def from_json(cls, data: dict) -> "PoolEntry":
        return cls(
            name=str(data["name"]),
            source=Path(data["source"]),
            mode=str(data["mode"]),
            robot=None if data.get("robot") is None else int(data["robot"]),
            version=str(data["version"]),
            digest=str(data["digest"]),
            tactic=str(data.get("tactic") or "balanced"),
            family=str(data.get("family") or family_of(str(data["name"]))),
            profile=str(data.get("profile") or ""),
        )


def discover(
    root: str | Path,
    profile_prefix: str = PROFILE_PREFIX,
    modes: tuple[str, ...] = ("team", "adaptive"),
) -> list[PoolEntry]:
    """Scan an Arena-1 ``examples`` tree for usable strategy snapshots.

    ``root`` may be the Arena-1 checkout itself or its ``examples`` directory.
    """
    root = Path(root)
    if (root / "examples").is_dir():
        root = root / "examples"
    entries: list[PoolEntry] = []
    for manifest_path in sorted(root.glob("*/manifest.json")):
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if not isinstance(manifest, dict) or int(manifest.get("api_version") or 0) != 3:
            continue
        profile = str(manifest.get("rules_profile") or "")
        if not profile.startswith(profile_prefix):
            continue
        mode = str(manifest.get("mode") or "")
        if mode not in modes:
            continue
        files = manifest.get("entries") or {}
        if not isinstance(files, dict) or not files:
            continue
        source = manifest_path.parent / str(sorted(files.values())[0])
        if not source.is_file():
            continue
        text = source.read_text(encoding="utf-8", errors="replace")
        robot = manifest.get("robot")
        name = manifest_path.parent.name
        version = str(manifest.get("version") or _sha256(manifest_path)[:16])
        entries.append(PoolEntry(
            name=name,
            source=source,
            mode=mode,
            robot=None if robot is None else int(robot),
            version=version,
            digest=_sha256(source),
            tactic=tactic_of(name, text),
            family=family_of(name),
            profile=profile,
        ))
    return entries


def split_held_out(
    entries: list[PoolEntry], fraction: float = 0.25, salt: str = ""
) -> tuple[list[PoolEntry], list[PoolEntry]]:
    """Deterministically split ``entries`` into (pool, held-out)."""
    ordered = sorted(entries, key=lambda e: hashlib.sha256((salt + e.key).encode()).hexdigest())
    if len(ordered) < 2:
        return ordered, []
    count = int(round(len(ordered) * max(0.0, min(1.0, float(fraction)))))
    count = max(1, count) if fraction > 0 else 0
    return ordered[count:], ordered[:count]


class TeammatePool:
    """``adaptive`` strategies indexed by the robot model they were built for."""

    def __init__(self, entries: list[PoolEntry]):
        self.entries = [e for e in entries if e.mode == "adaptive"]
        self.by_robot: dict[int | None, list[PoolEntry]] = {}
        for entry in self.entries:
            self.by_robot.setdefault(entry.robot, []).append(entry)

    def __len__(self) -> int:
        return len(self.entries)

    @property
    def robots(self) -> list[int | None]:
        return sorted(self.by_robot, key=lambda value: (value is None, value or 0))

    def for_slot(self, model: int | str) -> list[PoolEntry]:
        candidates = self.by_robot.get(int(model), [])
        if not candidates:
            raise KeyError(
                f"no adaptive strategy for robot {model}; have {self.robots}"
            )
        return list(candidates)

    def sample(
        self,
        lineup: dict[str, int],
        rng: random.Random,
        mix: str = "hetero",
    ) -> dict[int, PoolEntry]:
        """Choose one strategy per teammate slot.

        ``mix="homo"`` keeps the same (family, tactic) across slots -- one
        coherent team script; ``mix="hetero"`` draws each slot independently,
        which is what actually stresses coordination.
        """
        slots = [slot for slot in TEAMMATE_SLOTS if slot in lineup]
        if not slots:
            raise ValueError(f"no teammate slots in lineup {sorted(lineup)}")
        if mix not in ("hetero", "homo"):
            raise ValueError(f"unknown pool mix {mix!r}; use 'hetero' or 'homo'")

        if mix == "homo":
            common: set[tuple[str, str]] | None = None
            for slot in slots:
                keys = {(e.family, e.tactic) for e in self.for_slot(lineup[slot])}
                common = keys if common is None else (common & keys)
            if common:
                family, tactic = rng.choice(sorted(common))
                return {
                    int(slot): next(
                        e for e in self.for_slot(lineup[slot])
                        if e.family == family and e.tactic == tactic
                    )
                    for slot in slots
                }
            # No uniform tactic exists for this lineup: fall through to per-slot.

        return {
            int(slot): rng.choice(self.for_slot(lineup[slot]))
            for slot in slots
        }

    def named(self, name: str) -> PoolEntry:
        for entry in self.entries:
            if entry.name == name:
                return entry
        raise KeyError(f"no teammate strategy named {name!r}")


class OpponentPool:
    """``team`` strategies, played as the whole opposite side."""

    def __init__(self, entries: list[PoolEntry]):
        self.entries = [e for e in entries if e.mode == "team"]

    def __len__(self) -> int:
        return len(self.entries)

    def sample(self, rng: random.Random, exclude: tuple[str, ...] = ()) -> PoolEntry:
        options = [e for e in self.entries if e.key not in set(exclude)]
        if not options:
            raise ValueError("opponent pool is empty")
        return rng.choice(options)

    def named(self, name: str) -> PoolEntry:
        for entry in self.entries:
            if entry.name == name:
                return entry
        raise KeyError(f"no opponent strategy named {name!r}")

    def names(self) -> list[str]:
        return sorted(e.name for e in self.entries)


def entries_digest(entries: list[PoolEntry]) -> str:
    digest = hashlib.sha256()
    for entry in sorted(entries, key=lambda e: e.key):
        digest.update(f"{entry.key}:{entry.digest}".encode())
    return digest.hexdigest()


def save_pool(
    path: str | Path,
    *,
    teammates: list[PoolEntry],
    opponents: list[PoolEntry],
    held_out: list[PoolEntry] | None = None,
    arena_root: str | Path = "",
    meta: dict | None = None,
) -> dict:
    """Freeze a pool to JSON so a run can be reproduced after the tree changes."""
    payload = {
        "format": POOL_FORMAT,
        "arena_root": str(arena_root),
        "teammates": [e.to_json() for e in teammates],
        "opponents": [e.to_json() for e in opponents],
        "held_out": [e.to_json() for e in (held_out or [])],
        "digest": entries_digest(teammates + opponents + list(held_out or [])),
        "meta": dict(meta or {}),
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return payload


def load_pool(path: str | Path) -> dict:
    """Load a frozen pool and verify its digest."""
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if payload.get("format") != POOL_FORMAT:
        raise ValueError(f"not a {POOL_FORMAT} file: {path}")
    for key in ("teammates", "opponents", "held_out"):
        payload[key] = [PoolEntry.from_json(item) for item in payload.get(key) or []]
    expected = entries_digest(payload["teammates"] + payload["opponents"] + payload["held_out"])
    if payload.get("digest") != expected:
        raise ValueError(f"pool digest mismatch: {payload.get('digest')} != {expected}")
    return payload


def _main() -> None:
    """``python -m sentry_rl.pools [ROOT]``: list what a pool contains."""
    import argparse

    parser = argparse.ArgumentParser(description="List an Arena-1 strategy pool")
    parser.add_argument("root", nargs="?",
                        default=str(Path(__file__).resolve().parents[2] / "Arena-1"))
    parser.add_argument("--held-out", action="store_true", help="show the held-out split too")
    parser.add_argument("--fraction", type=float, default=0.25)
    args = parser.parse_args()

    entries = discover(args.root)
    teammates = [e for e in entries if e.mode == "adaptive"]
    pool, held = split_held_out([e for e in entries if e.mode == "team"], args.fraction)
    print(f"root: {args.root}")
    print(f"adaptive: {len(teammates)}   team: {len(pool)} (+{len(held)} held out)")
    print("teammate robots:", TeammatePool(teammates).robots)
    print("opponents:")
    for entry in pool:
        print(f"  {entry.name:42s} {entry.tactic:8s} {entry.family}")
    if args.held_out:
        print("held out:")
        for entry in held:
            print(f"  {entry.name:42s} {entry.tactic:8s} {entry.family}")


if __name__ == "__main__":
    _main()
