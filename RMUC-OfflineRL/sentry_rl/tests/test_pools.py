import json
from pathlib import Path

import pytest

from sentry_rl.pools import (
    OpponentPool,
    TeammatePool,
    discover,
    entries_digest,
    load_pool,
    save_pool,
    split_held_out,
)

ARENA_EXAMPLES = Path(__file__).resolve().parents[2] / "Arena-1" / "examples"


def _write_example(root: Path, name: str, *, mode: str, robot=None,
                   tactic: str = "balanced", season: str = "2027",
                   api_version: int = 3) -> Path:
    directory = root / name
    directory.mkdir(parents=True)
    source = directory / "strategy.py"
    source.write_text(f'TACTIC = "{tactic}"\n\ndef act(obs, state):\n    return {{}}, state\n',
                      encoding="utf-8")
    entries = {"team": "strategy.py"} if mode == "team" else {str(robot): "strategy.py"}
    (directory / "manifest.json").write_text(json.dumps({
        "api_version": api_version,
        "mode": mode,
        "robot": robot,
        "rules_profile": f"rmuc-{season}-v1.0.0-arena-v1",
        "version": f"{name}-v1",
        "entries": entries,
    }), encoding="utf-8")
    return directory


@pytest.fixture()
def fake_examples(tmp_path: Path) -> Path:
    root = tmp_path / "examples"
    root.mkdir()
    for robot in (2711, 2731, 2732, 2761):
        _write_example(root, f"claude_2027_adaptive_{robot}", mode="adaptive", robot=robot)
        _write_example(root, f"claude_2027_adaptive_{robot}_guard", mode="adaptive",
                       robot=robot, tactic="guard")
    _write_example(root, "gpt61_2027_team", mode="team")
    _write_example(root, "gpt61_2027_team_guard", mode="team", tactic="guard")
    _write_example(root, "old_2026_team", mode="team", season="2026")
    _write_example(root, "stale_2027_team", mode="team", api_version=1)
    return root


def test_discover_filters_profile_and_api_version(fake_examples: Path):
    entries = discover(fake_examples)
    names = {e.name for e in entries}
    assert "old_2026_team" not in names
    assert "stale_2027_team" not in names
    assert names == {
        "claude_2027_adaptive_2711", "claude_2027_adaptive_2711_guard",
        "claude_2027_adaptive_2731", "claude_2027_adaptive_2731_guard",
        "claude_2027_adaptive_2732", "claude_2027_adaptive_2732_guard",
        "claude_2027_adaptive_2761", "claude_2027_adaptive_2761_guard",
        "gpt61_2027_team", "gpt61_2027_team_guard",
    }
    entry = next(e for e in entries if e.name == "claude_2027_adaptive_2711_guard")
    assert entry.tactic == "guard"
    assert entry.family == "claude"
    assert entry.robot == 2711
    assert entry.mode == "adaptive"
    assert entry.version == "claude_2027_adaptive_2711_guard-v1"
    assert len(entry.digest) == 64


def test_split_held_out_is_deterministic_and_disjoint(fake_examples: Path):
    entries = [e for e in discover(fake_examples) if e.mode == "team"]
    pool_a, held_a = split_held_out(entries, fraction=0.5, salt="exam")
    pool_b, held_b = split_held_out(entries, fraction=0.5, salt="exam")
    assert [e.key for e in pool_a] == [e.key for e in pool_b]
    assert [e.key for e in held_a] == [e.key for e in held_b]
    assert not ({e.key for e in pool_a} & {e.key for e in held_a})
    assert len(pool_a) + len(held_a) == len(entries)


def test_teammate_pool_respects_slot_models(fake_examples: Path):
    entries = discover(fake_examples)
    pool = TeammatePool([e for e in entries if e.mode == "adaptive"])
    lineup = {"1": 2711, "3": 2731, "4": 2732, "6": 2761, "7": 2771}
    assert pool.robots == [2711, 2731, 2732, 2761]

    import random
    picked = pool.sample(lineup, random.Random(7), mix="hetero")
    assert set(picked) == {1, 3, 4, 6}
    for slot, entry in picked.items():
        assert entry.robot == lineup[str(slot)]

    homo = pool.sample(lineup, random.Random(7), mix="homo")
    assert len({e.tactic for e in homo.values()}) == 1

    with pytest.raises(KeyError):
        pool.for_slot(2771)


def test_opponent_pool_and_digest(fake_examples: Path):
    entries = discover(fake_examples)
    pool = OpponentPool(entries)
    assert pool.names() == ["gpt61_2027_team", "gpt61_2027_team_guard"]

    import random
    first = pool.sample(random.Random(3))
    assert first.mode == "team"

    same = entries_digest(entries)
    assert same == entries_digest(list(reversed(entries)))
    assert same != entries_digest(entries[:-1])


def test_freeze_roundtrip(tmp_path: Path, fake_examples: Path):
    entries = discover(fake_examples)
    teammates = [e for e in entries if e.mode == "adaptive"][:4]
    opponents = [e for e in entries if e.mode == "team"]
    path = tmp_path / "pool.json"
    payload = save_pool(path, teammates=teammates, opponents=opponents,
                        held_out=[], arena_root=fake_examples)
    loaded = load_pool(path)
    assert loaded["digest"] == payload["digest"]
    assert [e.key for e in loaded["opponents"]] == [e.key for e in opponents]

    tampered = json.loads(path.read_text(encoding="utf-8"))
    tampered["opponents"][0]["digest"] = "0" * 64
    path.write_text(json.dumps(tampered), encoding="utf-8")
    with pytest.raises(ValueError, match="digest mismatch"):
        load_pool(path)


@pytest.mark.skipif(not ARENA_EXAMPLES.is_dir(), reason="Arena-1 examples not vendored")
def test_real_arena_pool_covers_the_lineup():
    entries = discover(ARENA_EXAMPLES)
    teammates = [e for e in entries if e.mode == "adaptive"]
    opponents = [e for e in entries if e.mode == "team"]
    assert len(opponents) >= 15, len(opponents)
    by_robot = {e.robot for e in teammates}
    assert {2711, 2731, 2732, 2761, 2771} <= by_robot
    for robot in (2711, 2731, 2732, 2761, 2771):
        tactics = {e.tactic for e in teammates if e.robot == robot}
        assert {"balanced", "assault", "guard"} <= tactics


@pytest.mark.skipif(not ARENA_EXAMPLES.is_dir(), reason="Arena-1 examples not vendored")
def test_env_samples_composition_without_engine(monkeypatch):
    from sentry_rl.config import load_config
    from sentry_rl.envs.arena1 import Arena1SentryEnv

    monkeypatch.setattr(Arena1SentryEnv, "reset", lambda self, seed=None: None)
    cfg = load_config()
    entries = discover(ARENA_EXAMPLES)
    env = Arena1SentryEnv(
        cfg,
        teammate_pool=TeammatePool(entries),
        opponent_pool=OpponentPool(entries),
        pool_mix="homo",
        pool_seed=11,
    )
    env._episode = 0
    env._sample_composition()
    assert set(env.composition["teammates"]) == {"1", "3", "4", "6"}
    assert env.composition["opponent"].split("@")[0] in OpponentPool(entries).names()
    assert env.teammates is not None and set(env.teammates) == {1, 3, 4, 6}
    for slot, policy in env.teammates.items():
        assert policy.source.is_file()
        assert env.composition["teammates"][str(slot)].split("@")[0] == policy.source.parent.name
