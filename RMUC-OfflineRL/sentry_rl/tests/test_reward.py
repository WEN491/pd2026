import pytest

from sentry_rl.config import load_config
from sentry_rl.reward import RewardTracker


def _frame(control=0.0, enemy_hp=400.0, own_hp=400.0, enemy_outpost=1000.0,
           status="running", winner=""):
    return {
        "time": 5.0, "status": status, "winner": winner,
        "teams": {"red": {"control": control}, "blue": {"control": 0.0}},
        "robots": [
            {"id": "red_7", "slot": 7, "team": "red", "role": "sentry", "hp": own_hp},
            {"id": "blue_1", "slot": 1, "team": "blue", "role": "heavy", "hp": enemy_hp},
        ],
        "bases": {"red": {"hp": 10000.0}, "blue": {"hp": 10000.0}},
        "outposts": {"red": {"hp": 1000.0}, "blue": {"hp": enemy_outpost}},
        "positions": {
            "red_outpost": {"progress": 0.0}, "blue_outpost": {"progress": 0.0},
            "red_base": {"progress": 0.0}, "blue_base": {"progress": 0.0},
        },
    }


def test_control_and_damage_positive():
    cfg = load_config()
    tracker = RewardTracker(cfg)
    first = _frame()
    tracker.reset(first)
    second = _frame(control=3.0, enemy_hp=380.0, own_hp=390.0, enemy_outpost=900.0)
    reward = tracker.update(second)
    assert reward > 0.0


def test_win_terminal():
    cfg = load_config()
    tracker = RewardTracker(cfg)
    tracker.reset(_frame())
    reward = tracker.update(_frame(control=10.0, status="finished", winner="red"))
    assert reward >= cfg["reward"]["win"]


def test_losing_base_penalty():
    cfg = load_config()
    tracker = RewardTracker(cfg)
    tracker.reset(_frame())
    frame = _frame(status="finished", winner="blue")
    frame["bases"]["red"]["hp"] = 0.0
    reward = tracker.update(frame)
    assert reward < 0.0
