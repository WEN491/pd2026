import pytest

from sentry_rl.actions import Intent
from sentry_rl.config import load_config
from sentry_rl.envs.executor import MacroExecutor

INTENTS = (Intent.HOLD_OUTPOST, Intent.HOLD_BASE, Intent.TAKE_OUTPOST,
           Intent.TAKE_BASE, Intent.RESUPPLY, Intent.ENGAGE, Intent.RETREAT)


def _executor(team: str = "red") -> MacroExecutor:
    cfg = load_config()
    cfg["team"] = team
    return MacroExecutor(cfg, sdk=object())


def test_goals_land_inside_the_engine_field():
    """Config goals are centre-origin; the engine is corner-origin 0..W x 0..H."""
    cfg = load_config()
    executor = _executor()
    for intent in INTENTS:
        x, y = executor._goal_for(intent)
        assert 0.0 <= x <= cfg["field"]["width"], (intent.name, x)
        assert 0.0 <= y <= cfg["field"]["height"], (intent.name, y)


def test_own_side_goals_are_on_the_own_half():
    cfg = load_config()
    centre_x = cfg["field"]["center_x"]
    executor = _executor()
    for intent in (Intent.HOLD_OUTPOST, Intent.HOLD_BASE, Intent.RESUPPLY, Intent.RETREAT):
        assert executor._goal_for(intent)[0] < centre_x, intent.name
    for intent in (Intent.TAKE_OUTPOST, Intent.TAKE_BASE):
        assert executor._goal_for(intent)[0] > centre_x, intent.name


def test_blue_goals_are_point_mirrored():
    cfg = load_config()
    red = _executor("red")
    blue = _executor("blue")
    for intent in INTENTS:
        rx, ry = red._goal_for(intent)
        bx, by = blue._goal_for(intent)
        assert bx == pytest.approx(2 * cfg["field"]["center_x"] - rx)
        assert by == pytest.approx(2 * cfg["field"]["center_y"] - ry)
