import pytest

from sentry_rl.radar import RadarChannel, RadarConfig


def _frame(x=2.0, y=3.0, t=0.0):
    return {"time": t, "robots": [
        {"slot": 1, "team": "blue", "role": "heavy", "alive": True, "x": x, "y": y},
        {"slot": 7, "team": "red", "role": "sentry", "alive": True, "x": 0.0, "y": 0.0},
    ]}


def test_synthetic_latency_and_link():
    radar = RadarChannel(RadarConfig(mode="synthetic", latency_s=0.15, dropout=0.0,
                                     sigma_m=0.0, rate_hz=5.0), "red", seed=0)
    radar.update(_frame(), 0.0)
    assert radar.measure(0.0)["enemies"] == {}          # still in flight
    later = radar.measure(0.2)
    assert later["link"] is True
    assert "1" in later["enemies"]
    assert later["enemies"]["1"]["known"] is True


def test_dropout_can_remove_all():
    radar = RadarChannel(RadarConfig(mode="synthetic", dropout=1.0, rate_hz=20.0), "red", 0)
    for t in (0.0, 0.1, 0.2):
        radar.update(_frame(t=t), t)
    assert radar.measure(1.0)["enemies"] == {}


def test_oracle_is_exact_and_immediate():
    radar = RadarChannel(RadarConfig(mode="oracle", rate_hz=5.0), "red", 0)
    radar.update(_frame(x=4.0, y=5.0), 0.0)
    out = radar.measure(0.0)
    assert out["link"] is True
    assert out["enemies"]["1"]["x"] == pytest.approx(4.0)
    assert out["enemies"]["1"]["y"] == pytest.approx(5.0)


def test_none_mode_empty():
    radar = RadarChannel(RadarConfig(mode="none"), "red", 0)
    radar.update(_frame(), 0.0)
    assert radar.measure(1.0) == {"enemies": {}, "link": False, "age": 1.0}
