from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import radar_sentry_sentry.radar_link as r  # noqa: E402
from radar_sentry_sentry.app import SentryApp  # noqa: E402
from radar_sentry_sentry.serial_port import FakeTransport  # noqa: E402
from radar_sentry_sentry.state_provider import DemoStateProvider  # noqa: E402


def last_state(data: bytes):
    frames = r.FrameStreamParser().feed(data)
    states = [f for f in frames if r.decode_interaction(f.raw).data_cmd_id == r.SUBCMD_SENTRY_STATE]
    return r.decode_sentry_state(states[-1].raw, 7, 9)


def suggestion(seq=1, action=6, target=101, age=50):
    return r.Suggestion(
        flags=r.FLAG_ACTION_VALID | r.FLAG_TARGET_VALID | r.FLAG_TARGET_ALIVE | r.FLAG_POSITION_VALID,
        action_id=action, action_seq=seq, target_id=target,
        target_x=1.0, target_y=2.0, target_age_ms=age,
        target_quality=2, timestamp_ms=1000,
    )


class SentryAppTests(unittest.TestCase):
    def test_receive_and_state_ack(self):
        tr = FakeTransport()
        app = SentryApp(tr, DemoStateProvider())
        raw = r.encode_suggestion(suggestion(), 9, 7, 0)
        logs = app.step(0, raw)
        self.assertTrue(any("mode=approach" in line for line in logs))
        st = last_state(bytes(tr.written))
        self.assertEqual(st.last_action_seq, 1)
        self.assertEqual(st.rx_count, 1)
        self.assertIsNotNone(st.timers)

    def test_link_loss_and_same_seq_recovery(self):
        tr = FakeTransport()
        app = SentryApp(tr, DemoStateProvider())
        raw = r.encode_suggestion(suggestion(seq=5), 9, 7, 0)
        app.step(0, raw)
        logs = app.step(2100)
        self.assertTrue(any("fallback" in line for line in logs))
        st = last_state(bytes(tr.written))
        self.assertEqual(st.last_action_seq, 0)
        logs = app.step(2200, raw)
        self.assertTrue(any("mode=approach" in line for line in logs))
        self.assertFalse(app.rx.fallback_latched)
        self.assertEqual(app.rx.last_action_seq, 5)

    def test_demo_state_provider(self):
        p = DemoStateProvider()
        s = p.sample(0)
        self.assertEqual((s.hp, s.ammo, s.posture), (400, 300, 1))
        self.assertIsNone(p.visible_target_id())


if __name__ == "__main__":
    unittest.main()
