from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import radar_link as r  # noqa: E402
from radar_tx import RadarTransmitter  # noqa: E402
from sentry_rx import MODE_APPROACH, SentryReceiver  # noqa: E402
from radar_state_rx import RadarStateReceiver  # noqa: E402
from sentry_state import SentryStateBuilder  # noqa: E402


class LoopbackTests(unittest.TestCase):
    def test_suggestion_loopback_10hz(self):
        tx = RadarTransmitter(r.RED_RADAR, r.RED_SENTRY)
        rx = SentryReceiver(engage_action_id=6)
        for i in range(10):
            s, raw = tx.suggestion_frame(
                i * 100, 6, target_id=101,
                target_x=1.0 + i * 0.01, target_y=2.0,
                target_age_ms=10 + i, target_quality=2,
            )
            d = rx.ingest(r.decode_suggestion(raw, r.RED_RADAR, r.RED_SENTRY), i * 100)
            self.assertEqual(d.mode, MODE_APPROACH)
        self.assertEqual(rx.applied_count, 1)

    def test_state_loopback(self):
        b = SentryStateBuilder()
        st = b.build(
            0, sentry_x=1, sentry_y=2, hp=400, ammo=300, posture=1,
            enhanced=True, timers=[0, 0, 0, 15, 0, 0],
            team_positions=[(1, 2), (3, 4), None], last_action_seq=1,
            rx_count=10, crc_errors=0,
        )
        raw = r.encode_sentry_state(st, r.RED_SENTRY, r.RED_RADAR, 0)
        rx = RadarStateReceiver(r.RED_SENTRY, r.RED_RADAR)
        self.assertIsNotNone(rx.on_frame(raw, 0))
        self.assertTrue(rx.state_valid(2000))
        self.assertFalse(rx.state_valid(2001))
        self.assertEqual(rx.ack()["last_action_seq"], 1)
        self.assertEqual(rx.ack()["rx_count"], 10)


if __name__ == "__main__":
    unittest.main()
