from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import radar_link as r  # noqa: E402
from radar_state_rx import RadarStateReceiver  # noqa: E402
from sentry_state import SentryStateBuilder  # noqa: E402


class RadarStateRxTests(unittest.TestCase):
    def make_frame(self):
        st = SentryStateBuilder().build(
            1000, sentry_x=1, sentry_y=2, hp=400, ammo=300, posture=1,
            enhanced=False, timers=[1, 2, 3, 4, 5, 6],
            team_positions=[(1, 2), (3, 4), None], last_action_seq=7,
            rx_count=9, crc_errors=0,
        )
        return r.encode_sentry_state(st, r.RED_SENTRY, r.RED_RADAR, 1)

    def test_valid_timeout_and_ack(self):
        rx = RadarStateReceiver(r.RED_SENTRY, r.RED_RADAR, timeout_ms=2000)
        self.assertFalse(rx.state_valid(0))
        self.assertIsNotNone(rx.on_frame(self.make_frame(), 1000))
        self.assertTrue(rx.state_valid(3000))
        self.assertFalse(rx.state_valid(3001))
        self.assertTrue(rx.state_lost(3001))
        self.assertEqual(rx.ack()["last_action_seq"], 7)
        self.assertEqual(rx.ack()["rx_count"], 9)

    def test_wrong_ids_and_crc(self):
        raw = self.make_frame()
        wrong = RadarStateReceiver(r.RED_RADAR, r.RED_SENTRY)
        self.assertIsNone(wrong.on_frame(raw, 1000))
        rx = RadarStateReceiver(r.RED_SENTRY, r.RED_RADAR)
        bad = bytearray(raw); bad[-1] ^= 0xFF
        self.assertIsNone(rx.on_frame(bytes(bad), 1000))
        self.assertEqual(rx.crc_errors, 1)


if __name__ == "__main__":
    unittest.main()
