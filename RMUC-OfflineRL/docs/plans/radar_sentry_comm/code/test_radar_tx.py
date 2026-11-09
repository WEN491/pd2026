from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import radar_link as r  # noqa: E402
from radar_tx import RadarStatusBuilder, RadarTransmitter, SuggestionBuilder  # noqa: E402


class RadarTxTests(unittest.TestCase):
    def test_action_seq_and_1hz(self):
        b = SuggestionBuilder()
        s1 = b.build(0, 6, target_id=101, target_x=1, target_y=2, target_age_ms=50, target_quality=2)
        self.assertEqual(s1.action_seq, 1)
        self.assertTrue(s1.target_valid and s1.position_valid and s1.target_alive)
        s2 = b.build(100, 6, target_id=101, target_x=1.1, target_y=2.1, target_age_ms=40, target_quality=2)
        self.assertEqual(s2.action_seq, 1)
        s3 = b.build(1000, 1)
        self.assertEqual(s3.action_seq, 2)
        with self.assertRaises(r.LinkError):
            b.build(1200, 6)
        s4 = b.build(2000, 6, target_id=101, target_x=1, target_y=2)
        self.assertEqual(s4.action_seq, 3)

    def test_target_flags_and_stale(self):
        b = SuggestionBuilder()
        s = b.build(0, 6, target_id=101, target_x=1, target_y=2, target_age_ms=400)
        self.assertTrue(s.stale)
        b2 = SuggestionBuilder()
        s0 = b2.build(0, 6)
        self.assertFalse(s0.target_valid)
        self.assertEqual(s0.target_id, 0)
        with self.assertRaises(r.LinkError):
            b.build(2000, 9)
        with self.assertRaises(r.LinkError):
            b.build(2000, 6, target_id=2)

    def test_transmitter_and_status(self):
        tx = RadarTransmitter(r.RED_RADAR, r.RED_SENTRY)
        s, raw = tx.suggestion_frame(0, 6, target_id=101, target_x=1, target_y=2, target_age_ms=50)
        self.assertEqual(len(raw), 36)
        self.assertEqual(r.decode_suggestion(raw, r.RED_RADAR, r.RED_SENTRY), s)
        st = RadarStatusBuilder().build(1000, status=r.STATUS_OK, tx_count=1)
        raw2 = tx.status_frame(st)
        self.assertEqual(len(raw2), 27)
        self.assertEqual(r.decode_radar_status(raw2, r.RED_RADAR, r.RED_SENTRY), st)


if __name__ == "__main__":
    unittest.main()
