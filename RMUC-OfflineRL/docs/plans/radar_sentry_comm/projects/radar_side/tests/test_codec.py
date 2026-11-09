from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import radar_sentry_radar.radar_link as r  # noqa: E402


class CodecTests(unittest.TestCase):
    def test_suggestion_size(self):
        s = r.Suggestion(
            flags=r.FLAG_ACTION_VALID | r.FLAG_TARGET_VALID | r.FLAG_POSITION_VALID,
            action_id=6, action_seq=1, target_id=101, target_x=1.0,
            target_y=2.0, target_age_ms=50, target_quality=2, timestamp_ms=1000,
        )
        self.assertEqual(len(s.pack()), 21)
        raw = r.encode_suggestion(s, 9, 7, 0)
        self.assertEqual(len(raw), 36)
        self.assertEqual(r.decode_suggestion(raw, 9, 7), s)

    def test_state_size(self):
        st = r.SentryState(seq=1, timestamp_ms=1000, sentry_x=1, sentry_y=2,
                           hp=400, ammo=300, posture=1, team_valid=0,
                           timers=b"\x00" * 6, team_pos=(0.0,) * 6)
        self.assertEqual(len(st.pack()), 59)
        raw = r.encode_sentry_state(st, 7, 9, 0)
        self.assertEqual(len(raw), 74)
        self.assertEqual(r.decode_sentry_state(raw, 7, 9), st)


if __name__ == "__main__":
    unittest.main()
