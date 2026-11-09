from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import radar_link as r  # noqa: E402
import sentry_rx as s  # noqa: E402


def sug(seq=1, action=6, target=101, age=50, alive=True, valid=True, ts=1000):
    flags = r.FLAG_ACTION_VALID
    if valid:
        flags |= r.FLAG_TARGET_VALID | r.FLAG_POSITION_VALID
        if alive:
            flags |= r.FLAG_TARGET_ALIVE
    return r.Suggestion(
        version=1, flags=flags, action_id=action, action_seq=seq,
        target_id=target if valid else 0,
        target_x=1.0, target_y=2.0, target_age_ms=age,
        target_quality=2, timestamp_ms=ts,
    )


class SentryRxTests(unittest.TestCase):
    def test_approach_search_target_lost(self):
        rx = s.SentryReceiver(engage_action_id=6, link_timeout_ms=10000)
        d = rx.ingest(sug(), 1000, visible_target_id=None)
        self.assertEqual(d.mode, s.MODE_APPROACH)
        self.assertEqual(rx.applied_count, 1)
        self.assertEqual(rx.tick(1250).mode, s.MODE_APPROACH)
        self.assertEqual(rx.tick(1300).mode, s.MODE_SEARCH)
        d = rx.tick(3000)
        self.assertEqual(d.mode, s.MODE_FALLBACK)
        self.assertEqual(d.reason, "target_lost")
        self.assertTrue(rx.fallback_latched)
        self.assertIsNone(rx.last_action_seq)

    def test_link_loss_vision_then_fallback(self):
        rx = s.SentryReceiver(engage_action_id=6)
        rx.ingest(sug(), 1000)
        d = rx.tick(2100, visible_target_id=101)
        self.assertEqual(d.mode, s.MODE_VISION)
        self.assertFalse(d.fallback)
        d = rx.tick(2100, visible_target_id=None)
        self.assertEqual(d.mode, s.MODE_FALLBACK)
        self.assertEqual(d.reason, "link_lost")

    def test_non_engage_link_loss_is_fallback(self):
        rx = s.SentryReceiver(engage_action_id=6)
        d = rx.ingest(sug(action=1, valid=False), 1000)
        self.assertEqual(d.mode, s.MODE_ACTION)
        d = rx.tick(2001)
        self.assertEqual(d.mode, s.MODE_FALLBACK)
        self.assertEqual(d.reason, "link_lost")

    def test_recovery_same_seq_reapplies(self):
        rx = s.SentryReceiver(engage_action_id=6)
        rx.ingest(sug(seq=5), 1000)
        rx.tick(2100)  # fallback link_lost
        self.assertTrue(rx.fallback_latched)
        before = rx.applied_count
        d = rx.ingest(sug(seq=5, age=10), 2200)
        self.assertTrue(d.recovered)
        self.assertEqual(d.mode, s.MODE_APPROACH)
        self.assertEqual(rx.applied_count, before + 1)
        self.assertFalse(rx.fallback_latched)

    def test_same_seq_does_not_reapply(self):
        rx = s.SentryReceiver(engage_action_id=6)
        for i in range(10):
            d = rx.ingest(sug(seq=1, age=10 + i), 1000 + i * 100)
            self.assertEqual(d.mode, s.MODE_APPROACH)
        self.assertEqual(rx.applied_count, 1)

    def test_action_change_reapplies(self):
        rx = s.SentryReceiver(engage_action_id=6)
        rx.ingest(sug(seq=1, action=6), 1000)
        d = rx.ingest(sug(seq=2, action=1, valid=False), 2000)
        self.assertEqual(d.mode, s.MODE_ACTION)
        self.assertEqual(rx.applied_count, 2)

    def test_invalid_target_with_visible(self):
        rx = s.SentryReceiver(engage_action_id=6)
        d = rx.ingest(sug(valid=False), 1000, visible_target_id=101)
        self.assertEqual(d.mode, s.MODE_VISION)
        self.assertEqual(d.target_id, 101)
        self.assertFalse(d.fallback)

    def test_dead_target_without_vision(self):
        rx = s.SentryReceiver(engage_action_id=6)
        d = rx.ingest(sug(alive=False), 1000)
        self.assertEqual(d.mode, s.MODE_FALLBACK)
        self.assertEqual(d.reason, "target_invalid")


if __name__ == "__main__":
    unittest.main()
