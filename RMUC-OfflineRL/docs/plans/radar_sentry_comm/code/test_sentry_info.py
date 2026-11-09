from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sentry_info as si  # noqa: E402


class SentryInfoTests(unittest.TestCase):
    def test_0x020d(self):
        word0 = 100 | (2 << 11) | (1 << 15) | (1 << 19) | (1 << 20) | (50 << 21)
        word1 = 1 | (50 << 1) | (1 << 12) | (1 << 14) | (1 << 15)
        timers = [1, 2, 3, 4, 5, 6]
        word2 = sum(v << (8 * i) for i, v in enumerate(timers))
        info = si.parse_0x020d(struct.pack("<IHQ", word0, word1, word2))
        self.assertEqual(info.exchanged_ammo, 100)
        self.assertEqual(info.remote_ammo_exchanges, 2)
        self.assertEqual(info.remote_hp_exchanges, 1)
        self.assertTrue(info.free_revive_available)
        self.assertTrue(info.instant_revive_available)
        self.assertEqual(info.instant_revive_cost, 50)
        self.assertTrue(info.out_of_combat)
        self.assertEqual(info.exchangeable_ammo, 50)
        self.assertEqual(info.posture, 1)
        self.assertTrue(info.rune_activatable)
        self.assertTrue(info.enhanced)
        self.assertEqual(info.timers, tuple(timers))

    def test_0x0208(self):
        allowance = si.parse_0x0208(struct.pack("<4H", 300, 0, 120, 50))
        self.assertEqual(allowance.projectile_17mm, 300)
        self.assertEqual(allowance.coins, 120)
        self.assertEqual(allowance.fortress_reserve, 50)

    def test_bad_lengths(self):
        with self.assertRaises(si.LinkError):
            si.parse_0x020d(b"\x00" * 13)
        with self.assertRaises(si.LinkError):
            si.parse_0x0208(b"\x00" * 7)


if __name__ == "__main__":
    unittest.main()
