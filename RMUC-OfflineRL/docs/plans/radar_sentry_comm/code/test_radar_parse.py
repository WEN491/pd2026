from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import radar_parse as p  # noqa: E402


class RadarParseTests(unittest.TestCase):
    def test_positions_cm_to_m_and_ids(self):
        data = struct.pack("<12h", 100, 200, 0, 0, 300, 400, 500, 600, 0, 0, 700, 800)
        out = p.parse_0a01(data, p.DEFAULT_RED_ENEMIES)
        self.assertEqual(out[101], (1.0, 2.0))
        self.assertEqual(out[103], (3.0, 4.0))
        self.assertEqual(out[104], (5.0, 6.0))
        self.assertEqual(out[107], (7.0, 8.0))
        out_blue = p.parse_0a01(data, p.DEFAULT_BLUE_ENEMIES)
        self.assertIn(1, out_blue)

    def test_hp_and_ammo(self):
        hp = struct.pack("<6H", 400, 0, 300, 200, 0, 100)
        self.assertEqual(p.parse_0a02(hp), {101: 400, 103: 300, 104: 200, 107: 100})
        ammo = struct.pack("<5H", 300, 250, 200, 0, 150)
        self.assertEqual(p.parse_0a03(ammo), {101: 300, 103: 250, 104: 200, 107: 150})

    def test_aggregator_and_key(self):
        agg = p.RadarAggregator()
        agg.update(0x0A01, struct.pack("<12h", 100, 200, 0, 0, 300, 400, 500, 600, 0, 0, 700, 800), 1000)
        agg.update(0x0A02, struct.pack("<6H", 400, 0, 300, 200, 0, 100), 1000)
        agg.update(0x0A03, struct.pack("<5H", 300, 250, 200, 0, 150), 1000)
        agg.update(0x0A06, b"ABCDEF", 1000)
        snap = agg.to_snapshot()
        self.assertEqual(snap.timestamp_ms, 1000)
        self.assertEqual(snap.key, b"ABCDEF")
        t = snap.get(101)
        self.assertIsNotNone(t)
        assert t is not None
        self.assertEqual((t.x, t.y, t.hp, t.ammo), (1.0, 2.0, 400, 300))
        self.assertTrue(t.position_valid and t.hp_valid and t.ammo_valid)

    def test_bad_lengths(self):
        for fn, data in ((p.parse_0a01, b""), (p.parse_0a02, b"\x00" * 11),
                         (p.parse_0a06, b"12345")):
            with self.assertRaises(p.LinkError):
                fn(data)


if __name__ == "__main__":
    unittest.main()
