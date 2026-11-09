from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import radar_link as r  # noqa: E402


class StreamTests(unittest.TestCase):
    def test_split_and_garbage(self):
        a = r.encode_frame(0x0301, b"aaaa", 1)
        b = r.encode_frame(0x0301, b"bbbb", 2)
        p = r.FrameStreamParser()
        self.assertEqual(p.feed(b"junk" + a[:3]), [])
        self.assertEqual(p.feed(a[3:] + b"junk" + b), [r.decode_frame(a), r.decode_frame(b)])
        self.assertEqual(p.buf, b"")

    def test_bad_frame_resync(self):
        good = r.encode_frame(0x0301, b"good", 1)
        bad = bytearray(good); bad[-1] ^= 0xFF
        p = r.FrameStreamParser()
        out = p.feed(bytes(bad) + good)
        self.assertEqual(out, [r.decode_frame(good)])


if __name__ == "__main__":
    unittest.main()
