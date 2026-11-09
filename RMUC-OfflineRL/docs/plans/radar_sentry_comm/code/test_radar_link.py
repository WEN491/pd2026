import binascii
import math
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import radar_link as r  # noqa: E402


class FrameTests(unittest.TestCase):
    def test_crc16_ccitt_false(self):
        self.assertEqual(r.crc16(b"123456789"), 0x29B1)
        for data in (b"", b"\x00", b"\xA5\x00\x00\x01", b"abc" * 40):
            self.assertEqual(r.crc16(data), binascii.crc_hqx(data, 0xFFFF))

    def test_frame_roundtrip(self):
        raw = r.encode_frame(0x0301, b"hello", seq=7)
        f = r.decode_frame(raw)
        self.assertEqual(f.cmd_id, 0x0301)
        self.assertEqual(f.data, b"hello")
        self.assertEqual(f.seq, 7)
        self.assertEqual(raw[0], r.SOF)
        self.assertEqual(struct.unpack_from("<H", raw, 1)[0], 5)
        self.assertEqual(raw[4], r.crc8(raw[:4]))
        self.assertEqual(struct.unpack_from("<H", raw, len(raw) - 2)[0],
                         binascii.crc_hqx(raw[:-2], 0xFFFF))

    def test_bad_frames(self):
        raw = bytearray(r.encode_frame(0x0301, b"hello"))
        for mutate in (
            lambda b: b.__setitem__(0, 0x00),
            lambda b: b.__setitem__(4, (b[4] + 1) & 0xFF),
            lambda b: b.__setitem__(-1, (b[-1] + 1) & 0xFF),
            lambda b: b.__setitem__(1, (b[1] + 1) & 0xFF),
        ):
            bad = bytearray(raw)
            mutate(bad)
            with self.assertRaises(r.LinkError):
                r.decode_frame(bytes(bad))

    def test_interaction_header_and_ids(self):
        raw = r.encode_interaction(r.SUBCMD_SUGGESTION, r.RED_RADAR, r.RED_SENTRY, b"x", 1)
        it = r.decode_interaction(raw, r.RED_RADAR, r.RED_SENTRY)
        self.assertEqual(it.data_cmd_id, r.SUBCMD_SUGGESTION)
        self.assertEqual(it.sender_id, r.RED_RADAR)
        self.assertEqual(it.receiver_id, r.RED_SENTRY)
        self.assertEqual(it.user_data, b"x")
        with self.assertRaises(r.LinkError):
            r.decode_interaction(raw, r.RED_SENTRY, r.RED_RADAR)

    def test_user_data_limit(self):
        raw = r.encode_interaction(r.SUBCMD_SUGGESTION, 9, 7, b"x" * 112)
        self.assertEqual(len(raw), 127)
        with self.assertRaises(r.LinkError):
            r.encode_interaction(r.SUBCMD_SUGGESTION, 9, 7, b"x" * 113)


class SuggestionTests(unittest.TestCase):
    def make(self, **kw):
        args = dict(
            version=1,
            flags=r.FLAG_ACTION_VALID | r.FLAG_TARGET_VALID | r.FLAG_TARGET_ALIVE | r.FLAG_POSITION_VALID,
            action_id=6,
            action_seq=3,
            target_id=101,
            target_x=1.5,
            target_y=-2.5,
            target_age_ms=50,
            target_quality=2,
            timestamp_ms=1000,
        )
        args.update(kw)
        return r.Suggestion(**args)

    def test_size_and_roundtrip(self):
        s = self.make()
        raw = s.pack()
        self.assertEqual(len(raw), 21)
        self.assertEqual(r.Suggestion.unpack(raw), s)
        frame = r.encode_suggestion(s, r.RED_RADAR, r.RED_SENTRY, 7)
        self.assertEqual(len(frame), 36)
        self.assertEqual(r.decode_suggestion(frame, r.RED_RADAR, r.RED_SENTRY), s)

    def test_no_target(self):
        s = self.make(flags=r.FLAG_ACTION_VALID, target_id=0, target_x=0, target_y=0,
                      target_age_ms=0, target_quality=0)
        self.assertEqual(r.Suggestion.unpack(s.pack()), s)
        with self.assertRaises(r.LinkError):
            self.make(flags=r.FLAG_ACTION_VALID, target_id=101).validate()

    def test_bad_action(self):
        with self.assertRaises(r.LinkError):
            self.make(action_id=0).validate()
        with self.assertRaises(r.LinkError):
            self.make(action_id=9).validate()
        # action_valid=0 allows 0, but not 9
        self.make(flags=0, action_id=0, target_id=0).validate()
        with self.assertRaises(r.LinkError):
            self.make(flags=0, action_id=9, target_id=0).validate()

    def test_bad_target_and_float(self):
        with self.assertRaises(r.LinkError):
            self.make(target_id=2).validate()
        with self.assertRaises(r.LinkError):
            self.make(target_x=math.nan).validate()
        with self.assertRaises(r.LinkError):
            self.make(target_y=math.inf).validate()
        with self.assertRaises(r.LinkError):
            self.make(timestamp_ms=420001).validate()

    def test_flags(self):
        s = self.make()
        self.assertTrue(s.action_valid and s.target_valid and s.target_alive and s.position_valid)
        self.assertFalse(s.stale)
        s2 = self.make(flags=s.flags | r.FLAG_STALE)
        self.assertTrue(s2.stale)


class StateTests(unittest.TestCase):
    def test_status_roundtrip(self):
        s = r.RadarStatus(version=1, status=r.STATUS_DEGRADED, seq=2,
                          timestamp_ms=1000, tx_count=10, drop_count=1)
        self.assertEqual(len(s.pack()), 12)
        self.assertEqual(r.RadarStatus.unpack(s.pack()), s)
        frame = r.encode_radar_status(s, 9, 7, 1)
        self.assertEqual(len(frame), 27)
        self.assertEqual(r.decode_radar_status(frame, 9, 7), s)

    def test_sentry_state_roundtrip(self):
        s = r.SentryState(
            version=1, flags=5, seq=1, timestamp_ms=1000,
            last_action_seq=9, rx_count=10, crc_errors=1,
            sentry_x=1.25, sentry_y=2.5, hp=400, ammo=300,
            posture=1, enhanced=1,
            timers=bytes([1, 2, 3, 4, 5, 6]),
            team_valid=0b101,
            team_pos=(1.0, 2.0, 3.0, 4.0, 0.0, 0.0),
        )
        raw = s.pack()
        self.assertEqual(len(raw), 59)
        self.assertEqual(r.SentryState.unpack(raw), s)
        frame = r.encode_sentry_state(s, 7, 9, 1)
        self.assertEqual(len(frame), 74)
        self.assertEqual(r.decode_sentry_state(frame, 7, 9), s)

    def test_sentry_state_bad(self):
        s = r.SentryState(posture=4)
        with self.assertRaises(r.LinkError):
            s.validate()
        s = r.SentryState(team_valid=8)
        with self.assertRaises(r.LinkError):
            s.validate()
        s = r.SentryState(timers=b"\x00" * 5)
        with self.assertRaises(r.LinkError):
            s.validate()


if __name__ == "__main__":
    unittest.main()
