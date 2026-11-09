from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import radar_sentry_radar.radar_link as r  # noqa: E402
from radar_sentry_radar.app import RadarApp  # noqa: E402
from radar_sentry_radar.serial_port import FakeTransport  # noqa: E402
from radar_sentry_radar.source import DemoSource  # noqa: E402


def frames(data: bytes):
    return r.FrameStreamParser().feed(data)


def suggestions(data: bytes):
    out = []
    for f in frames(data):
        it = r.decode_interaction(f.raw, 9, 7)
        if it.data_cmd_id == r.SUBCMD_SUGGESTION:
            out.append(r.decode_suggestion(f.raw, 9, 7))
    return out


class RadarAppTests(unittest.TestCase):
    def test_step_writes_suggestion_and_status(self):
        tr = FakeTransport()
        app = RadarApp(tr, DemoSource())
        app.step(0)
        fs = frames(bytes(tr.written))
        self.assertEqual(len(fs), 2)
        self.assertEqual(fs[0].cmd_id, r.CMD_INTERACTION)
        s = r.decode_suggestion(fs[0].raw, 9, 7)
        self.assertEqual((s.action_id, s.action_seq, s.target_id), (6, 1, 101))
        st = r.decode_radar_status(fs[1].raw, 9, 7)
        self.assertEqual(st.status, r.STATUS_OK)

    def test_action_1hz_and_target_10hz(self):
        tr = FakeTransport()
        app = RadarApp(tr, DemoSource())
        app.step(0)
        app.step(100)
        app.step(100)
        ss = suggestions(bytes(tr.written))
        s1 = ss[0]
        self.assertEqual(ss[0].action_seq, ss[1].action_seq)
        app.step(5000)
        ss = suggestions(bytes(tr.written))
        last = ss[-1]
        self.assertNotEqual(last.action_seq, s1.action_seq)
        self.assertEqual(last.action_id, 1)

    def test_state_rx(self):
        tr = FakeTransport()
        app = RadarApp(tr, DemoSource())
        st = r.SentryState(seq=1, timestamp_ms=1000, sentry_x=1, sentry_y=2,
                           hp=400, ammo=300, posture=1, team_valid=0,
                           timers=b"\x00" * 6, team_pos=(0.0,) * 6,
                           last_action_seq=1)
        raw = r.encode_sentry_state(st, 7, 9, 0)
        app.step(100, raw)
        self.assertIsNotNone(app.state_rx.last_state)
        self.assertEqual(app.state_rx.last_state.last_action_seq, 1)


if __name__ == "__main__":
    unittest.main()
