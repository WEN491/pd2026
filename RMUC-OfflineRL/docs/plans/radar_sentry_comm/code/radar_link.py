"""Radar <-> sentry 0x0301 codec, ICD v1.0-draft.

Pure stdlib. No ROS dependency. All multi-byte fields are little-endian.
The byte layouts are the single source of truth in ``ICD.md``; do not add
fields here without bumping the ICD version.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import struct
from typing import Iterable, Optional, Sequence

SOF = 0xA5
CMD_INTERACTION = 0x0301
SUBCMD_SUGGESTION = 0x0210
SUBCMD_RADAR_STATUS = 0x0211
SUBCMD_SENTRY_STATE = 0x0220

MAX_USER_DATA = 112
MAX_FRAME_DATA = 118  # 6 B interaction header + 112 B user data
MAX_FRAME_TOTAL = 127

RED_SENTRY = 7
RED_RADAR = 9
BLUE_SENTRY = 107
BLUE_RADAR = 109

TARGET_IDS = frozenset({1, 3, 4, 7, 101, 103, 104, 107})
ACTION_IDS = frozenset(range(1, 9))
MAX_MATCH_MS = 420_000

# 0x0210 flags
FLAG_ACTION_VALID = 1 << 0
FLAG_TARGET_VALID = 1 << 1
FLAG_TARGET_ALIVE = 1 << 2
FLAG_POSITION_VALID = 1 << 3
FLAG_STALE = 1 << 4

# 0x0211 status
STATUS_OK = 0
STATUS_DEGRADED = 1
STATUS_RF_LOST = 2


class LinkError(ValueError):
    """Malformed frame / payload."""


def _u8(v: int, name: str) -> int:
    if not isinstance(v, int) or not 0 <= v <= 0xFF:
        raise LinkError(f"{name} must be uint8, got {v!r}")
    return v


def _u16(v: int, name: str) -> int:
    if not isinstance(v, int) or not 0 <= v <= 0xFFFF:
        raise LinkError(f"{name} must be uint16, got {v!r}")
    return v


def _u32(v: int, name: str) -> int:
    if not isinstance(v, int) or not 0 <= v <= 0xFFFFFFFF:
        raise LinkError(f"{name} must be uint32, got {v!r}")
    return v


def _finite(v: float, name: str) -> float:
    if not isinstance(v, (int, float)) or not math.isfinite(float(v)):
        raise LinkError(f"{name} must be a finite number, got {v!r}")
    return float(v)


def crc8(data: bytes) -> int:
    """CRC8, poly 0x31, init 0xFF, no reflection/xorout."""
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def crc16(data: bytes) -> int:
    """CRC16-CCITT-FALSE, poly 0x1021, init 0xFFFF, no reflection/xorout."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    cmd_id: int
    data: bytes
    seq: int
    raw: bytes


def encode_frame(cmd_id: int, data: bytes, seq: int = 0) -> bytes:
    _u16(cmd_id, "cmd_id")
    _u8(seq, "seq")
    if len(data) > MAX_FRAME_DATA:
        raise LinkError(f"frame data too long: {len(data)} > {MAX_FRAME_DATA}")
    header = bytes([SOF]) + struct.pack("<H", len(data)) + bytes([seq & 0xFF])
    header += bytes([crc8(header)])
    body = header + struct.pack("<H", cmd_id) + data
    return body + struct.pack("<H", crc16(body))


def decode_frame(raw: bytes) -> Frame:
    if len(raw) < 9:
        raise LinkError(f"frame too short: {len(raw)}")
    if len(raw) > MAX_FRAME_TOTAL:
        raise LinkError(f"frame too long: {len(raw)}")
    if raw[0] != SOF:
        raise LinkError(f"bad SOF: 0x{raw[0]:02X}")
    data_len = struct.unpack_from("<H", raw, 1)[0]
    if data_len > MAX_FRAME_DATA:
        raise LinkError(f"bad data_length: {data_len}")
    if len(raw) != 9 + data_len:
        raise LinkError(f"length mismatch: frame={len(raw)} data={data_len}")
    if crc8(raw[:4]) != raw[4]:
        raise LinkError("header CRC8 mismatch")
    expected_tail = struct.unpack_from("<H", raw, len(raw) - 2)[0]
    if crc16(raw[:-2]) != expected_tail:
        raise LinkError("frame CRC16 mismatch")
    cmd_id = struct.unpack_from("<H", raw, 5)[0]
    return Frame(cmd_id=cmd_id, data=raw[7:7 + data_len], seq=raw[3], raw=raw)


class FrameStreamParser:
    """Incremental parser for a serial byte stream.

    Feed arbitrary chunks; returns complete frames and drops garbage/bad
    frames one byte at a time. This is intentionally independent of ROS.
    """

    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self.buf.extend(data)
        out: list[Frame] = []
        while True:
            start = self.buf.find(SOF)
            if start < 0:
                self.buf.clear()
                break
            if start:
                del self.buf[:start]
            if len(self.buf) < 5:
                break
            data_len = struct.unpack_from("<H", self.buf, 1)[0]
            if data_len > MAX_FRAME_DATA:
                del self.buf[0]
                continue
            total = 9 + data_len
            if len(self.buf) < total:
                break
            raw = bytes(self.buf[:total])
            try:
                frame = decode_frame(raw)
            except LinkError:
                del self.buf[0]
                continue
            out.append(frame)
            del self.buf[:total]
        return out


@dataclass(frozen=True)
class Interaction:
    data_cmd_id: int
    sender_id: int
    receiver_id: int
    user_data: bytes
    frame_seq: int


def encode_interaction(
    data_cmd_id: int,
    sender_id: int,
    receiver_id: int,
    user_data: bytes,
    frame_seq: int = 0,
) -> bytes:
    _u16(data_cmd_id, "data_cmd_id")
    _u16(sender_id, "sender_id")
    _u16(receiver_id, "receiver_id")
    if len(user_data) > MAX_USER_DATA:
        raise LinkError(f"user_data too long: {len(user_data)}")
    data = struct.pack("<HHH", data_cmd_id, sender_id, receiver_id) + user_data
    return encode_frame(CMD_INTERACTION, data, frame_seq)


def decode_interaction(
    raw: bytes,
    expected_sender: Optional[int] = None,
    expected_receiver: Optional[int] = None,
) -> Interaction:
    frame = decode_frame(raw)
    if frame.cmd_id != CMD_INTERACTION:
        raise LinkError(f"not 0x0301: 0x{frame.cmd_id:04X}")
    if len(frame.data) < 6:
        raise LinkError("interaction data shorter than 6 B header")
    data_cmd_id, sender_id, receiver_id = struct.unpack_from("<HHH", frame.data, 0)
    if expected_sender is not None and sender_id != expected_sender:
        raise LinkError(f"unexpected sender {sender_id}, want {expected_sender}")
    if expected_receiver is not None and receiver_id != expected_receiver:
        raise LinkError(f"unexpected receiver {receiver_id}, want {expected_receiver}")
    user_data = frame.data[6:]
    if len(user_data) > MAX_USER_DATA:
        raise LinkError("user_data too long")
    return Interaction(data_cmd_id, sender_id, receiver_id, user_data, frame.seq)


# ---------------------------------------------------------------------------
# 0x0210 Suggestion (21 B)
# ---------------------------------------------------------------------------

_SUGGESTION = struct.Struct("<BBBHBffHBI")


@dataclass
class Suggestion:
    version: int = 1
    flags: int = 0
    action_id: int = 0
    action_seq: int = 0
    target_id: int = 0
    target_x: float = 0.0
    target_y: float = 0.0
    target_age_ms: int = 0
    target_quality: int = 0
    timestamp_ms: int = 0

    @property
    def action_valid(self) -> bool:
        return bool(self.flags & FLAG_ACTION_VALID)

    @property
    def target_valid(self) -> bool:
        return bool(self.flags & FLAG_TARGET_VALID)

    @property
    def target_alive(self) -> bool:
        return bool(self.flags & FLAG_TARGET_ALIVE)

    @property
    def position_valid(self) -> bool:
        return bool(self.flags & FLAG_POSITION_VALID)

    @property
    def stale(self) -> bool:
        return bool(self.flags & FLAG_STALE)

    def pack(self) -> bytes:
        self.validate()
        return _SUGGESTION.pack(
            self.version,
            self.flags,
            self.action_id,
            self.action_seq,
            self.target_id,
            float(self.target_x),
            float(self.target_y),
            self.target_age_ms,
            self.target_quality,
            self.timestamp_ms,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "Suggestion":
        if len(data) != _SUGGESTION.size:
            raise LinkError(f"suggestion length {len(data)} != {_SUGGESTION.size}")
        s = cls(*_SUGGESTION.unpack(data))
        s.validate()
        return s

    def validate(self) -> None:
        if self.version != 1:
            raise LinkError(f"unsupported suggestion version {self.version}")
        _u8(self.flags, "flags")
        _u8(self.action_id, "action_id")
        if self.action_valid:
            if self.action_id not in ACTION_IDS:
                raise LinkError(f"action_id {self.action_id} not in 1..8")
        elif self.action_id not in (0, *ACTION_IDS):
            raise LinkError(f"invalid action_id {self.action_id}")
        _u16(self.action_seq, "action_seq")
        _u8(self.target_id, "target_id")
        if self.target_valid:
            if self.target_id not in TARGET_IDS:
                raise LinkError(f"target_id {self.target_id} not whitelisted")
        elif self.target_id != 0:
            raise LinkError("target_valid=0 requires target_id=0")
        _finite(self.target_x, "target_x")
        _finite(self.target_y, "target_y")
        _u16(self.target_age_ms, "target_age_ms")
        _u8(self.target_quality, "target_quality")
        if self.target_quality not in (0, 1, 2):
            raise LinkError(f"target_quality {self.target_quality} not 0/1/2")
        _u32(self.timestamp_ms, "timestamp_ms")
        if self.timestamp_ms > MAX_MATCH_MS:
            raise LinkError(f"timestamp_ms {self.timestamp_ms} > {MAX_MATCH_MS}")


def encode_suggestion(
    suggestion: Suggestion,
    sender_id: int,
    receiver_id: int,
    frame_seq: int = 0,
) -> bytes:
    return encode_interaction(
        SUBCMD_SUGGESTION, sender_id, receiver_id, suggestion.pack(), frame_seq
    )


def decode_suggestion(
    raw: bytes,
    expected_sender: Optional[int] = None,
    expected_receiver: Optional[int] = None,
) -> Suggestion:
    it = decode_interaction(raw, expected_sender, expected_receiver)
    if it.data_cmd_id != SUBCMD_SUGGESTION:
        raise LinkError(f"not 0x0210: 0x{it.data_cmd_id:04X}")
    return Suggestion.unpack(it.user_data)


# ---------------------------------------------------------------------------
# 0x0211 RadarStatus (12 B)
# ---------------------------------------------------------------------------

_STATUS = struct.Struct("<BBHIHH")


@dataclass
class RadarStatus:
    version: int = 1
    status: int = STATUS_OK
    seq: int = 0
    timestamp_ms: int = 0
    tx_count: int = 0
    drop_count: int = 0

    def pack(self) -> bytes:
        self.validate()
        return _STATUS.pack(
            self.version, self.status, self.seq, self.timestamp_ms,
            self.tx_count, self.drop_count,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "RadarStatus":
        if len(data) != _STATUS.size:
            raise LinkError(f"radar status length {len(data)} != {_STATUS.size}")
        s = cls(*_STATUS.unpack(data))
        s.validate()
        return s

    def validate(self) -> None:
        if self.version != 1:
            raise LinkError(f"unsupported status version {self.version}")
        if self.status not in (STATUS_OK, STATUS_DEGRADED, STATUS_RF_LOST):
            raise LinkError(f"invalid status {self.status}")
        _u16(self.seq, "seq")
        _u32(self.timestamp_ms, "timestamp_ms")
        if self.timestamp_ms > MAX_MATCH_MS:
            raise LinkError("timestamp_ms out of match range")
        _u16(self.tx_count, "tx_count")
        _u16(self.drop_count, "drop_count")


def encode_radar_status(
    status: RadarStatus, sender_id: int, receiver_id: int, frame_seq: int = 0
) -> bytes:
    return encode_interaction(
        SUBCMD_RADAR_STATUS, sender_id, receiver_id, status.pack(), frame_seq
    )


def decode_radar_status(
    raw: bytes,
    expected_sender: Optional[int] = None,
    expected_receiver: Optional[int] = None,
) -> RadarStatus:
    it = decode_interaction(raw, expected_sender, expected_receiver)
    if it.data_cmd_id != SUBCMD_RADAR_STATUS:
        raise LinkError(f"not 0x0211: 0x{it.data_cmd_id:04X}")
    return RadarStatus.unpack(it.user_data)


# ---------------------------------------------------------------------------
# 0x0220 SentryState + Ack (59 B)
# ---------------------------------------------------------------------------

_STATE = struct.Struct("<BBHIHHHffHHBB6sB6f")


@dataclass
class SentryState:
    version: int = 1
    flags: int = 0
    seq: int = 0
    timestamp_ms: int = 0
    last_action_seq: int = 0
    rx_count: int = 0
    crc_errors: int = 0
    sentry_x: float = 0.0
    sentry_y: float = 0.0
    hp: int = 0
    ammo: int = 0
    posture: int = 3
    enhanced: int = 0
    timers: bytes = b"\x00" * 6
    team_valid: int = 0
    team_pos: Sequence[float] = field(default_factory=lambda: (0.0,) * 6)

    def pack(self) -> bytes:
        self.validate()
        return _STATE.pack(
            self.version,
            self.flags,
            self.seq,
            self.timestamp_ms,
            self.last_action_seq,
            self.rx_count,
            self.crc_errors,
            float(self.sentry_x),
            float(self.sentry_y),
            self.hp,
            self.ammo,
            self.posture,
            self.enhanced,
            bytes(self.timers),
            self.team_valid,
            *[float(v) for v in self.team_pos],
        )

    @classmethod
    def unpack(cls, data: bytes) -> "SentryState":
        if len(data) != _STATE.size:
            raise LinkError(f"sentry state length {len(data)} != {_STATE.size}")
        values = list(_STATE.unpack(data))
        s = cls(
            version=values[0], flags=values[1], seq=values[2],
            timestamp_ms=values[3], last_action_seq=values[4],
            rx_count=values[5], crc_errors=values[6],
            sentry_x=values[7], sentry_y=values[8], hp=values[9],
            ammo=values[10], posture=values[11], enhanced=values[12],
            timers=values[13], team_valid=values[14],
            team_pos=tuple(values[15:21]),
        )
        s.validate()
        return s

    def validate(self) -> None:
        if self.version != 1:
            raise LinkError(f"unsupported sentry state version {self.version}")
        _u8(self.flags, "flags")
        _u16(self.seq, "seq")
        _u32(self.timestamp_ms, "timestamp_ms")
        if self.timestamp_ms > MAX_MATCH_MS:
            raise LinkError("timestamp_ms out of match range")
        _u16(self.last_action_seq, "last_action_seq")
        _u16(self.rx_count, "rx_count")
        _u16(self.crc_errors, "crc_errors")
        _finite(self.sentry_x, "sentry_x")
        _finite(self.sentry_y, "sentry_y")
        _u16(self.hp, "hp")
        _u16(self.ammo, "ammo")
        if self.posture not in (1, 2, 3):
            raise LinkError(f"posture {self.posture} not 1/2/3")
        if self.enhanced not in (0, 1):
            raise LinkError(f"enhanced {self.enhanced} not 0/1")
        if len(self.timers) != 6:
            raise LinkError("timers must be exactly 6 bytes")
        _u8(self.team_valid, "team_valid")
        if self.team_valid & ~0x07:
            raise LinkError("team_valid bits 3..7 must be zero")
        if len(self.team_pos) != 6:
            raise LinkError("team_pos must be 6 floats")
        for v in self.team_pos:
            _finite(v, "team_pos")


def encode_sentry_state(
    state: SentryState, sender_id: int, receiver_id: int, frame_seq: int = 0
) -> bytes:
    return encode_interaction(
        SUBCMD_SENTRY_STATE, sender_id, receiver_id, state.pack(), frame_seq
    )


def decode_sentry_state(
    raw: bytes,
    expected_sender: Optional[int] = None,
    expected_receiver: Optional[int] = None,
) -> SentryState:
    it = decode_interaction(raw, expected_sender, expected_receiver)
    if it.data_cmd_id != SUBCMD_SENTRY_STATE:
        raise LinkError(f"not 0x0220: 0x{it.data_cmd_id:04X}")
    return SentryState.unpack(it.user_data)


__all__ = [
    "SOF", "CMD_INTERACTION", "SUBCMD_SUGGESTION", "SUBCMD_RADAR_STATUS",
    "SUBCMD_SENTRY_STATE", "MAX_USER_DATA", "MAX_FRAME_DATA", "MAX_FRAME_TOTAL",
    "RED_SENTRY", "RED_RADAR", "BLUE_SENTRY", "BLUE_RADAR",
    "TARGET_IDS", "ACTION_IDS", "MAX_MATCH_MS",
    "FLAG_ACTION_VALID", "FLAG_TARGET_VALID", "FLAG_TARGET_ALIVE",
    "FLAG_POSITION_VALID", "FLAG_STALE",
    "STATUS_OK", "STATUS_DEGRADED", "STATUS_RF_LOST",
    "LinkError", "Frame", "Interaction", "FrameStreamParser", "Suggestion", "RadarStatus",
    "SentryState", "crc8", "crc16", "encode_frame", "decode_frame",
    "encode_interaction", "decode_interaction", "encode_suggestion",
    "decode_suggestion", "encode_radar_status", "decode_radar_status",
    "encode_sentry_state", "decode_sentry_state",
]
