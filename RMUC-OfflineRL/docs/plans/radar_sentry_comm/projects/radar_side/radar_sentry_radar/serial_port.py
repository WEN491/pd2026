"""Linux serial transport for the 115200 8N1 power-management User port.

No pyserial dependency. The frame layer is in ``radar_link``; this module only
moves bytes. For tests, use ``FakeTransport``.
"""
from __future__ import annotations

import os
import select
import termios
from typing import Iterable, List, Optional


class SerialError(RuntimeError):
    pass


class SerialTransport:
    def __init__(self, device: str, baud: int = 115200, timeout: float = 0.05) -> None:
        self.device = device
        self.timeout = timeout
        try:
            self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            raise SerialError(f"open {device}: {exc}") from exc
        try:
            self._configure(baud)
        except Exception:
            os.close(self.fd)
            self.fd = -1
            raise

    def _configure(self, baud: int) -> None:
        speed = getattr(termios, f"B{baud}", None)
        if speed is None:
            raise SerialError(f"unsupported baud {baud}")
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0  # iflag: raw
        attrs[1] = 0  # oflag: raw
        attrs[2] = (attrs[2] & ~termios.CSIZE) | termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[3] = 0  # lflag: raw
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def read(self, max_bytes: int = 4096) -> bytes:
        try:
            ready, _, _ = select.select([self.fd], [], [], self.timeout)
        except OSError as exc:
            raise SerialError(f"select: {exc}") from exc
        if not ready:
            return b""
        try:
            return os.read(self.fd, max_bytes)
        except BlockingIOError:
            return b""

    def write(self, data: bytes) -> None:
        view = memoryview(data)
        while view:
            try:
                n = os.write(self.fd, view)
            except OSError as exc:
                raise SerialError(f"write: {exc}") from exc
            if n <= 0:
                raise SerialError("write returned 0")
            view = view[n:]

    def close(self) -> None:
        if getattr(self, "fd", -1) >= 0:
            os.close(self.fd)
            self.fd = -1

    def __enter__(self) -> "SerialTransport":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class FakeTransport:
    """In-memory transport for unit tests / loopback demos."""

    def __init__(self, reads: Optional[Iterable[bytes]] = None) -> None:
        self.reads: List[bytes] = list(reads or [])
        self.written = bytearray()

    def push_read(self, data: bytes) -> None:
        self.reads.append(data)

    def read(self, max_bytes: int = 4096) -> bytes:
        if not self.reads:
            return b""
        return self.reads.pop(0)[:max_bytes]

    def write(self, data: bytes) -> None:
        self.written.extend(data)

    def close(self) -> None:
        pass


__all__ = ["SerialError", "SerialTransport", "FakeTransport"]
