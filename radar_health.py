"""Thread-safe runtime health state for the radar service."""

from __future__ import annotations

import threading
import time
from typing import Any, Mapping


class RadarHealthMonitor:
    """Summarize data-link health without exposing hardware objects to the UI."""

    def __init__(self, *, clock=time.time, monotonic=time.monotonic):
        self._clock = clock
        self._monotonic = monotonic
        self._lock = threading.RLock()
        self._serial_connected = False
        self._data_port = ""
        self._started_at_s: float | None = None
        self._last_frame_at_monotonic_s: float | None = None
        self._last_frame_num: int | None = None
        self._frames_parsed = 0
        self._bytes_received = 0
        self._parser_errors = 0
        self._last_error: str | None = None

    def mark_serial_open(self, data_port: str) -> None:
        with self._lock:
            self._serial_connected = True
            self._data_port = data_port
            self._started_at_s = self._clock()
            self._last_frame_at_monotonic_s = None
            self._last_frame_num = None
            self._last_error = None

    def mark_bytes(self, count: int) -> None:
        with self._lock:
            self._bytes_received += max(0, int(count))

    def mark_frame(self, frame_num: int) -> None:
        with self._lock:
            self._last_frame_at_monotonic_s = self._monotonic()
            self._last_frame_num = int(frame_num)
            self._frames_parsed += 1
            self._last_error = None

    def mark_error(self, error: Exception | str, *, disconnected: bool = False) -> None:
        with self._lock:
            self._parser_errors += 1
            self._last_error = str(error)
            if disconnected:
                self._serial_connected = False

    def mark_serial_closed(self) -> None:
        with self._lock:
            self._serial_connected = False

    def snapshot(self, *, recording: Mapping[str, Any] | None = None, stale_after_s: float = 2.0) -> dict[str, Any]:
        with self._lock:
            now = self._monotonic()
            age_s = None if self._last_frame_at_monotonic_s is None else max(0.0, now - self._last_frame_at_monotonic_s)
            if not self._serial_connected:
                data_status = "offline"
            elif age_s is None:
                data_status = "waiting_for_frame"
            elif age_s > stale_after_s:
                data_status = "stale"
            else:
                data_status = "streaming"
            return {
                "data_status": data_status,
                "data_port": self._data_port,
                "serial_connected": self._serial_connected,
                "last_frame_age_s": age_s,
                "last_frame_num": self._last_frame_num,
                "frames_parsed": self._frames_parsed,
                "bytes_received": self._bytes_received,
                "parser_errors": self._parser_errors,
                "last_error": self._last_error,
                "recording": dict(recording or {}),
            }
