"""Incremental frame synchronization and serial-port acquisition."""

import time
from typing import Any, Callable, List, Optional

from .protocol import FRAME_LENGTH, HEADER, Sample, parse_frame


SerialFactory = Callable[[str, int, float], Any]
AuditFunction = Callable[[str], None]


class SerialOpenError(RuntimeError):
    """Raised when the configured serial device cannot be opened."""


class FrameParser:
    """Incrementally extract valid frames from an arbitrary byte stream."""

    def __init__(self, timestamp_fn: Callable[[], float] = time.time):
        self.buffer = bytearray()
        self.timestamp_fn = timestamp_fn
        self.valid_frames = 0
        self.invalid_frames = 0

    def clear(self):
        self.buffer.clear()

    def feed(self, data: bytes) -> List[Sample]:
        """Consume bytes and return every complete sample found in them."""
        self.buffer.extend(data)
        samples = []

        while len(self.buffer) >= FRAME_LENGTH:
            offset = self.buffer.find(bytes((HEADER,)))
            if offset < 0:
                self.buffer.clear()
                break
            if offset > 0:
                del self.buffer[:offset]
            if len(self.buffer) < FRAME_LENGTH:
                break

            candidate = bytes(self.buffer[:FRAME_LENGTH])
            sample = parse_frame(candidate, self.timestamp_fn())
            if sample is None:
                # A payload byte can look like a header. Drop only this byte so
                # a real frame starting later in the candidate is not lost.
                self.invalid_frames += 1
                del self.buffer[:1]
                continue

            del self.buffer[:FRAME_LENGTH]
            self.valid_frames += 1
            samples.append(sample)

        return samples


def _default_serial_factory(port: str, baudrate: int, timeout: float) -> Any:
    import serial

    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)


class SerialFrameReader:
    """Open a serial device and yield decoded IMU samples."""

    def __init__(
        self,
        port: str,
        baudrate: int = 921600,
        timeout: float = 0.2,
        read_size: int = 256,
        serial_factory: Optional[SerialFactory] = None,
        timestamp_fn: Callable[[], float] = time.time,
        audit: Optional[AuditFunction] = None,
    ):
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.read_size = read_size
        self.serial_factory = serial_factory or _default_serial_factory
        self.parser = FrameParser(timestamp_fn=timestamp_fn)
        self.audit = audit or (lambda message: None)
        self._serial = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None

    def open(self):
        if self.is_open:
            return
        try:
            self._serial = self.serial_factory(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )
        except FileNotFoundError as exc:
            raise SerialOpenError(
                "{} does not exist; enable the Raspberry Pi UART and check "
                "wiring".format(self.port_name)
            ) from exc
        except PermissionError as exc:
            raise SerialOpenError(
                "permission denied for {}; add the user to the dialout group "
                "and log in again".format(self.port_name)
            ) from exc
        except Exception as exc:
            raise SerialOpenError(
                "failed to open {} @ {}: {}".format(
                    self.port_name, self.baudrate, exc
                )
            ) from exc

    def close(self):
        if self._serial is None:
            return
        try:
            self._serial.close()
        except Exception:
            pass
        self._serial = None

    def _wait_before_reconnect(self, delay: float, stop_event) -> bool:
        if stop_event is not None:
            return not stop_event.wait(delay)
        time.sleep(delay)
        return True

    def reconnect(self, delay: float = 2.0, stop_event=None) -> bool:
        """Retry the device. Return False if stop_event ended the wait."""
        self.close()
        while True:
            self.audit("disconnected; retrying in {:.0f} s".format(delay))
            if not self._wait_before_reconnect(delay, stop_event):
                return False
            try:
                self.open()
                self.audit(
                    "reconnected {} @ {}".format(self.port_name, self.baudrate)
                )
                return True
            except SerialOpenError as exc:
                self.audit("reconnect failed: {}".format(exc))
                delay = min(delay * 1.5, 30.0)

    def frames(self, stop_event=None):
        """Yield samples until stop_event is set or interrupted."""
        self.open()
        try:
            while stop_event is None or not stop_event.is_set():
                try:
                    data = self._serial.read(self.read_size)
                except Exception as exc:
                    self.audit("serial read failed: {}".format(exc))
                    self.parser.clear()
                    if not self.reconnect(stop_event=stop_event):
                        break
                    continue

                if not data:
                    continue
                yield from self.parser.feed(data)
        finally:
            self.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
