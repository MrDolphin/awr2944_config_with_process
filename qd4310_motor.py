"""QD4310 USB serial speed control.

The operator supplies the device path to the server; importing this module never
opens a port or enables motion.  A separate motor power supply is still needed.
"""

import math
import threading

import serial


MIN_RPM = 0.1
MAX_RPM = 30.0


def validate_rpm(value):
    try:
        rpm = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("QD4310 rpm must be a number") from exc
    if not math.isfinite(rpm) or not MIN_RPM <= rpm <= MAX_RPM:
        raise ValueError(f"QD4310 rpm must be between {MIN_RPM} and {MAX_RPM}")
    return rpm


class Qd4310Motor:
    def __init__(self, port):
        if not port:
            raise ValueError("QD4310 USB serial port is not configured")
        self.port = port
        self._serial = None
        self._lock = threading.Lock()

    @staticmethod
    def _speed_command(rpm):
        return f"ctrl speed {rpm:g}\n".encode("ascii")

    @staticmethod
    def _write(conn, command):
        if conn.write(command) != len(command):
            raise OSError("QD4310 USB serial command was only partly written")

    def start(self, rpm):
        rpm = validate_rpm(rpm)
        with self._lock:
            if self._serial is not None:
                raise RuntimeError("QD4310 motor is already enabled")
            conn = serial.Serial(
                self.port, baudrate=115200, timeout=0.005, write_timeout=0.5
            )
            try:
                self._write(conn, b"enable\n")
                self._write(conn, self._speed_command(rpm))
            except Exception:
                for command in (b"ctrl speed 0\n", b"disable\n"):
                    try:
                        self._write(conn, command)
                    except Exception:
                        pass
                conn.close()
                raise
            self._serial = conn

    def set_speed(self, rpm):
        rpm = validate_rpm(rpm)
        with self._lock:
            if self._serial is None:
                raise RuntimeError("QD4310 motor is not running")
            self._write(self._serial, self._speed_command(rpm))

    def stop(self):
        with self._lock:
            conn, self._serial = self._serial, None
            if conn is None:
                return
            first_error = None
            try:
                self._write(conn, b"ctrl speed 0\n")
            except Exception as exc:
                first_error = exc
            try:
                self._write(conn, b"disable\n")
            except Exception as exc:
                if first_error is None:
                    first_error = exc
            finally:
                conn.close()
            if first_error is not None:
                raise first_error
