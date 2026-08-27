"""JY931 UART protocol definitions and pure frame parsing."""

import struct
import time
from dataclasses import dataclass
from typing import Optional, Tuple


FRAME_LENGTH = 11
HEADER = 0x55

TYPE_NAMES = {
    0x51: "acc",
    0x52: "gyro",
    0x53: "angle",
    0x54: "mag",
    0x59: "quat",
}
TYPE_CODES = {name: code for code, name in TYPE_NAMES.items()}
SAMPLE_TYPES = frozenset(TYPE_NAMES.values())

ACC_SCALE = 16.0 / 32768.0
GYRO_SCALE = 2000.0 / 32768.0
ANGLE_SCALE = 180.0 / 32768.0
QUAT_SCALE = 1.0 / 32768.0

_SCALES = {
    "acc": ACC_SCALE,
    "gyro": GYRO_SCALE,
    "angle": ANGLE_SCALE,
}


@dataclass(frozen=True)
class Sample:
    """One decoded protocol frame."""

    timestamp: float
    type: str
    values: Tuple[float, ...]
    norm: Optional[float] = None
    raw: Optional[Tuple[int, ...]] = None
    aux: Optional[float] = None


def checksum(frame: bytes) -> int:
    """Return the low byte of the sum of the first ten frame bytes."""
    return sum(frame[:10]) & 0xFF


def parse_frame(frame: bytes, timestamp: Optional[float] = None) -> Optional[Sample]:
    """Parse one complete 11-byte protocol frame."""
    if len(frame) != FRAME_LENGTH or frame[0] != HEADER:
        return None
    if checksum(frame) != frame[10]:
        return None

    sample_type = TYPE_NAMES.get(frame[1])
    if sample_type is None:
        return None

    if timestamp is None:
        timestamp = time.time()

    try:
        payload = frame[2:10]
        if sample_type == "quat":
            raw = struct.unpack("<4h", bytes(payload))
            values = tuple(value * QUAT_SCALE for value in raw)
            norm = sum(value * value for value in values) ** 0.5
            return Sample(timestamp, sample_type, values, norm, raw)

        raw = struct.unpack("<3h", bytes(payload[:6]))
        aux_format = "<H" if sample_type in ("gyro", "angle") else "<h"
        aux_raw = struct.unpack(aux_format, bytes(payload[6:]))[0]
        if sample_type == "mag":
            values = tuple(float(value) for value in raw)
            aux = aux_raw / 100.0
        elif sample_type == "acc":
            values = tuple(value * ACC_SCALE for value in raw)
            aux = aux_raw / 100.0
        elif sample_type == "gyro":
            values = tuple(value * GYRO_SCALE for value in raw)
            aux = aux_raw / 100.0
        else:
            values = tuple(value * _SCALES[sample_type] for value in raw)
            aux = float(aux_raw)
        return Sample(timestamp, sample_type, values, aux=aux)
    except (struct.error, IndexError):
        return None
