"""Pure, conservative encoder-to-radar-frame registration primitives.

The caller must supply monotonic times from the same host clock. Radar packet
parse time is *not* the RF acquisition time; a registered result based on it
remains provisional until device/host latency is measured in the field.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


@dataclass(frozen=True)
class AngleSample:
    query_started_monotonic_s: float
    query_completed_monotonic_s: float
    angle_deg: float

    @property
    def midpoint_s(self) -> float:
        return (self.query_started_monotonic_s + self.query_completed_monotonic_s) / 2.0

    @property
    def query_duration_s(self) -> float:
        return self.query_completed_monotonic_s - self.query_started_monotonic_s


@dataclass(frozen=True)
class Registration:
    status: str
    yaw_deg: float | None = None
    sample_gap_s: float | None = None
    angular_span_deg: float | None = None
    time_basis: str = "host_packet_parse_monotonic"


def estimate_frame_yaw(
    frame_parse_monotonic_s: float,
    before: AngleSample,
    after: AngleSample,
    *,
    direction: int,
    zero_deg: float = 0.0,
    mounting_yaw_deg: float = 0.0,
    max_gap_s: float = 0.1,
    max_query_duration_s: float = 0.05,
    max_span_deg: float = 10.0,
    max_possible_rpm: float = 130.0,
) -> Registration:
    """Interpolate a single-turn encoder angle only between trustworthy samples.

    ``direction`` is +1 for increasing encoder angles, -1 for decreasing.
    Samples that could cover >=180 degrees at ``max_possible_rpm`` are rejected
    because single-turn readings cannot reveal a hidden full revolution.
    A rejected frame must not be plotted at a guessed azimuth.
    """
    if direction not in (-1, 1):
        raise ValueError("direction must be +1 or -1")
    if not all(math.isfinite(value) for value in (
        frame_parse_monotonic_s, before.query_started_monotonic_s,
        before.query_completed_monotonic_s, before.angle_deg,
        after.query_started_monotonic_s, after.query_completed_monotonic_s,
        after.angle_deg, zero_deg, mounting_yaw_deg,
    )):
        raise ValueError("timestamps and angles must be finite")
    limits = (max_gap_s, max_query_duration_s, max_span_deg, max_possible_rpm)
    if not all(math.isfinite(value) and value > 0 for value in limits):
        raise ValueError("quality limits must be positive")
    if before.query_duration_s < 0 or after.query_duration_s < 0:
        raise ValueError("query completion cannot precede its start")

    start_s, end_s = before.midpoint_s, after.midpoint_s
    if end_s <= start_s or not start_s <= frame_parse_monotonic_s <= end_s:
        return Registration("unbracketed")
    gap_s = end_s - start_s
    if gap_s > max_gap_s:
        return Registration("sample_gap", sample_gap_s=gap_s)
    if max(before.query_duration_s, after.query_duration_s) > max_query_duration_s:
        return Registration("query_duration", sample_gap_s=gap_s)
    if gap_s * max_possible_rpm * 6.0 >= 180.0:
        return Registration("ambiguous_wrap", sample_gap_s=gap_s)

    forward_span = (direction * (after.angle_deg - before.angle_deg)) % 360.0
    if forward_span >= 180.0:
        return Registration("ambiguous_wrap", sample_gap_s=gap_s,
                            angular_span_deg=forward_span)
    if forward_span > max_span_deg:
        return Registration("angular_span", sample_gap_s=gap_s,
                            angular_span_deg=forward_span)

    fraction = (frame_parse_monotonic_s - start_s) / gap_s
    angle = before.angle_deg + direction * forward_span * fraction
    yaw = (angle - zero_deg + mounting_yaw_deg) % 360.0
    return Registration("registered", yaw, gap_s, forward_span)


def register_points(
    points: list[Mapping[str, Any]],
    registration: Registration,
    *,
    mounting_roll_deg: float = 0.0,
) -> list[dict[str, Any]] | None:
    """Apply installation roll about local forward y, then turntable yaw about z.

    Coordinates are local x=lateral, y=forward, z=up. The roll sign and
    electrical/mechanical zero require target-based physical calibration.
    """
    if registration.status != "registered" or registration.yaw_deg is None:
        return None
    if not math.isfinite(mounting_roll_deg):
        raise ValueError("mounting roll must be finite")
    roll = math.radians(mounting_roll_deg)
    yaw = math.radians(registration.yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(yaw), math.sin(yaw)
    transformed = []
    for point in points:
        x, y, z = float(point["x"]), float(point["y"]), float(point["z"])
        rolled_x, rolled_z = x * cr + z * sr, -x * sr + z * cr
        result = dict(point)
        result.update({"x": rolled_x * cy - y * sy,
                       "y": rolled_x * sy + y * cy,
                       "z": rolled_z})
        transformed.append(result)
    return transformed
