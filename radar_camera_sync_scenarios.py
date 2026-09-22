"""Deterministic, hardware-free timing scenarios for radar-camera synchronization."""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
import math
from typing import Iterable

from radar_camera_sync import match_camera_frame
from tools.camera.camera_capture import CameraFrame, CameraFrameBuffer


_BASE_URL = "http://synthetic-camera"


@dataclass(frozen=True)
class SyncScenarioSample:
    radar_monotonic_ns: int
    status: str
    frame_id: int | None
    time_offset_ms: float | None


@dataclass(frozen=True)
class SyncScenarioReport:
    samples: tuple[SyncScenarioSample, ...]
    status_counts: dict[str, int]
    matched_ratio: float
    absolute_matched_offset_p95_ms: float | None
    retained_frame_ids: tuple[int, ...]


def run_timing_scenario(
    *,
    camera_timestamps_ns: Iterable[int],
    radar_timestamps_ns: Iterable[int],
    capacity: int = 120,
    matched_limit_ms: float = 50.0,
    stale_limit_ms: float = 100.0,
) -> SyncScenarioReport:
    """Replay receive-time sequences without opening a camera or radar device.

    A radar sample only observes camera frames whose receive timestamp is at or
    before that radar timestamp.  This models the live process faithfully: a
    future camera frame cannot be used to improve a historical radar match.
    """
    camera_times = _validate_timestamps("camera_timestamps_ns", camera_timestamps_ns)
    radar_times = _validate_timestamps("radar_timestamps_ns", radar_timestamps_ns)
    if capacity <= 0:
        raise ValueError("capacity must be positive")

    buffer = CameraFrameBuffer(capacity=capacity)
    retained_ids: deque[int] = deque(maxlen=capacity)
    camera_index = 0
    samples: list[SyncScenarioSample] = []
    counts: Counter[str] = Counter()
    matched_offsets: list[float] = []

    for radar_time in radar_times:
        while camera_index < len(camera_times) and camera_times[camera_index] <= radar_time:
            frame_id = camera_index + 1
            timestamp = camera_times[camera_index]
            buffer.append(
                CameraFrame(
                    frame_id=frame_id,
                    host_monotonic_ns=timestamp,
                    host_wall_time_ns=timestamp,
                    width=1280,
                    height=720,
                    jpeg=b"synthetic-jpeg",
                )
            )
            retained_ids.append(frame_id)
            camera_index += 1

        result = match_camera_frame(
            radar_time,
            buffer,
            _BASE_URL,
            matched_limit_ms=matched_limit_ms,
            stale_limit_ms=stale_limit_ms,
        )
        counts[result.status] += 1
        samples.append(
            SyncScenarioSample(
                radar_monotonic_ns=radar_time,
                status=result.status,
                frame_id=result.frame_id,
                time_offset_ms=result.time_offset_ms,
            )
        )
        if result.status == "matched" and result.time_offset_ms is not None:
            matched_offsets.append(abs(result.time_offset_ms))

    total = len(samples)
    return SyncScenarioReport(
        samples=tuple(samples),
        status_counts={status: counts[status] for status in ("matched", "degraded", "stale", "unavailable")},
        matched_ratio=counts["matched"] / total if total else 0.0,
        absolute_matched_offset_p95_ms=_percentile_nearest_rank(matched_offsets, 0.95),
        retained_frame_ids=tuple(retained_ids),
    )


def _validate_timestamps(name: str, timestamps: Iterable[int]) -> tuple[int, ...]:
    values = tuple(timestamps)
    for value in values:
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise ValueError(f"{name} must contain non-negative integer timestamps")
    if any(later <= earlier for earlier, later in zip(values, values[1:])):
        raise ValueError(f"{name} must be strictly increasing")
    return values


def _percentile_nearest_rank(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[math.ceil(percentile * len(ordered)) - 1]
