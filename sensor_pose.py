"""Measured sensor pose history, independent of motion commands or GPIO."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SensorPose:
    host_monotonic_ns: int
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    source: str

    def __post_init__(self):
        if self.host_monotonic_ns < 0:
            raise ValueError("host_monotonic_ns must be non-negative")
        if not self.source:
            raise ValueError("source is required")
        if not all(math.isfinite(value) for value in (self.yaw_deg, self.pitch_deg, self.roll_deg)):
            raise ValueError("pose angles must be finite")


class SensorPoseHistory:
    def __init__(self, capacity: int = 240):
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self._poses = deque(maxlen=capacity)

    def append(self, pose: SensorPose) -> None:
        if self._poses and pose.host_monotonic_ns <= self._poses[-1].host_monotonic_ns:
            raise ValueError("pose timestamps must be strictly increasing")
        self._poses.append(pose)

    def nearest(self, host_monotonic_ns: int) -> tuple[SensorPose | None, float | None]:
        if not self._poses:
            return None, None
        pose = min(self._poses, key=lambda candidate: abs(candidate.host_monotonic_ns - host_monotonic_ns))
        return pose, abs(pose.host_monotonic_ns - host_monotonic_ns) / 1_000_000


def pose_metadata(history: SensorPoseHistory, host_monotonic_ns: int, *, stale_after_ms: float = 50.0) -> dict[str, object]:
    pose, age_ms = history.nearest(host_monotonic_ns)
    if pose is None:
        return {"status": "unavailable", "yaw_deg": None, "pitch_deg": None, "roll_deg": None, "pose_age_ms": None, "source": None}
    return {
        "status": "stale" if age_ms > stale_after_ms else "fresh",
        "yaw_deg": pose.yaw_deg,
        "pitch_deg": pose.pitch_deg,
        "roll_deg": pose.roll_deg,
        "pose_age_ms": age_ms,
        "source": pose.source,
    }
