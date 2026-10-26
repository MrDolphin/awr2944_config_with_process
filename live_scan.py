"""Bounded live assembly of radar frames and measured turntable angles.

This module never opens hardware or estimates angle from commanded speed.  It
only publishes a radar frame after two QD4310 status samples bracket the host
packet-parse timestamp.  The result therefore remains provisional until the
radar acquisition-to-host latency is measured in the field.
"""

from __future__ import annotations

from collections import Counter, deque
import math
import threading
from typing import Any, Mapping

from scan_registration import AngleSample, estimate_frame_yaw, register_points


class LiveScanAssembler:
    """Join measured encoder samples to frames without inventing azimuth data."""

    def __init__(
        self,
        *,
        direction: int,
        zero_deg: float,
        mounting_yaw_deg: float,
        mounting_roll_deg: float,
        max_gap_s: float = 0.1,
        max_query_duration_s: float = 0.05,
        max_span_deg: float = 10.0,
        max_possible_rpm: float = 130.0,
        stale_after_s: float = 1.0,
        max_pending_frames: int = 16,
        max_frame_points: int = 2048,
        max_map_frames: int = 512,
        max_map_points: int = 6000,
        max_event_backlog: int = 2048,
    ) -> None:
        if direction not in (-1, 1):
            raise ValueError("direction must be +1 or -1")
        finite_values = (
            zero_deg, mounting_yaw_deg, mounting_roll_deg, max_gap_s,
            max_query_duration_s, max_span_deg, max_possible_rpm,
            stale_after_s,
        )
        if not all(math.isfinite(float(value)) for value in finite_values):
            raise ValueError("live scan configuration must be finite")
        if min(max_gap_s, max_query_duration_s, max_span_deg,
               max_possible_rpm, stale_after_s) <= 0:
            raise ValueError("live scan quality limits must be positive")
        if min(int(max_pending_frames), int(max_frame_points), int(max_map_frames),
               int(max_map_points), int(max_event_backlog)) <= 0:
            raise ValueError("live scan buffer limits must be positive")

        self._direction = direction
        self._zero_deg = float(zero_deg)
        self._mounting_yaw_deg = float(mounting_yaw_deg)
        self._mounting_roll_deg = float(mounting_roll_deg)
        self._quality = {
            "max_gap_s": float(max_gap_s),
            "max_query_duration_s": float(max_query_duration_s),
            "max_span_deg": float(max_span_deg),
            "max_possible_rpm": float(max_possible_rpm),
        }
        self._stale_after_s = float(stale_after_s)
        self._max_pending_frames = int(max_pending_frames)
        self._max_frame_points = int(max_frame_points)
        self._max_map_frames = int(max_map_frames)
        self._max_map_points = int(max_map_points)
        self._max_event_backlog = int(max_event_backlog)
        self._lock = threading.RLock()
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self._samples: deque[AngleSample] = deque(maxlen=2)
            self._pending: deque[dict[str, Any]] = deque()
            self._registered: deque[tuple[float, list[dict[str, Any]]]] = deque()
            self._events: deque[dict[str, Any]] = deque()
            self._rejected: Counter[str] = Counter()
            self._registered_frames = 0
            self._last_frame_status: str | None = None
            self._last_yaw_deg: float | None = None
            self._last_sample_midpoint_s: float | None = None
            self._cumulative_angle_deg = 0.0
            self._map_points_count = 0
            self._map_truncated = False
            self._discontinuity_reason: str | None = None
            self._dropped_event_backlog = 0

    def _clear_map(self, reason: str) -> None:
        self._registered.clear()
        self._map_points_count = 0
        self._cumulative_angle_deg = 0.0
        self._last_yaw_deg = None
        self._discontinuity_reason = reason
        self._map_truncated = False

    def _append_event(self, event: dict[str, Any]) -> None:
        if len(self._events) >= self._max_event_backlog:
            self._events.popleft()
            self._dropped_event_backlog += 1
        self._events.append(event)

    def _trim_measured_revolution(self) -> None:
        while (self._registered and
               self._cumulative_angle_deg - self._registered[0][0] >= 360.0):
            _, old_points = self._registered.popleft()
            self._map_points_count -= len(old_points)

    @staticmethod
    def _angle_sample(sample: Mapping[str, Any]) -> AngleSample:
        return AngleSample(
            float(sample["query_started_monotonic_s"]),
            float(sample["query_completed_monotonic_s"]),
            float(sample["angle_deg"]),
        )

    def add_motor_sample(self, sample: Mapping[str, Any]) -> None:
        measured = self._angle_sample(sample)
        if not all(math.isfinite(value) for value in (
            measured.query_started_monotonic_s,
            measured.query_completed_monotonic_s, measured.angle_deg,
        )) or measured.query_duration_s < 0:
            raise ValueError("motor sample times and angle must be finite and ordered")
        with self._lock:
            if self._samples and measured.midpoint_s <= self._samples[-1].midpoint_s:
                self._rejected["non_monotonic_sample"] += 1
                self._clear_map("non_monotonic_sample")
                return
            if self._samples:
                before = self._samples[-1]
                decision = estimate_frame_yaw(
                    (before.midpoint_s + measured.midpoint_s) / 2.0,
                    before, measured, direction=self._direction,
                    zero_deg=self._zero_deg,
                    mounting_yaw_deg=self._mounting_yaw_deg,
                    **self._quality,
                )
                if decision.status == "registered":
                    self._cumulative_angle_deg += decision.angular_span_deg or 0.0
                    self._trim_measured_revolution()
                else:
                    self._clear_map(decision.status)
            self._samples.append(measured)
            self._last_sample_midpoint_s = measured.midpoint_s
            self._process_pending()

    def add_frame(self, frame: Mapping[str, Any]) -> None:
        payload = dict(frame)
        frame_time = float(payload["host_monotonic_s"])
        if not math.isfinite(frame_time):
            raise ValueError("frame host_monotonic_s must be finite")
        with self._lock:
            points = payload.get("points", [])
            if isinstance(points, (list, tuple)) and len(points) > min(
                    self._max_frame_points, self._max_map_points):
                self._rejected["point_limit"] += 1
                self._last_frame_status = "point_limit"
                self._append_event(self._event(payload, "point_limit"))
                return
            try:
                valid_points = isinstance(points, (list, tuple)) and all(
                    isinstance(point, Mapping) and
                    all(math.isfinite(float(point[key])) for key in ("x", "y", "z")) and
                    all(math.isfinite(float(point[key])) for key in ("v", "snr", "noise")
                        if key in point and point[key] is not None)
                    for point in points
                )
            except (KeyError, TypeError, ValueError, OverflowError):
                valid_points = False
            if not valid_points:
                self._rejected["invalid_point"] += 1
                self._last_frame_status = "invalid_point"
                self._append_event(self._event(payload, "invalid_point"))
                return
            # The producer may reuse its point dictionaries while this frame waits
            # for the next encoder sample. Keep the validated coordinates immutable
            # from the producer's perspective until registration has finished.
            payload["points"] = [dict(point) for point in points]
            self._pending.append(payload)
            while len(self._pending) > self._max_pending_frames:
                dropped = self._pending.popleft()
                self._rejected["pending_overflow"] += 1
                self._last_frame_status = "pending_overflow"
                self._append_event(self._event(dropped, "pending_overflow"))
            self._process_pending()

    def drain_events(self) -> list[dict[str, Any]]:
        """Return completed per-frame decisions once for capture diagnostics."""
        with self._lock:
            events = list(self._events)
            self._events.clear()
            return events

    def _event(
        self,
        frame: Mapping[str, Any],
        status: str,
        *,
        registration=None,
        before: AngleSample | None = None,
        after: AngleSample | None = None,
    ) -> dict[str, Any]:
        return {
            "frame_num": int(frame.get("frame_num", -1)),
            "host_monotonic_s": float(frame["host_monotonic_s"]),
            "status": status,
            "yaw_deg": registration.yaw_deg if registration is not None else None,
            "sample_gap_s": registration.sample_gap_s if registration is not None else None,
            "angular_span_deg": registration.angular_span_deg if registration is not None else None,
            "before_sample_midpoint_s": before.midpoint_s if before is not None else None,
            "after_sample_midpoint_s": after.midpoint_s if after is not None else None,
            "before_query_duration_s": before.query_duration_s if before is not None else None,
            "after_query_duration_s": after.query_duration_s if after is not None else None,
            "point_count": len(frame.get("points", [])),
            "registration_direction": self._direction,
            "registration_zero_deg": self._zero_deg,
            "registration_mounting_yaw_deg": self._mounting_yaw_deg,
            "registration_mounting_roll_deg": self._mounting_roll_deg,
            **self._quality,
            "stale_after_s": self._stale_after_s,
            "time_basis": "host_packet_parse_monotonic",
            "rf_time_calibrated": False,
            "pose_compensated": False,
        }

    def _process_pending(self) -> None:
        if len(self._samples) < 2:
            return
        before, after = self._samples
        remaining: deque[dict[str, Any]] = deque()
        while self._pending:
            frame = self._pending.popleft()
            frame_time = float(frame["host_monotonic_s"])
            if frame_time > after.midpoint_s:
                remaining.append(frame)
                continue
            registration = estimate_frame_yaw(
                frame_time, before, after,
                direction=self._direction,
                zero_deg=self._zero_deg,
                mounting_yaw_deg=self._mounting_yaw_deg,
                **self._quality,
            )
            self._last_frame_status = registration.status
            self._append_event(self._event(
                frame, registration.status, registration=registration,
                before=before, after=after,
            ))
            if registration.status != "registered":
                self._rejected[registration.status] += 1
                continue
            points = register_points(
                list(frame.get("points", [])), registration,
                mounting_roll_deg=self._mounting_roll_deg,
            ) or []
            fraction = (frame_time - before.midpoint_s) / (after.midpoint_s - before.midpoint_s)
            span = registration.angular_span_deg or 0.0
            frame_progress = self._cumulative_angle_deg - span * (1.0 - fraction)
            self._registered.append((frame_progress, points))
            self._map_points_count += len(points)
            self._trim_measured_revolution()
            while (len(self._registered) > self._max_map_frames or
                   self._map_points_count > self._max_map_points):
                _, old_points = self._registered.popleft()
                self._map_points_count -= len(old_points)
                self._map_truncated = True
            self._registered_frames += 1
            self._last_yaw_deg = registration.yaw_deg
            self._discontinuity_reason = None
        self._pending = remaining

    def snapshot(self, *, now_monotonic_s: float) -> dict[str, Any]:
        now = float(now_monotonic_s)
        with self._lock:
            stale = (self._last_sample_midpoint_s is None or
                     now - self._last_sample_midpoint_s > self._stale_after_s)
            if stale and self._registered:
                self._clear_map("telemetry_stale")
            points = [] if stale else [point for _, batch in self._registered for point in batch]
            return {
                "state": "stale" if stale else "provisional",
                "points": points,
                "registered_frames": self._registered_frames,
                "pending_frames": len(self._pending),
                "map_frames": len(self._registered),
                "map_truncated": self._map_truncated,
                "discontinuity_reason": self._discontinuity_reason,
                "dropped_event_backlog": self._dropped_event_backlog,
                "rejected_frames": dict(self._rejected),
                "last_frame_status": self._last_frame_status,
                "last_yaw_deg": self._last_yaw_deg,
                "last_sample_age_ms": (
                    None if self._last_sample_midpoint_s is None
                    else round(max(0.0, now - self._last_sample_midpoint_s) * 1000)
                ),
                "rf_time_calibrated": False,
                "pose_compensated": False,
                "time_basis": "host_packet_parse_monotonic",
                "azimuth_source": "qd4310_usb_status_interpolated",
            }
