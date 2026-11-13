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
import uuid
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
        max_profile_spokes: int = 512,
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
               int(max_map_points), int(max_profile_spokes), int(max_event_backlog)) <= 0:
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
        self._max_profile_spokes = int(max_profile_spokes)
        self._max_event_backlog = int(max_event_backlog)
        self._lock = threading.RLock()
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self._samples: deque[AngleSample] = deque(maxlen=2)
            self._pending: deque[dict[str, Any]] = deque()
            self._registered: deque[tuple[float, list[dict[str, Any]]]] = deque()
            self._profiles: deque[tuple[float, dict[str, Any]]] = deque()
            self._profile_generation = uuid.uuid4().hex
            self._profile_seq = 0
            self._profile_capacity_eviction_progress: float | None = None
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
            self._latest_complete_turn: dict[str, Any] | None = None
            self._reset_turn_diagnostics()

    def _reset_turn_diagnostics(self) -> None:
        self._turn_start_progress = self._cumulative_angle_deg
        self._turn_spokes: list[tuple[float, int, float]] = []
        self._turn_raw_frames: deque[tuple[int, float, bool]] = deque(
            maxlen=self._max_event_backlog)
        self._turn_decisions: deque[tuple[int, float, str]] = deque(
            maxlen=self._max_event_backlog)
        self._turn_evidence_truncated = False
        self._turn_first_raw_frame_num: int | None = None
        self._turn_raw_missing = 0
        self._turn_tlv2_missing = 0
        self._turn_rejected: Counter[str] = Counter()
        self._last_raw_frame_num: int | None = None

    def _finish_turn_diagnostics(self) -> None:
        # Keep the first and last observation at each bearing so consecutive
        # frames at the same angle do not become false evidence for a gap.
        bearings: dict[float, list[tuple[int, float]]] = {}
        for angle, frame_num, stamp in self._turn_spokes:
            angle = round(angle % 360.0, 6) % 360.0
            row = (frame_num, stamp)
            edges = bearings.setdefault(angle, [row, row])
            edges[1] = row
        angles = sorted(bearings)
        gaps = [(round((angles[(i + 1) % len(angles)] - angle) % 360.0, 6),
                 angle, angles[(i + 1) % len(angles)])
                for i, angle in enumerate(angles)]
        if len(angles) == 1:
            gaps = [(360.0, angles[0], angles[0])]
        largest = max(gaps, default=None)
        start = end = None
        if largest:
            start = bearings[largest[1]][int(self._direction == 1)]
            end = bearings[largest[2]][int(self._direction == -1)]
        earlier, later = (start, end) if self._direction == 1 else (end, start)
        evidence = self._turn_gap_evidence(earlier, later)
        self._latest_complete_turn = {
            "valid_tlv2_spokes": len(self._turn_spokes),
            "unique_azimuths": len(angles),
            "max_gap_deg": largest[0] if largest else None,
            "max_gap_start_deg": largest[1] if largest else None,
            "max_gap_end_deg": largest[2] if largest else None,
            "first_frame_num": self._turn_spokes[0][1] if self._turn_spokes else None,
            "last_frame_num": self._turn_spokes[-1][1] if self._turn_spokes else None,
            "first_raw_frame_num": self._turn_first_raw_frame_num,
            "last_raw_frame_num": self._last_raw_frame_num,
            "max_gap_start_frame_num": start[0] if start else None,
            "max_gap_end_frame_num": end[0] if end else None,
            "max_gap_start_host_monotonic_s": start[1] if start else None,
            "max_gap_end_host_monotonic_s": end[1] if end else None,
            "max_gap_evidence": evidence,
            "raw_frame_missing": self._turn_raw_missing,
            "tlv2_missing": self._turn_tlv2_missing,
            "registration_rejected": dict(self._turn_rejected),
            "time_basis": "host_packet_parse_monotonic",
            "rf_time_calibrated": False,
        }
        self._reset_turn_diagnostics()

    def _turn_gap_evidence(self, start, end):
        evidence = {
            "raw_missing_frame_numbers": [], "raw_missing_count": 0,
            "tlv2_missing_frame_numbers": [], "tlv2_missing_count": 0,
            "registration_rejected_frames": [], "registration_rejected_count": 0,
            "classification": "undetermined", "evidence_truncated": self._turn_evidence_truncated,
            "interval_observed": False,
        }
        if start is None or end is None or self._turn_evidence_truncated:
            return evidence
        start_num, start_time = start
        end_num, end_time = end
        # The angular 0° boundary can lie inside a turn. Only an interval
        # reversed in acquisition time crosses the unobserved turn seam.
        if start_time >= end_time:
            return evidence
        interior = sorted((row for row in self._turn_raw_frames
                           if start_time < row[1] < end_time), key=lambda row: row[1])
        sequence = [start_num] + [row[0] for row in interior] + [end_num]
        if any(not 0 <= number < 2**32 for number in sequence):
            return evidence
        steps = [(after - before) % 2**32 for before, after in zip(sequence, sequence[1:])]
        if any(not 0 < step < 100000 for step in steps):
            return evidence
        evidence["interval_observed"] = True
        for before, step in zip(sequence, steps):
            evidence["raw_missing_count"] += step - 1
            remaining = max(0, 16 - len(evidence["raw_missing_frame_numbers"]))
            evidence["raw_missing_frame_numbers"].extend(
                (before + offset) % 2**32
                for offset in range(1, min(step, remaining + 1)))
        missing_tlv2 = [row[0] for row in interior if not row[2]]
        evidence["tlv2_missing_count"] = len(missing_tlv2)
        evidence["tlv2_missing_frame_numbers"] = missing_tlv2[:16]
        rejected = [{"frame_num": num, "reason": status}
                    for num, stamp, status in self._turn_decisions
                    if start_time < stamp < end_time and status != "registered"]
        evidence["registration_rejected_count"] = len(rejected)
        evidence["registration_rejected_frames"] = rejected[:16]
        causes = [key for key, count in (
            ("raw_frame_missing", evidence["raw_missing_count"]),
            ("tlv2_missing", evidence["tlv2_missing_count"]),
            ("registration_rejected", evidence["registration_rejected_count"])) if count]
        evidence["classification"] = (causes[0] if len(causes) == 1 else
                                      "undetermined" if causes else "unobserved")
        return evidence

    def _clear_map(self, reason: str) -> None:
        self._registered.clear()
        self._profiles.clear()
        self._profile_capacity_eviction_progress = None
        self._profile_generation = uuid.uuid4().hex
        self._map_points_count = 0
        self._cumulative_angle_deg = 0.0
        self._last_yaw_deg = None
        self._discontinuity_reason = reason
        self._map_truncated = False
        self._reset_turn_diagnostics()

    def _append_event(self, event: dict[str, Any]) -> None:
        if len(self._events) >= self._max_event_backlog:
            self._events.popleft()
            self._dropped_event_backlog += 1
        self._events.append(event)
        if len(self._turn_decisions) == self._turn_decisions.maxlen:
            self._turn_evidence_truncated = True
        self._turn_decisions.append((event["frame_num"], event["host_monotonic_s"],
                                     event["status"]))
        if event["status"] != "registered":
            self._turn_rejected[event["status"]] += 1

    def _trim_measured_revolution(self) -> None:
        while (self._registered and
               self._cumulative_angle_deg - self._registered[0][0] >= 360.0):
            _, old_points = self._registered.popleft()
            self._map_points_count -= len(old_points)
        while (self._profiles and
               self._cumulative_angle_deg - self._profiles[0][0] >= 360.0):
            self._profiles.popleft()

    def profile_delta(self, generation: str | None, after_seq: int,
                      *, max_spokes: int = 64) -> dict[str, Any]:
        """Return bounded registered TLV2 spokes for one WebSocket client.

        A generation change means the old sweep was invalidated; clients must
        clear it rather than blending unrelated angular coordinate systems.
        """
        if max_spokes <= 0:
            raise ValueError("max_spokes must be positive")
        with self._lock:
            same_generation = generation == self._profile_generation
            cursor = int(after_seq) if same_generation else 0
            oldest_seq = self._profiles[0][1]["seq"] if self._profiles else self._profile_seq + 1
            newest_seq = self._profiles[-1][1]["seq"] if self._profiles else 0
            truncated = same_generation and cursor < oldest_seq - 1
            batch = [profile for _, profile in self._profiles if profile["seq"] > cursor][:max_spokes]
            latest_seq = batch[-1]["seq"] if batch else cursor
            return {
                "generation": self._profile_generation,
                "oldest_seq": oldest_seq,
                "latest_seq": latest_seq,
                "has_more": latest_seq < newest_seq,
                "truncated": truncated,
                "spokes": batch,
                "time_basis": "host_packet_parse_monotonic",
                "rf_time_calibrated": False,
            }

    @staticmethod
    def _angle_sample(sample: Mapping[str, Any]) -> AngleSample:
        return AngleSample(
            float(sample["query_started_monotonic_s"]),
            float(sample["query_completed_monotonic_s"]),
            float(sample["angle_deg"]),
        )

    def add_motor_sample(self, sample: Mapping[str, Any]) -> dict[str, Any] | None:
        """Return a turn summary only on the sample that completes it."""
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
                return None
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
            if self._cumulative_angle_deg - self._turn_start_progress >= 360.0:
                self._finish_turn_diagnostics()
                return self._latest_complete_turn
            return None

    def add_frame(self, frame: Mapping[str, Any]) -> None:
        payload = dict(frame)
        frame_time = float(payload["host_monotonic_s"])
        if not math.isfinite(frame_time):
            raise ValueError("frame host_monotonic_s must be finite")
        with self._lock:
            frame_num = int(payload.get("frame_num", -1))
            profile = payload.get("range_profile")
            has_profile = (isinstance(profile, (list, tuple)) and 0 < len(profile) <= 2048 and
                           all(type(value) is int and 0 <= value <= 65535 for value in profile))
            tlv_types = payload.get("tlv_types")
            has_tlv2 = 2 in tlv_types if isinstance(tlv_types, (list, tuple)) else has_profile
            if len(self._turn_raw_frames) == self._turn_raw_frames.maxlen:
                self._turn_evidence_truncated = True
            self._turn_raw_frames.append((frame_num, frame_time, has_tlv2))
            if self._last_raw_frame_num is not None and frame_num >= 0:
                step = (frame_num - self._last_raw_frame_num) % 2**32
                if 1 < step < 100000:
                    self._turn_raw_missing += step - 1
            if frame_num >= 0:
                if self._turn_first_raw_frame_num is None:
                    self._turn_first_raw_frame_num = frame_num
                self._last_raw_frame_num = frame_num
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
            if has_profile:
                payload["range_profile"] = list(profile)
            else:
                payload.pop("range_profile", None)
                self._turn_tlv2_missing += 1
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
            profile = frame.get("range_profile")
            if profile is not None:
                bin_size = frame.get("range_profile_bin_size_m")
                try:
                    bin_size = float(bin_size)
                    if not math.isfinite(bin_size) or bin_size <= 0:
                        bin_size = None
                except (TypeError, ValueError, OverflowError):
                    bin_size = None
                self._profile_seq += 1
                self._profiles.append((frame_progress, {
                    "seq": self._profile_seq,
                    "frame_num": int(frame.get("frame_num", -1)),
                    "yaw_deg": registration.yaw_deg,
                    "host_monotonic_s": frame_time,
                    "range_profile": profile,
                    "range_profile_bin_size_m": bin_size,
                    "rf_time_calibrated": False,
                }))
                self._turn_spokes.append((float(registration.yaw_deg),
                                          int(frame.get("frame_num", -1)), frame_time))
                while len(self._profiles) > self._max_profile_spokes:
                    self._profile_capacity_eviction_progress, _ = self._profiles.popleft()
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
            if stale and (self._registered or self._profiles):
                self._clear_map("telemetry_stale")
            points = [] if stale else [point for _, batch in self._registered for point in batch]
            # Bearings describe retained observations, not antenna coverage or
            # painted sector area. Include the wrap gap even during the first turn.
            angles = sorted({round(profile['yaw_deg'] % 360.0, 6) % 360.0
                             for _, profile in self._profiles})
            max_gap = max(((angles[(i + 1) % len(angles)] - angle) % 360.0
                           for i, angle in enumerate(angles)), default=None)
            if len(angles) == 1:
                max_gap = 360.0
            eviction = self._profile_capacity_eviction_progress
            coverage = {
                "window": "rolling_360_deg",
                "phase": "stale" if stale else (
                    "rolling" if self._cumulative_angle_deg >= 360.0 else "collecting"),
                "observed_travel_deg": self._cumulative_angle_deg,
                "valid_spokes": len(self._profiles),
                "unique_azimuths": len(angles),
                "max_gap_deg": max_gap,
                "capacity_limited": (eviction is not None and
                                     self._cumulative_angle_deg - eviction < 360.0),
            }
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
                "coverage": coverage,
                "latest_complete_turn": self._latest_complete_turn,
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
