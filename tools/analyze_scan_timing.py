"""Summarize *received* radar frame timing from PointCloudRecorder frames.jsonl.

This cannot measure RF acquisition time, encoder latency, or root cause of a
missing frame number. It does not connect to or configure radar hardware.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
from typing import Any, Iterable, Mapping


def frame_period_from_metadata(metadata: Mapping[str, Any]) -> float:
    """Read the sole active frameCfg period from a recorder CFG snapshot."""
    content = metadata.get("active_config", {}).get("content")
    if not isinstance(content, str) or not content.strip():
        raise ValueError("metadata has no active_config.content")
    commands = [line.split() for raw in content.splitlines()
                if (line := raw.strip()) and not line.startswith(("%", "#"))
                and line.split()[0] == "frameCfg"]
    if len(commands) != 1 or len(commands[0]) < 7:
        raise ValueError("expected exactly one complete active frameCfg")
    period_ms = float(commands[0][6])
    if not math.isfinite(period_ms) or period_ms <= 0:
        raise ValueError("invalid active frameCfg period")
    return period_ms


def analyze_records(records: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    frames = list(records)
    if len(frames) < 2:
        raise ValueError("at least two recorded frames are required")
    times = [float(frame["host_monotonic_s"]) for frame in frames]
    if not all(math.isfinite(value) for value in times):
        raise ValueError("frame times must be finite")
    intervals = [later - earlier for earlier, later in zip(times, times[1:])]
    if any(interval <= 0 for interval in intervals):
        raise ValueError("frame times must be strictly increasing")

    missing = 0
    duplicate = 0
    for earlier, later in zip(frames, frames[1:]):
        step = (int(later["frame_num"]) - int(earlier["frame_num"])) % 2**32
        if step == 0:
            duplicate += 1
        elif step <= 2**31:
            missing += step - 1
        else:
            raise ValueError("frame counter moved backward or reset")

    sorted_intervals = sorted(intervals)
    p95_index = math.ceil(0.95 * len(sorted_intervals)) - 1
    return {
        "frame_count": len(frames),
        "duration_s": times[-1] - times[0],
        "effective_received_hz": len(intervals) / (times[-1] - times[0]),
        "median_interval_ms": statistics.median(intervals) * 1000.0,
        "p95_interval_ms": sorted_intervals[p95_index] * 1000.0,
        "max_interval_ms": sorted_intervals[-1] * 1000.0,
        "missing_frame_numbers": missing,
        "duplicate_frame_numbers": duplicate,
        "time_basis": "host_packet_parse_monotonic",
    }


def compare_frame_timebases(
    records: Iterable[Mapping[str, Any]], *, frame_period_ms: float, rpm: float,
) -> dict[str, Any]:
    """Compare host parse spacing with a *hypothetical* frame-counter clock.

    The counter clock removes host delivery jitter, but its constant offset to
    encoder samples and RF acquisition is unknown. Never use this as calibrated
    frame azimuth, and never infer that missing counters were received.
    """
    frames = list(records)
    observed = analyze_records(frames)
    if not math.isfinite(frame_period_ms) or frame_period_ms <= 0:
        raise ValueError("frame period must be positive and finite")
    if not math.isfinite(rpm) or rpm <= 0:
        raise ValueError("rpm must be positive and finite")
    period_s = frame_period_ms / 1000.0
    counter_steps = []
    for earlier, later in zip(frames, frames[1:]):
        step = (int(later["frame_num"]) - int(earlier["frame_num"])) % 2**32
        if not 0 < step <= 2**31:
            raise ValueError("frame counter must advance without reset or duplicate")
        counter_steps.append(step)
    host_intervals = [float(b["host_monotonic_s"]) - float(a["host_monotonic_s"])
                      for a, b in zip(frames, frames[1:])]
    estimated_intervals = [step * period_s for step in counter_steps]
    degrees_per_second = rpm * 6.0
    residuals = []
    elapsed_counter_s = 0.0
    first_host_s = float(frames[0]["host_monotonic_s"])
    for frame, step in zip(frames[1:], counter_steps):
        elapsed_counter_s += step * period_s
        residuals.append(float(frame["host_monotonic_s"]) - first_host_s
                         - elapsed_counter_s)
    return {
        "frame_count": len(frames),
        "missing_frame_numbers": observed["missing_frame_numbers"],
        "frame_period_ms_from_cfg": frame_period_ms,
        "rpm_for_angle_budget": rpm,
        "host_gaps_over_5_deg": sum(dt * degrees_per_second > 5 for dt in host_intervals),
        "estimated_gaps_over_5_deg": sum(dt * degrees_per_second > 5 for dt in estimated_intervals),
        "host_max_angle_step_deg": max(host_intervals) * degrees_per_second,
        "estimated_max_angle_step_deg": max(estimated_intervals) * degrees_per_second,
        "host_intervals_over_twice_cfg": sum(dt > 2 * period_s for dt in host_intervals),
        "host_intervals_under_half_cfg": sum(dt < .5 * period_s for dt in host_intervals),
        "parse_minus_counter_elapsed_end_ms": residuals[-1] * 1000,
        "parse_minus_counter_elapsed_range_ms": (max(residuals) - min(residuals)) * 1000,
        "candidate_time_basis": "frame_counter_x_cfg_period_estimate_only",
        "rf_time_calibrated": False,
        "encoder_time_offset_calibrated": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path, help="PointCloudRecorder frames.jsonl")
    parser.add_argument("--output", type=Path, help="Write JSON summary to this file")
    parser.add_argument("--metadata", type=Path,
                        help="Capture metadata.json with active_config.content")
    parser.add_argument("--rpm", type=float,
                        help="Measured/reported motor RPM for angle-step budget; requires --metadata")
    args = parser.parse_args()
    if (args.metadata is None) != (args.rpm is None):
        parser.error("--metadata and --rpm must be supplied together")
    with args.frames.open(encoding="utf-8") as handle:
        frames = [json.loads(line) for line in handle if line.strip()]
    summary = analyze_records(frames)
    if args.metadata is not None:
        metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
        summary["timebase_comparison"] = compare_frame_timebases(
            frames, frame_period_ms=frame_period_from_metadata(metadata), rpm=args.rpm)
    rendered = json.dumps(summary, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
