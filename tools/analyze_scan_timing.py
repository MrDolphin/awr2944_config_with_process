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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path, help="PointCloudRecorder frames.jsonl")
    parser.add_argument("--output", type=Path, help="Write JSON summary to this file")
    args = parser.parse_args()
    with args.frames.open(encoding="utf-8") as handle:
        summary = analyze_records(json.loads(line) for line in handle if line.strip())
    rendered = json.dumps(summary, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
