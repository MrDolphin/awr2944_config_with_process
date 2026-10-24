"""Offline, provisional 360-degree point registration for recorded radar data.

Reads an existing PointCloudRecorder capture and independent timestamped motor
samples. Never accesses serial ports, changes a CFG, or starts a motor.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter
import csv
import json
import math
from pathlib import Path
import struct
import sys
from typing import Any

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scan_registration import AngleSample, Registration, estimate_frame_yaw, register_points


POINT_COLUMNS = [
    "record_index", "frame_num", "point_index", "yaw_deg",
    "x_m", "y_m", "z_m", "radar_x_m", "radar_y_m", "radar_z_m",
    "v_mps", "snr_db", "noise_db",
]
MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"


def extract_range_profile(packet: bytes) -> list[int] | None:
    """Extract TLV type 2 as raw uint16 range bins; no physical calibration."""
    if len(packet) < 40 or packet[:8] != MAGIC_WORD:
        raise ValueError("invalid radar packet header")
    header = struct.unpack_from("<8sIIIIIIII", packet)
    if header[2] != len(packet):
        raise ValueError("raw packet length disagrees with radar header")
    offset = 40
    for _ in range(header[7]):
        if offset + 8 > len(packet):
            raise ValueError("truncated TLV header")
        kind, length = struct.unpack_from("<II", packet, offset)
        offset += 8
        if length > len(packet) - offset:
            raise ValueError("truncated TLV payload")
        if kind == 2:
            if length % 2:
                raise ValueError("range profile TLV has odd byte length")
            return list(struct.unpack_from(f"<{length // 2}H", packet, offset))
        offset += length
    return None


def _jsonl(path: Path) -> list[dict[str, Any]]:
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def _load_inputs(capture_dir: Path, samples_path: Path):
    frames = _jsonl(capture_dir / "frames.jsonl")
    if not frames:
        raise ValueError("capture contains no frames")
    frame_times = []
    for index, frame in enumerate(frames):
        if int(frame["record_index"]) != index:
            raise ValueError("frame record_index must be contiguous from zero")
        frame_time = float(frame["host_monotonic_s"])
        if not math.isfinite(frame_time) or (frame_times and frame_time <= frame_times[-1]):
            raise ValueError("frame host_monotonic_s must strictly increase")
        frame_times.append(frame_time)

    samples = [AngleSample(
        float(row["query_started_monotonic_s"]),
        float(row["query_completed_monotonic_s"]), float(row["angle_deg"]),
    ) for row in _jsonl(samples_path)]
    if len(samples) < 2:
        raise ValueError("at least two motor samples are required")
    sample_times = []
    for sample in samples:
        if (not all(math.isfinite(value) for value in (
            sample.query_started_monotonic_s, sample.query_completed_monotonic_s,
            sample.angle_deg,
        )) or sample.query_duration_s < 0 or
                (sample_times and sample.midpoint_s <= sample_times[-1])):
            raise ValueError("motor samples need finite, increasing query times")
        sample_times.append(sample.midpoint_s)

    point_path = capture_dir / "points.csv"
    with point_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {"record_index", "frame_num", "point_index", "x_m", "y_m", "z_m"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("points.csv is missing required columns")
        last_index = -1
        for point in reader:
            index = int(point["record_index"])
            if (index < 0 or index < last_index or index >= len(frames) or
                    int(point["frame_num"]) != int(frames[index]["frame_num"])):
                raise ValueError("point record_index/frame_num does not match frames.jsonl")
            last_index = index
    return frames, frame_times, samples, sample_times


def register_capture(
    capture_dir: str | Path,
    motor_samples_path: str | Path,
    output_dir: str | Path,
    *,
    direction: int,
    zero_deg: float,
    mounting_yaw_deg: float,
    mounting_roll_deg: float,
    max_gap_s: float = 0.1,
    max_query_duration_s: float = 0.05,
    max_span_deg: float = 10.0,
    max_possible_rpm: float = 130.0,
) -> dict[str, Any]:
    """Write derived frame/point files, preserving the original capture."""
    capture_dir = Path(capture_dir)
    samples_path = Path(motor_samples_path)
    output_dir = Path(output_dir)
    if output_dir.exists():
        raise FileExistsError(f"output already exists: {output_dir}")
    if direction not in (-1, 1):
        raise ValueError("direction must be +1 or -1 in encoder angle coordinates")
    if not all(math.isfinite(value) for value in
               (zero_deg, mounting_yaw_deg, mounting_roll_deg)):
        raise ValueError("mounting angles must be finite")
    if not all(math.isfinite(value) and value > 0 for value in
               (max_gap_s, max_query_duration_s, max_span_deg, max_possible_rpm)):
        raise ValueError("quality limits must be finite and positive")
    frames, frame_times, samples, sample_times = _load_inputs(capture_dir, samples_path)
    registrations: list[Registration] = []
    statuses: Counter[str] = Counter()
    observed_bins: set[int] = set()
    for frame_time in frame_times:
        right = bisect_right(sample_times, frame_time)
        if right == 0 or right >= len(samples):
            registration = Registration("unbracketed")
        else:
            registration = estimate_frame_yaw(
                frame_time, samples[right - 1], samples[right], direction=direction,
                zero_deg=zero_deg, mounting_yaw_deg=mounting_yaw_deg,
                max_gap_s=max_gap_s, max_query_duration_s=max_query_duration_s,
                max_span_deg=max_span_deg, max_possible_rpm=max_possible_rpm,
            )
        registrations.append(registration)
        statuses[registration.status] += 1
        if registration.yaw_deg is not None:
            observed_bins.add(int(registration.yaw_deg // 10) % 36)

    output_dir.mkdir(parents=True, exist_ok=False)
    registered_points = 0
    with (output_dir / "registered_frames.jsonl").open("w", encoding="utf-8") as frame_output:
        for frame, registration in zip(frames, registrations):
            frame_output.write(json.dumps({
                "record_index": frame["record_index"],
                "frame_num": frame["frame_num"],
                "host_monotonic_s": frame["host_monotonic_s"],
                "status": registration.status,
                "yaw_deg": registration.yaw_deg,
                "sample_gap_s": registration.sample_gap_s,
                "angular_span_deg": registration.angular_span_deg,
                "time_basis": registration.time_basis,
            }, ensure_ascii=False) + "\n")

    with (capture_dir / "points.csv").open(newline="", encoding="utf-8") as source, \
            (output_dir / "registered_points.csv").open("w", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=POINT_COLUMNS)
        writer.writeheader()
        for point in csv.DictReader(source):
            index = int(point["record_index"])
            registration = registrations[index]
            radar_point = {"x": float(point["x_m"]),
                           "y": float(point["y_m"]),
                           "z": float(point["z_m"])}
            transformed = register_points([radar_point], registration,
                                          mounting_roll_deg=mounting_roll_deg)
            if transformed is None:
                continue
            writer.writerow({
                "record_index": index,
                "frame_num": point["frame_num"],
                "point_index": point["point_index"],
                "yaw_deg": registration.yaw_deg,
                "x_m": transformed[0]["x"],
                "y_m": transformed[0]["y"],
                "z_m": transformed[0]["z"],
                "radar_x_m": point["x_m"],
                "radar_y_m": point["y_m"],
                "radar_z_m": point["z_m"],
                "v_mps": point.get("v_mps", ""),
                "snr_db": point.get("snr_db", ""),
                "noise_db": point.get("noise_db", ""),
            })
            registered_points += 1

    range_profile_frames = 0
    raw_path = capture_dir / "frames.tlv"
    with (output_dir / "polar_range_profiles.jsonl").open("w", encoding="utf-8") as target:
        raw_source = raw_path.open("rb") if raw_path.exists() else None
        try:
            for frame, registration in zip(frames, registrations):
                if registration.status != "registered":
                    continue
                length = int(frame.get("raw_length") or 0)
                if length == 0:
                    continue
                if raw_source is None:
                    raise ValueError("frames.tlv is missing despite nonempty raw frame")
                raw_source.seek(int(frame["raw_offset"]))
                packet = raw_source.read(length)
                try:
                    profile = extract_range_profile(packet)
                except ValueError as exc:
                    raise ValueError(f"raw frame {frame['record_index']}: {exc}") from exc
                if profile is None:
                    continue
                target.write(json.dumps({
                    "record_index": frame["record_index"],
                    "frame_num": frame["frame_num"],
                    "host_monotonic_s": frame["host_monotonic_s"],
                    "yaw_deg": registration.yaw_deg,
                    "range_bins_raw": profile,
                    "range_axis": "bin_index_uncalibrated",
                    "value_units": "firmware_raw_uint16",
                    "time_basis": registration.time_basis,
                }, ensure_ascii=False) + "\n")
                range_profile_frames += 1
        finally:
            if raw_source is not None:
                raw_source.close()

    summary = {
        "frame_count": len(frames),
        "registered_frames": statuses["registered"],
        "unregistered_frames": len(frames) - statuses["registered"],
        "registered_points": registered_points,
        "range_profile_frames": range_profile_frames,
        "statuses": dict(statuses),
        "observed_azimuth_bins_10deg": sorted(observed_bins),
        "time_basis": "host_packet_parse_monotonic",
        "rf_time_calibrated": False,
        "mounting": {"direction": direction, "zero_deg": zero_deg,
                     "yaw_deg": mounting_yaw_deg, "roll_deg": mounting_roll_deg},
        "limits": {"max_gap_s": max_gap_s,
                   "max_query_duration_s": max_query_duration_s,
                   "max_span_deg": max_span_deg,
                   "max_possible_rpm": max_possible_rpm},
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("motor_samples_jsonl", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--angle-direction", choices=("increasing", "decreasing"),
                        required=True, help="Measured encoder angle trend, not motor CW/CCW label")
    parser.add_argument("--zero-deg", type=float, required=True)
    parser.add_argument("--mounting-yaw-deg", type=float, required=True)
    parser.add_argument("--mounting-roll-deg", type=float, required=True)
    parser.add_argument("--max-gap-ms", type=float, default=100.0)
    parser.add_argument("--max-query-ms", type=float, default=50.0)
    parser.add_argument("--max-span-deg", type=float, default=10.0)
    parser.add_argument("--max-possible-rpm", type=float, default=130.0)
    args = parser.parse_args()
    result = register_capture(
        args.capture_dir, args.motor_samples_jsonl, args.output_dir,
        direction=1 if args.angle_direction == "increasing" else -1,
        zero_deg=args.zero_deg, mounting_yaw_deg=args.mounting_yaw_deg,
        mounting_roll_deg=args.mounting_roll_deg,
        max_gap_s=args.max_gap_ms / 1000.0,
        max_query_duration_s=args.max_query_ms / 1000.0,
        max_span_deg=args.max_span_deg, max_possible_rpm=args.max_possible_rpm,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
