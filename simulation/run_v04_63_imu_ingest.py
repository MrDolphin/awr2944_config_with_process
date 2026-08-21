"""Normalize IMU timestamp units/latency, then run V0.4.62 alignment."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from simulation.run_v04_62_imu_time_alignment import run as align_run


UNIT_SCALE = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}


def _read(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def run(radar_frames_path: Path, raw_imu_path: Path, output: Path, *, timestamp_unit: str = "s", latency_s: float = 0.0, max_gap_s: float = 0.2) -> dict:
    if timestamp_unit not in UNIT_SCALE:
        raise ValueError(f"timestamp_unit must be one of {sorted(UNIT_SCALE)}")
    raw_rows = _read(raw_imu_path)
    if not raw_rows:
        raise ValueError("raw IMU file is empty")
    timestamp_key = "timestamp" if "timestamp" in raw_rows[0] else "time_s"
    scale = UNIT_SCALE[timestamp_unit]
    normalized = []
    for row in raw_rows:
        raw_timestamp = float(row[timestamp_key])
        # latency_s is the correction added to the source timestamp to express
        # the sample in the radar clock. Positive means IMU event is later.
        normalized.append({
            "time_s": raw_timestamp * scale + latency_s,
            "roll_deg": float(row["roll_deg"]),
            "pitch_deg": float(row["pitch_deg"]),
            "yaw_deg": float(row["yaw_deg"]),
        })
    output.mkdir(parents=True, exist_ok=True)
    normalized_path = output / "normalized_imu.csv"
    with normalized_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(normalized[0]))
        writer.writeheader(); writer.writerows(normalized)
    alignment = align_run(radar_frames_path, normalized_path, output / "alignment", max_gap_s=max_gap_s)
    summary = {
        "status": "completed_imu_ingest_and_alignment",
        "timestamp_key": timestamp_key,
        "timestamp_unit": timestamp_unit,
        "timestamp_scale_to_seconds": scale,
        "latency_correction_s": latency_s,
        "raw_imu_sample_count": len(raw_rows),
        "alignment": alignment,
        "source_is_hardware_imu": False,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# V0.4.63 IMU 数据接入与时间归一化",
        "",
        f"原始时间字段：`{timestamp_key}`；原始单位：`{timestamp_unit}`；换算到秒的比例：{scale:g}。",
        f"固定延迟修正：{latency_s:.6f} s。约定为 `effective_time_s = raw_timestamp * scale + latency_correction_s`。",
        f"原始 IMU 样本：{len(raw_rows)}；对齐质量门：{alignment['quality_pass']}。",
        "",
        "## 如何分析",
        "",
        "先检查 `normalized_imu.csv`，确认时间是否已经转换成秒并应用延迟修正；再检查 `alignment/aligned_imu.csv` 和 `alignment/summary.json`。只有当 coverage、单调性和最大间隔均通过时，才能把对齐姿态送入 V0.4.61。",
        "",
        "## 证据边界",
        "",
        "当前示例用于验证单位和延迟逻辑，`source_is_hardware_imu=false`。真实数据接入时必须把 timestamp 来源、设备时钟、延迟测量方法和符号约定一并记录，不能仅凭文件名猜测单位。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--radar-frames", type=Path, required=True)
    parser.add_argument("--raw-imu", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timestamp-unit", choices=sorted(UNIT_SCALE), default="s")
    parser.add_argument("--latency-s", type=float, default=0.0)
    parser.add_argument("--max-gap-s", type=float, default=0.2)
    args = parser.parse_args()
    print(json.dumps(run(args.radar_frames.resolve(), args.raw_imu.resolve(), args.output.resolve(), timestamp_unit=args.timestamp_unit, latency_s=args.latency_s, max_gap_s=args.max_gap_s), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
