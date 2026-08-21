"""Align an IMU time series to radar frame timestamps with explicit quality gates."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def _read(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _write(path: Path, rows: list[dict]) -> None:
    fields = list(rows[0]) if rows else ["frame", "radar_time_s"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def run(radar_frames_path: Path, imu_path: Path, output: Path, max_gap_s: float = 0.2) -> dict:
    radar_rows = _read(radar_frames_path)
    imu_rows = _read(imu_path)
    if not radar_rows or not imu_rows:
        raise ValueError("radar frame and IMU files must both be non-empty")
    radar_times = np.asarray([float(row["time_s"]) for row in radar_rows], dtype=float)
    imu_times = np.asarray([float(row["time_s"]) for row in imu_rows], dtype=float)
    if np.any(np.diff(radar_times) < 0) or np.any(np.diff(imu_times) < 0):
        raise ValueError("timestamps must be monotonic non-decreasing")
    if np.any(np.diff(imu_times) == 0):
        raise ValueError("IMU timestamps must be strictly increasing")
    covered = bool(radar_times.min() >= imu_times.min() and radar_times.max() <= imu_times.max())
    source_gaps = np.diff(imu_times)
    max_source_gap = float(np.max(source_gaps)) if len(source_gaps) else 0.0
    fields = ("roll_deg", "pitch_deg", "yaw_deg")
    values = {field: np.asarray([float(row[field]) for row in imu_rows], dtype=float) for field in fields}
    aligned: list[dict] = []
    for row, radar_time in zip(radar_rows, radar_times):
        index = int(np.searchsorted(imu_times, radar_time, side="right") - 1)
        index = max(0, min(index, len(imu_times) - 2))
        bracket_gap = float(imu_times[index + 1] - imu_times[index])
        interpolated = {field: float(np.interp(radar_time, imu_times, values[field])) for field in fields}
        aligned.append({
            "frame": int(row["frame"]), "radar_time_s": float(radar_time),
            "imu_left_time_s": float(imu_times[index]), "imu_right_time_s": float(imu_times[index + 1]),
            "interpolation_gap_s": bracket_gap, "interpolation_alpha": float((radar_time - imu_times[index]) / bracket_gap),
            **interpolated,
            "alignment_status": "interpolated_or_exact" if covered and bracket_gap <= max_gap_s else "quality_gate_failed",
        })
    output.mkdir(parents=True, exist_ok=True)
    _write(output / "aligned_imu.csv", aligned)
    gap_values = [row["interpolation_gap_s"] for row in aligned]
    quality_pass = covered and max_source_gap <= max_gap_s and all(row["alignment_status"] != "quality_gate_failed" for row in aligned)
    summary = {
        "status": "completed_imu_radar_time_alignment",
        "radar_frame_count": len(radar_rows), "imu_sample_count": len(imu_rows),
        "radar_time_start_s": float(radar_times.min()), "radar_time_end_s": float(radar_times.max()),
        "imu_time_start_s": float(imu_times.min()), "imu_time_end_s": float(imu_times.max()),
        "coverage_ok": covered, "max_source_imu_gap_s": max_source_gap,
        "max_alignment_gap_s": max(gap_values), "max_allowed_gap_s": max_gap_s,
        "quality_pass": quality_pass, "source_is_hardware_imu": False,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# V0.4.62 IMU/雷达时间对齐",
        "",
        f"雷达 frame 数：{len(radar_rows)}；IMU 样本数：{len(imu_rows)}。",
        f"雷达时间范围：{radar_times.min():.6f}～{radar_times.max():.6f} s；IMU 时间范围：{imu_times.min():.6f}～{imu_times.max():.6f} s。",
        f"最大 IMU 源间隔：{max_source_gap:.6f} s；允许最大间隔：{max_gap_s:.6f} s；coverage_ok={covered}；quality_pass={quality_pass}。",
        "",
        "## 如何解释",
        "",
        "`aligned_imu.csv` 的每一行对应一个雷达 frame。姿态由相邻 IMU 样本线性插值得到；`interpolation_alpha` 为 0 表示恰好落在左侧样本，1 表示右侧样本。任何雷达时间超出 IMU 覆盖范围、IMU 时间戳不单调或采样间隔超过门限，都会使质量门失败。",
        "",
        "## 证据边界",
        "",
        "当前输入为合成时间序列，source_is_hardware_imu=false。真实数据接入时还需加入时间戳来源、时钟偏差、串口延迟和姿态滤波延迟记录。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--radar-frames", type=Path, required=True)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-gap-s", type=float, default=0.2)
    args = parser.parse_args()
    print(json.dumps(run(args.radar_frames.resolve(), args.imu.resolve(), args.output.resolve(), args.max_gap_s), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
