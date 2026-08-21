"""Scan fixed IMU-to-radar latency corrections and summarize quality."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from simulation.run_v04_63_imu_ingest import run as ingest_run


def _read(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def run(radar_frames: Path, raw_imu: Path, output: Path, delays_s: tuple[float, ...] = (-0.01, -0.005, 0.0, 0.005, 0.01), timestamp_unit: str = "ms", max_gap_s: float = 0.06) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    scan_rows: list[dict] = []
    aligned_by_delay: dict[float, list[dict[str, str]]] = {}
    for delay in delays_s:
        delay_key = f"delay_{delay:+.6f}s".replace("+", "p").replace("-", "m").replace(".", "_")
        result = ingest_run(radar_frames, raw_imu, output / delay_key, timestamp_unit=timestamp_unit, latency_s=delay, max_gap_s=max_gap_s)
        aligned = _read(output / delay_key / "alignment" / "aligned_imu.csv")
        aligned_by_delay[delay] = aligned
        scan_rows.append({
            "latency_correction_s": delay,
            "quality_pass": result["alignment"]["quality_pass"],
            "coverage_ok": result["alignment"]["coverage_ok"],
            "max_source_imu_gap_s": result["alignment"]["max_source_imu_gap_s"],
            "max_alignment_gap_s": result["alignment"]["max_alignment_gap_s"],
            "radar_time_start_s": result["alignment"]["radar_time_start_s"],
            "radar_time_end_s": result["alignment"]["radar_time_end_s"],
            "effective_imu_start_s": result["alignment"]["imu_time_start_s"],
            "effective_imu_end_s": result["alignment"]["imu_time_end_s"],
        })
    reference_delay = min(delays_s, key=lambda delay: abs(delay - 0.005))
    reference = aligned_by_delay[reference_delay]
    orientation_rows: list[dict] = []
    for delay, aligned in aligned_by_delay.items():
        for ref, current in zip(reference, aligned):
            orientation_rows.append({
                "latency_correction_s": delay,
                "frame": int(current["frame"]),
                "reference_latency_s": reference_delay,
                "roll_difference_deg": float(current["roll_deg"]) - float(ref["roll_deg"]),
                "pitch_difference_deg": float(current["pitch_deg"]) - float(ref["pitch_deg"]),
                "yaw_difference_deg": float(current["yaw_deg"]) - float(ref["yaw_deg"]),
            })
    with (output / "latency_scan.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(scan_rows[0])); writer.writeheader(); writer.writerows(scan_rows)
    with (output / "latency_pose_difference.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(orientation_rows[0])); writer.writeheader(); writer.writerows(orientation_rows)
    passing = [row for row in scan_rows if row["quality_pass"]]
    summary = {
        "status": "completed_imu_latency_scan",
        "delay_count": len(delays_s), "passing_delay_count": len(passing),
        "delays_s": list(delays_s), "reference_delay_s": reference_delay,
        "quality_pass_delays_s": [row["latency_correction_s"] for row in passing],
        "max_abs_roll_difference_deg": max(abs(row["roll_difference_deg"]) for row in orientation_rows),
        "max_abs_pitch_difference_deg": max(abs(row["pitch_difference_deg"]) for row in orientation_rows),
        "max_abs_yaw_difference_deg": max(abs(row["yaw_difference_deg"]) for row in orientation_rows),
        "source_is_hardware_imu": False,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# V0.4.64 IMU/雷达固定延迟扫描",
        "",
        f"扫描延迟：{list(delays_s)} s；通过质量门：{summary['quality_pass_delays_s']} s。",
        f"参考延迟：{reference_delay:+.6f} s；相对参考姿态最大差异：roll {summary['max_abs_roll_difference_deg']:.6f}°，pitch {summary['max_abs_pitch_difference_deg']:.6f}°，yaw {summary['max_abs_yaw_difference_deg']:.6f}°。",
        "",
        "## 如何解释",
        "",
        "`latency_scan.csv` 判断每个延迟修正是否覆盖全部雷达 frame；`latency_pose_difference.csv` 把每个延迟下的插值姿态与参考延迟比较。延迟扫描用于定位时间同步敏感区间，不等于从仿真数据反推出真实硬件延迟。",
        "",
        "## 证据边界",
        "",
        "当前输入是合成 IMU 和雷达时间序列，source_is_hardware_imu=false。真实数据应以外部同步脉冲、设备日志或已知事件测量固定延迟，并把测量不确定度纳入扫描范围。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--radar-frames", type=Path, required=True)
    parser.add_argument("--raw-imu", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.radar_frames.resolve(), args.raw_imu.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
