"""Propagate IMU/radar latency choices into compensated point-cloud metrics."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from simulation.run_v04_61_imu_pose_compensation import run as compensate_run
from simulation.run_v04_63_imu_ingest import run as ingest_run


def run(point_cloud: Path, radar_frames: Path, raw_imu: Path, output: Path, *, delays_s: tuple[float, ...] = (-0.01, -0.005, 0.0, 0.005, 0.01), timestamp_unit: str = "ms", max_gap_s: float = 0.06, nominal_pose: Path | None = None) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []
    for delay in delays_s:
        name = f"delay_{delay:+.6f}s".replace("+", "p").replace("-", "m").replace(".", "_")
        delay_dir = output / name
        ingest = ingest_run(radar_frames, raw_imu, delay_dir / "ingest", timestamp_unit=timestamp_unit, latency_s=delay, max_gap_s=max_gap_s)
        alignment = ingest["alignment"]
        if alignment["quality_pass"]:
            aligned_path = delay_dir / "ingest" / "alignment" / "aligned_imu.csv"
            compensation_imu = delay_dir / "ingest" / "alignment" / "aligned_imu_for_compensation.csv"
            with aligned_path.open(encoding="utf-8", newline="") as handle:
                aligned_rows = list(csv.DictReader(handle))
            with compensation_imu.open("w", encoding="utf-8", newline="") as handle:
                fields = list(aligned_rows[0])
                if "time_s" not in fields:
                    fields.insert(1, "time_s")
                writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader()
                for aligned_row in aligned_rows:
                    aligned_row["time_s"] = aligned_row.get("radar_time_s", "")
                    writer.writerow(aligned_row)
            compensated = compensate_run(point_cloud, compensation_imu, delay_dir / "compensated", nominal_pose)
            rows.append({
                "latency_correction_s": delay, "quality_pass": True,
                "max_position_change_m": compensated["max_position_change_m"],
                "max_abs_azimuth_change_deg": compensated["max_abs_azimuth_change_deg"],
                "max_abs_elevation_change_deg": compensated["max_abs_elevation_change_deg"],
                "point_count": compensated["point_count"],
            })
        else:
            rows.append({
                "latency_correction_s": delay, "quality_pass": False,
                "max_position_change_m": "", "max_abs_azimuth_change_deg": "",
                "max_abs_elevation_change_deg": "", "point_count": 0,
            })
    with (output / "latency_pointcloud_metrics.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    passing = [row for row in rows if row["quality_pass"]]
    summary = {
        "status": "completed_latency_to_pointcloud_propagation",
        "delay_count": len(rows), "passing_delay_count": len(passing),
        "quality_pass_delays_s": [row["latency_correction_s"] for row in passing],
        "max_position_change_m_over_passing": max(row["max_position_change_m"] for row in passing) if passing else None,
        "max_abs_azimuth_change_deg_over_passing": max(row["max_abs_azimuth_change_deg"] for row in passing) if passing else None,
        "max_abs_elevation_change_deg_over_passing": max(row["max_abs_elevation_change_deg"] for row in passing) if passing else None,
        "point_cloud_source_is_hardware": False, "imu_source_is_hardware": False,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# V0.4.65 延迟误差到海杂波点云指标传播",
        "",
        f"扫描延迟数：{len(rows)}；通过时间质量门：{summary['quality_pass_delays_s']} s。",
        f"通过场景中的最大坐标变化：{summary['max_position_change_m_over_passing']}; 最大方位变化：{summary['max_abs_azimuth_change_deg_over_passing']}°；最大俯仰变化：{summary['max_abs_elevation_change_deg_over_passing']}°。",
        "",
        "## 如何分析",
        "",
        "`latency_pointcloud_metrics.csv` 将每个延迟的时间对齐质量直接连接到 V0.4.61 的点云姿态补偿结果。先剔除 `quality_pass=false` 的延迟，再比较坐标、方位和俯仰变化；这条链路可以用于真实 IMU 接入后的同步误差预算。",
        "",
        "## 证据边界",
        "",
        "当前点云和 IMU 均为合成/历史仿真输入，不能作为实船误差结论。输出指标描述的是姿态解释变化，不是海面目标真实运动或 AoA 硬件误差。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--point-cloud", type=Path, required=True)
    parser.add_argument("--radar-frames", type=Path, required=True)
    parser.add_argument("--raw-imu", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--nominal-pose", type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.point_cloud.resolve(), args.radar_frames.resolve(), args.raw_imu.resolve(), args.output.resolve(), nominal_pose=args.nominal_pose.resolve() if args.nominal_pose else None), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
