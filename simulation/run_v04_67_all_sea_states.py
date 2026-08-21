"""Build a complete sea-state coverage matrix from V0.44 point-cloud HDF5s."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np


def _read(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _stats(values: np.ndarray) -> tuple[float | None, float | None, float | None, float | None, float | None]:
    if values.size == 0:
        return None, None, None, None, None
    return tuple(float(item) for item in (np.mean(values), np.std(values), np.percentile(values, 5), np.percentile(values, 50), np.percentile(values, 95)))


def run(point_cloud_root: Path, output: Path, propagated_metrics: Path | None = None) -> dict:
    case_rows = _read(point_cloud_root / "case_summary.csv")
    propagated = {}
    if propagated_metrics and propagated_metrics.exists():
        for row in _read(propagated_metrics):
            propagated[row["case_id"]] = row
    rows: list[dict] = []
    for case in case_rows:
        case_id = case["case_id"]
        h5_path = Path(case["output_h5"])
        if not h5_path.is_absolute():
            h5_path = point_cloud_root / h5_path
        with h5py.File(h5_path, "r") as handle:
            group = handle["/point_cloud"]
            point_count = int(group["frame"].shape[0]) if "frame" in group else 0
            frames = np.asarray(group["frame"][...], dtype=int) if point_count else np.empty((0,), dtype=int)
            ranges = np.asarray(group["range_m"][...], dtype=float) if point_count else np.empty((0,), dtype=float)
            velocities = np.asarray(group["velocity_mps"][...], dtype=float) if point_count else np.empty((0,), dtype=float)
            azimuth = np.asarray(group["azimuth_deg"][...], dtype=float) if point_count else np.empty((0,), dtype=float)
            elevation = np.asarray(group["elevation_deg"][...], dtype=float) if point_count else np.empty((0,), dtype=float)
            total_frames = int(case.get("frames", 0))
        r_stats = _stats(ranges); v_stats = _stats(velocities); a_stats = _stats(azimuth); e_stats = _stats(elevation)
        active_frames = int(len(set(frames.tolist())))
        propagation = propagated.get(case_id)
        rows.append({
            "case_id": case_id, "sea_state": int(case_id[2]) if case_id.startswith("ss") and case_id[2].isdigit() else "",
            "frames": total_frames, "point_count": point_count, "active_detection_frames": active_frames,
            "detection_frame_rate": (active_frames / total_frames) if total_frames else None,
            "mean_points_per_frame": (point_count / total_frames) if total_frames else None,
            "range_mean_m": r_stats[0], "range_std_m": r_stats[1], "range_p05_m": r_stats[2], "range_p50_m": r_stats[3], "range_p95_m": r_stats[4],
            "velocity_mean_mps": v_stats[0], "velocity_std_mps": v_stats[1],
            "azimuth_mean_deg": a_stats[0], "azimuth_std_deg": a_stats[1], "azimuth_p05_deg": a_stats[2], "azimuth_p50_deg": a_stats[3], "azimuth_p95_deg": a_stats[4],
            "elevation_mean_deg": e_stats[0], "elevation_std_deg": e_stats[1], "elevation_p05_deg": e_stats[2], "elevation_p50_deg": e_stats[3], "elevation_p95_deg": e_stats[4],
            "cfar_status": "detected_synthetic_points" if point_count else "no_cfar_detections",
            "latency_propagation_available": bool(propagation),
        })
    output.mkdir(parents=True, exist_ok=True)
    with (output / "all_sea_state_coverage.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    detected = [row for row in rows if row["point_count"] > 0]
    summary = {
        "status": "completed_all_sea_state_coverage_matrix",
        "case_count": len(rows), "detected_case_count": len(detected),
        "zero_detection_case_count": len(rows) - len(detected),
        "cases_with_latency_propagation": sum(bool(row["latency_propagation_available"]) for row in rows),
        "point_cloud_source_is_hardware": False,
        "interpretation": "zero_cfar_detections_is_not_zero_sea_clutter",
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# V0.4.67 全海况点云覆盖矩阵",
        "",
        f"海况/案例数：{len(rows)}；有 CFAR 检测点案例：{len(detected)}；无 CFAR 检测点案例：{len(rows) - len(detected)}。",
        "",
        "| 案例 | 帧数 | 点数 | 有检测点帧率 | CFAR 状态 | 姿态/延迟传播 |",
        "|---|---:|---:|---:|---|---|",
    ]
    lines.extend(f"| {row['case_id']} | {row['frames']} | {row['point_count']} | {row['detection_frame_rate'] if row['detection_frame_rate'] is not None else '—'} | {row['cfar_status']} | {'有' if row['latency_propagation_available'] else '无'} |" for row in rows)
    lines += [
        "",
        "## 关键解释",
        "",
        "`no_cfar_detections` 只表示在当前功率、CFAR 训练/保护单元和 Pfa 设置下没有保留下来的检测点，不能解释为海面没有杂波。需要降低门限、扩大观测帧数或直接分析未 CFAR 的复数回波，才能判断低海况下的海杂波能量。",
        "",
        "V0.4.65 的姿态/延迟传播目前只覆盖有点云的 ss3 案例；零检测案例没有点可供逐点补偿，但已在本矩阵中保留，避免把数据缺口误写成物理结论。",
        "",
        "## 证据边界",
        "",
        "所有 HDF5 都是合成海杂波和非硬件校准结果，不能作为实船虚警率、检测距离或真实 AoA 结论。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--point-cloud-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--propagated-metrics", type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.point_cloud_root.resolve(), args.output.resolve(), args.propagated_metrics.resolve() if args.propagated_metrics else None), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
