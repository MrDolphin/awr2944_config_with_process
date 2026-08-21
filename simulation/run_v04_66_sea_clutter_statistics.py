"""Aggregate delay-propagated point-cloud changes by sea case and delay."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def _read(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _stats(values: np.ndarray) -> tuple[float, float, float, float, float]:
    return tuple(float(item) for item in (np.mean(values), np.std(values), np.percentile(values, 5), np.percentile(values, 50), np.percentile(values, 95)))


def run(input_run: Path, output: Path) -> dict:
    metric_rows = _read(input_run / "latency_pointcloud_metrics.csv")
    delay_dirs = [path for path in input_run.iterdir() if path.is_dir() and path.name.startswith("delay_")]
    detail_rows: list[dict] = []
    delay_rows: list[dict] = []
    for metric in metric_rows:
        if str(metric["quality_pass"]).lower() != "true":
            continue
        delay = float(metric["latency_correction_s"])
        name = f"delay_{delay:+.6f}s".replace("+", "p").replace("-", "m").replace(".", "_")
        comparison = next((path / "compensated" / "point_cloud_pose_comparison.csv" for path in delay_dirs if path.name == name), None)
        if comparison is None or not comparison.exists():
            continue
        rows = _read(comparison)
        cases = sorted({row.get("case_id", "unknown") for row in rows})
        abs_az = np.asarray([abs(float(row["azimuth_change_deg"])) for row in rows])
        abs_el = np.asarray([abs(float(row["elevation_change_deg"])) for row in rows])
        pos = np.asarray([float(row["position_change_m"]) for row in rows])
        delay_rows.append({
            "latency_correction_s": delay, "point_count": len(rows), "case_count": len(cases),
            "mean_abs_azimuth_change_deg": float(np.mean(abs_az)),
            "mean_abs_elevation_change_deg": float(np.mean(abs_el)),
            "mean_position_change_m": float(np.mean(pos)),
            "max_position_change_m": float(np.max(pos)),
        })
        for case in cases:
            case_rows = [row for row in rows if row.get("case_id", "unknown") == case]
            fixed_az = np.asarray([float(row["fixed_azimuth_deg"]) for row in case_rows])
            fixed_el = np.asarray([float(row["fixed_elevation_deg"]) for row in case_rows])
            comp_az = np.asarray([float(row["compensated_azimuth_deg"]) for row in case_rows])
            comp_el = np.asarray([float(row["compensated_elevation_deg"]) for row in case_rows])
            az_change = np.asarray([float(row["azimuth_change_deg"]) for row in case_rows])
            el_change = np.asarray([float(row["elevation_change_deg"]) for row in case_rows])
            position = np.asarray([float(row["position_change_m"]) for row in case_rows])
            f_az = _stats(fixed_az); f_el = _stats(fixed_el); c_az = _stats(comp_az); c_el = _stats(comp_el); a_change = _stats(az_change); e_change = _stats(el_change)
            detail_rows.append({
                "case_id": case, "latency_correction_s": delay, "point_count": len(case_rows),
                "fixed_az_mean_deg": f_az[0], "fixed_az_std_deg": f_az[1], "fixed_az_p05_deg": f_az[2], "fixed_az_p50_deg": f_az[3], "fixed_az_p95_deg": f_az[4],
                "fixed_el_mean_deg": f_el[0], "fixed_el_std_deg": f_el[1], "fixed_el_p05_deg": f_el[2], "fixed_el_p50_deg": f_el[3], "fixed_el_p95_deg": f_el[4],
                "comp_az_mean_deg": c_az[0], "comp_az_std_deg": c_az[1], "comp_az_p05_deg": c_az[2], "comp_az_p50_deg": c_az[3], "comp_az_p95_deg": c_az[4],
                "comp_el_mean_deg": c_el[0], "comp_el_std_deg": c_el[1], "comp_el_p05_deg": c_el[2], "comp_el_p50_deg": c_el[3], "comp_el_p95_deg": c_el[4],
                "az_change_mean_deg": a_change[0], "az_change_std_deg": a_change[1], "az_change_p05_deg": a_change[2], "az_change_p50_deg": a_change[3], "az_change_p95_deg": a_change[4],
                "el_change_mean_deg": e_change[0], "el_change_std_deg": e_change[1], "el_change_p05_deg": e_change[2], "el_change_p50_deg": e_change[3], "el_change_p95_deg": e_change[4],
                "mean_position_change_m": float(np.mean(position)), "max_position_change_m": float(np.max(position)),
            })
    if not detail_rows:
        raise ValueError("no passing compensated point-cloud results found")
    output.mkdir(parents=True, exist_ok=True)
    with (output / "sea_state_delay_statistics.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(detail_rows[0])); writer.writeheader(); writer.writerows(detail_rows)
    with (output / "delay_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(delay_rows[0])); writer.writeheader(); writer.writerows(delay_rows)
    best = min(delay_rows, key=lambda row: row["mean_abs_azimuth_change_deg"] + row["mean_abs_elevation_change_deg"])
    worst = max(delay_rows, key=lambda row: row["max_position_change_m"])
    summary = {
        "status": "completed_sea_clutter_delay_statistics",
        "case_count": len({row["case_id"] for row in detail_rows}), "delay_count": len(delay_rows),
        "best_delay_by_mean_angular_change_s": best["latency_correction_s"],
        "worst_delay_by_max_position_change_s": worst["latency_correction_s"],
        "best_delay_mean_angular_change_deg": best["mean_abs_azimuth_change_deg"] + best["mean_abs_elevation_change_deg"],
        "worst_delay_max_position_change_m": worst["max_position_change_m"],
        "source_is_hardware": False,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# V0.4.66 海杂波点云统计与延迟敏感性",
        "",
        f"海况/案例数：{summary['case_count']}；延迟数：{summary['delay_count']}。",
        f"按平均方位+俯仰变化的最小延迟候选：{best['latency_correction_s']:+.6f} s；按最大坐标变化的最差延迟候选：{worst['latency_correction_s']:+.6f} s。",
        "",
        "## 输出字段",
        "",
        "`sea_state_delay_statistics.csv` 按 case_id 和延迟给出固定姿态/补偿姿态的均值、标准差、P05/P50/P95，以及方位/俯仰变化和坐标变化。`delay_summary.csv` 用于跨海况快速比较。",
        "",
        "## 如何解读",
        "",
        "先按 case_id 比较不同海况，再沿延迟维度比较同一海况。均值表示总体偏移，标准差表示点云离散程度，P05/P50/P95 用于观察异常点和长尾。当前点数很少时，统计量只用于验证数据链路，不能当作稳健海况分布。",
        "",
        "## 证据边界",
        "",
        "当前仍是合成/历史点云和合成 IMU，不能报告实船虚警率、检测距离或真实 AoA 误差；最优/最差延迟只是当前轨迹和点云条件下的候选。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.input_run.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
