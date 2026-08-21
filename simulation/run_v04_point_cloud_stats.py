"""Summarize CFAR point-cloud HDF5 runs for sea-state comparisons."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np


FIELDS = ("range_m", "velocity_mps", "azimuth_deg", "elevation_deg", "power_linear", "x_m", "y_m", "z_m")


def load_points(path: Path) -> dict[str, np.ndarray]:
    with h5py.File(path, "r") as handle:
        return {field: handle[f"/point_cloud/{field}"][...] for field in FIELDS}


def percentile(values: np.ndarray, q: float) -> float | None:
    return None if values.size == 0 else float(np.percentile(values, q))


def summarize(label: str, path: Path) -> dict:
    points = load_points(path)
    count = len(points["range_m"])
    summary = {"label": label, "input_h5": str(path.resolve()), "point_count": count}
    for field in FIELDS:
        values = np.asarray(points[field], dtype=float)
        summary[f"{field}_mean"] = None if values.size == 0 else float(np.mean(values))
        summary[f"{field}_std"] = None if values.size == 0 else float(np.std(values))
        summary[f"{field}_p95_abs"] = percentile(np.abs(values), 95.0)
    if count:
        radial_bins = np.arange(0.0, max(1.0, float(np.max(points["range_m"]))) + 5.0, 5.0)
        summary["range_bin_width_m"] = 5.0
        summary["range_bin_count"] = int(len(radial_bins) - 1)
        summary["max_range_m"] = float(np.max(points["range_m"]))
        summary["max_power_linear"] = float(np.max(points["power_linear"]))
    else:
        summary["range_bin_width_m"] = 5.0; summary["range_bin_count"] = 0
        summary["max_range_m"] = None; summary["max_power_linear"] = None
    return summary


def run(inputs: list[tuple[str, Path]], output: Path) -> list[dict]:
    summaries = [summarize(label, path) for label, path in inputs]
    output.mkdir(parents=True, exist_ok=True)
    with (output / "point_cloud_stats.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0]) if summaries else ["label"])
        writer.writeheader(); writer.writerows(summaries)
    (output / "summary.json").write_text(json.dumps({"runs": summaries}, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.24 海况点云统计", "", "| 标签 | 点数 | 距离均值(m) | 速度均值(m/s) | 方位标准差(°) | 俯仰标准差(°) | 最大功率 |", "|---|---:|---:|---:|---:|---:|---:|"]
    for item in summaries:
        fmt = lambda key: "-" if item.get(key) is None else f"{item[key]:.4f}"
        lines.append(f"| {item['label']} | {item['point_count']} | {fmt('range_m_mean')} | {fmt('velocity_mps_mean')} | {fmt('azimuth_deg_std')} | {fmt('elevation_deg_std')} | {fmt('max_power_linear')} |")
    lines += ["", "点数、速度展宽和角度标准差可用于比较海况；必须保持 CFG、CFAR 参数、采集时长和安装姿态一致。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summaries


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", action="append", nargs=2, metavar=("LABEL", "H5"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    summaries = run([(label, Path(path).resolve()) for label, path in args.input], args.output.resolve())
    print(json.dumps(summaries, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
