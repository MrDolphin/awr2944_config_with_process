"""Create leadership-ready 3D views of V0.130 sea-clutter AoA results."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def _plot_case(rows: list[dict[str, str]], case_id: str, model: str, output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    part = [row for row in rows if row["case_id"] == case_id and row["model"] == model]
    if not part:
        return
    x = np.asarray([float(row["x_m"]) for row in part]); y = np.asarray([float(row["y_m"]) for row in part]); z = np.asarray([float(row["z_m"]) for row in part])
    az = np.asarray([float(row["estimated_azimuth_deg"]) for row in part]); el = np.asarray([float(row["estimated_elevation_down_deg"]) for row in part]); weight = np.asarray([float(row["pattern_two_way_weight"]) for row in part]); scatter = np.asarray([float(row["aoa_weighted_scatter_two_way"]) for row in part])
    figure = plt.figure(figsize=(13, 5), constrained_layout=True)
    left = figure.add_subplot(121, projection="3d"); points = left.scatter(x, y, z, c=az, s=8, cmap="twilight", alpha=.8); left.set_title(f"{case_id} / {model}\nEstimated azimuth (deg)"); left.set_xlabel("x (m)"); left.set_ylabel("y (m)"); left.set_zlabel("z (m)"); figure.colorbar(points, ax=left, shrink=.65, label="estimated azimuth (deg)")
    right = figure.add_subplot(122, projection="3d"); nonzero = scatter > 0; points2 = right.scatter(x[nonzero], y[nonzero], z[nonzero], c=np.log10(np.maximum(weight[nonzero], 1e-12)), s=12, cmap="viridis", alpha=.85); right.set_title("Effective two-way pattern weight\nlog10(weight)"); right.set_xlabel("x (m)"); right.set_ylabel("y (m)"); right.set_zlabel("z (m)"); figure.colorbar(points2, ax=right, shrink=.65, label="log10(two-way weight)")
    figure.savefig(output / f"{case_id}_{model}_3d_aoa_clutter.png", dpi=150); plt.close(figure)


def run(aoa_csv: Path, geometry_csv: Path, output: Path) -> dict:
    with aoa_csv.open(encoding="utf-8", newline="") as handle:
        aoa_rows = list(csv.DictReader(handle))
    with geometry_csv.open(encoding="utf-8", newline="") as handle:
        geometry = {(row["case_id"], row["frame"], row["facet_index"]): row for row in csv.DictReader(handle)}
    rows = []
    for row in aoa_rows:
        source = geometry[(row["case_id"], row["frame"], row["facet_index"])]
        rows.append({**row, "x_m": source["x_m"], "y_m": source["y_m"], "z_m": source["z_m"], "slant_range_m": source["slant_range_m"]})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "visualization_points.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summaries = []
    for case_id in sorted({row["case_id"] for row in rows}):
        for model in sorted({row["model"] for row in rows}):
            part = [row for row in rows if row["case_id"] == case_id and row["model"] == model]; summaries.append({"case_id": case_id, "model": model, "point_count": len(part), "nonzero_weight_points": sum(float(row["pattern_two_way_weight"]) > 0 for row in part), "two_way_scatter_sum": sum(float(row["aoa_weighted_scatter_two_way"]) for row in part), "mean_estimated_azimuth_deg": float(np.mean([float(row["estimated_azimuth_deg"]) for row in part])), "mean_estimated_elevation_deg": float(np.mean([float(row["estimated_elevation_down_deg"]) for row in part]))})
    with (output / "visualization_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    for model in sorted({row["model"] for row in rows}):
        _plot_case(rows, "ss3_upper", model, output)
    result = {"status": "completed_sea_clutter_3d_visualization", "source_rows": len(rows), "case_count": len({row["case_id"] for row in rows}), "model_count": len({row["model"] for row in rows}), "figures_case": "ss3_upper", "point_semantics": "sea_surface_facets_with_synthetic_grid_aoa_and_pattern_proxy", "hardware_aoa_validated": False}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.131 海杂波三维点云与 AoA 可视化", "", f"本阶段将 V0.130 的 {len(rows)} 条有效海面微面元结果与 V0.120 的 x/y/z 海面坐标连接，生成按估计方位角着色的三维几何点云，以及按双程方向图权重着色的有效散射点。", "", "## 图片如何看", "", "左图：每个点是海面微面元，颜色表示合成 IQ 经网格波束扫描得到的估计方位角，不表示真实雷达检测强度。右图：只显示双程方向图加权后仍有散射代理的点，颜色表示 `log10(two-way weight)`；颜色越亮代表当前粗略方向图模型给予的相对权重越大。", "", "## 海况比较", "", "`visualization_summary.csv` 按海况和阵列模型汇总点数、有效权重点数、双程加权散射和、平均估计方位/俯仰。ss3_upper 图片用于代表性汇报，其他海况仍可从 CSV 筛选复现。", "", "## 证据边界", "", "这些是海面几何微面元叠加合成 AoA 与粗略方向图代理的科研可视化，不是未经 CFAR 的真实 ADC 点云，也不是检测点云。真实结论仍需 DCA1000 IQ、TI 校准、真实方向图、目标检测链路和海试数据。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--aoa", type=Path, required=True); parser.add_argument("--geometry", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.aoa.resolve(), args.geometry.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
