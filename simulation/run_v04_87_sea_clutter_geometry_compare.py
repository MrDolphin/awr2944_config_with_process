"""Compare ideal and PCB-candidate geometry in the dynamic sea AoA stress test."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_41_sea_clutter_aoa import run_case
from simulation.run_v04_86_geometry_comparison import _load_candidate
from simulation.v03 import FmcwConfig
from simulation.v04 import virtual_array_positions


def run(input_root: Path, candidate_csv: Path, output: Path) -> dict:
    config = FmcwConfig()
    models = {"ideal_v04": virtual_array_positions(config), "pcb_candidate": _load_candidate(candidate_csv)}
    paths = sorted(input_root.glob("ss*_seed101.h5"))
    if not paths:
        raise FileNotFoundError(f"no ss*_seed101.h5 under {input_root}")
    solver_az = np.arange(-60.0, 60.01, 1.0)
    solver_el = np.arange(-20.0, 20.01, 1.0)
    rows = []
    for case_index, path in enumerate(paths):
        for model_index, (model_name, positions) in enumerate(models.items()):
            # Same seed per case/model makes the facet sampling and impairments comparable.
            rows.extend(run_case(path, model_name, positions, config, solver_az, solver_el, np.random.default_rng(8700 + case_index)))
    output.mkdir(parents=True, exist_ok=True)
    with (output / "sea_clutter_geometry_frames.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summaries = []
    for case_id in sorted({row["case_id"] for row in rows}):
        for model in models:
            subset = [row for row in rows if row["case_id"] == case_id and row["model"] == model]
            az = np.asarray([row["azimuth_error_to_dominant_deg"] for row in subset], dtype=float)
            el = np.asarray([row["elevation_error_to_dominant_deg"] for row in subset], dtype=float)
            jumps = np.asarray([row["angle_jump_deg"] for row in subset[1:]], dtype=float)
            summaries.append({"case_id": case_id, "model": model, "frames": len(subset), "azimuth_rmse_to_dominant_deg": float(np.sqrt(np.mean(az ** 2))), "elevation_rmse_to_dominant_deg": float(np.sqrt(np.mean(el ** 2))), "combined_rmse_to_dominant_deg": float(np.sqrt(np.mean(np.concatenate((az, el)) ** 2))), "mean_peak_to_second_db": float(np.mean([row["peak_to_second_db"] for row in subset])), "jump_rate_over_5deg": float(np.mean(jumps > 5.0)) if len(jumps) else 0.0, "mean_angle_jump_deg": float(np.mean(jumps)) if len(jumps) else 0.0})
    with (output / "sea_clutter_geometry_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    summary = {"status": "completed_sea_clutter_geometry_comparison", "input_root": str(input_root.resolve()), "candidate_csv": str(candidate_csv.resolve()), "case_count": len(paths), "models": list(models), "random_seed_base": 8700, "input_status": "v02_surface_height_truth_not_measured_radar_iq", "hardware_aoa_validated": False, "summaries": summaries}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 4), constrained_layout=True)
    for model in models:
        subset = [row for row in summaries if row["model"] == model]
        labels = [row["case_id"] for row in subset]
        axes[0].plot(labels, [row["combined_rmse_to_dominant_deg"] for row in subset], marker="o", label=model)
        axes[1].plot(labels, [row["jump_rate_over_5deg"] for row in subset], marker="o", label=model)
    axes[0].set_title("Sea-clutter AoA error to dominant facet"); axes[0].set_ylabel("combined RMSE (deg)"); axes[0].grid(alpha=.25)
    axes[1].set_title("Angle jump rate"); axes[1].set_ylabel("fraction of jumps > 5 deg"); axes[1].grid(alpha=.25)
    for ax in axes: ax.set_xlabel("sea state"); ax.legend(fontsize=8)
    fig.savefig(output / "sea_clutter_geometry_comparison.png", dpi=160); plt.close(fig)

    lines = ["# V0.4.87 海杂波 AoA 几何对比", "", "本阶段在同一组 V0.2 动态海面真值、相同海况和相同随机种子下，比较 V0.4 理想虚拟阵列与 V0.4.85 PCB 候选虚拟阵列。", "", "## 图形如何阅读", "", "- 横轴是海况；左图是 AoA 主峰相对当前最强海面微元的综合 RMSE。海杂波不是单一目标，所以它不是固定目标真值误差。", "- 右图是相邻时间帧估计角跳变超过 5° 的比例，用于观察海杂波下角度稳定性。", "", "## 结论边界", "", "这是表面高度真值驱动的合成微元和合成复数通道压力测试，不是 DCA1000 实测 IQ；PCB 坐标是 Region 几何中心候选，不是电气相位中心。理想/候选阵列的差异可以作为模型敏感性证据，但不能直接报告为 AWR2944P 实测探测率、虚警率或 AoA 精度。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--candidate-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.input_root.resolve(), args.candidate_csv.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
