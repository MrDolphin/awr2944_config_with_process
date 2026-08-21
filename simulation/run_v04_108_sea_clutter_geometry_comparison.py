"""Compare ideal and provisional PCB geometry in the synthetic sea clutter loop."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_39_sparse_array_solver import estimate_sparse_grid
from simulation.run_v04_41_sea_clutter_aoa import clutter_channel, load_surface, surface_snapshot
from simulation.run_v04_107_provisional_array_comparison import load_provisional
from simulation.v03 import FmcwConfig
from simulation.v04 import virtual_array_positions


def run(input_root: Path, provisional_csv: Path, output: Path) -> dict:
    config = FmcwConfig()
    ideal = virtual_array_positions(config)
    provisional = load_provisional(provisional_csv, config)
    models = {"ideal_half_lambda": ideal, "provisional_pcb_endpoint": provisional}
    solver_az = np.arange(-60.0, 60.01, 1.0)
    solver_el = np.arange(-20.0, 20.01, 1.0)
    case_paths = sorted(input_root.glob("ss*_seed101.h5"))
    if not case_paths:
        raise FileNotFoundError(f"No ss*_seed101.h5 files under {input_root}")
    rows = []
    for case_path in case_paths:
        surface = load_surface(case_path)
        for frame, time_s in enumerate(surface["time_s"]):
            facets, _xyz, meta = surface_snapshot(surface, frame, np.random.default_rng(4141 + frame))
            dominant = facets[meta["dominant_index"]]
            for model_name, positions in models.items():
                clean = clutter_channel(config, facets, positions)
                # Same deterministic impairment seed for both geometries so the
                # comparison isolates geometry rather than random noise.
                rng = np.random.default_rng(9000 + frame)
                from simulation.run_v04_40_calibration_noise_peaks import apply_impairments
                observed = apply_impairments(clean, rng, gain_std_db=1.0, phase_std_deg=5.0, snr_db=20.0)
                estimate = estimate_sparse_grid(config, observed, positions[0], positions[1], solver_az, solver_el)
                rows.append({"case_id": surface["case_id"], "frame": frame, "time_s": float(time_s), "model": model_name, "facet_count": meta["facet_count"], "dominant_azimuth_deg": float(dominant[0]), "dominant_elevation_deg": float(dominant[1]), "estimated_azimuth_deg": estimate[0], "estimated_elevation_deg": estimate[1], "azimuth_error_to_dominant_deg": estimate[0] - dominant[0], "elevation_error_to_dominant_deg": estimate[1] - dominant[1], "score": estimate[2]})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "sea_clutter_geometry_frames.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    differences = []
    for case_id in sorted({row["case_id"] for row in rows}):
        ideal_rows = {(r["frame"]): r for r in rows if r["case_id"] == case_id and r["model"] == "ideal_half_lambda"}
        pcb_rows = {(r["frame"]): r for r in rows if r["case_id"] == case_id and r["model"] == "provisional_pcb_endpoint"}
        for frame in sorted(ideal_rows):
            a, b = ideal_rows[frame], pcb_rows[frame]
            differences.append({"case_id": case_id, "frame": frame, "azimuth_difference_deg": b["estimated_azimuth_deg"] - a["estimated_azimuth_deg"], "elevation_difference_deg": b["estimated_elevation_deg"] - a["estimated_elevation_deg"], "ideal_score": a["score"], "provisional_score": b["score"]})
    with (output / "sea_clutter_geometry_differences.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(differences[0])); writer.writeheader(); writer.writerows(differences)
    summaries = []
    for case_id in sorted({row["case_id"] for row in rows}):
        subset = [r for r in rows if r["case_id"] == case_id]
        summary = {"case_id": case_id, "frames": len(subset) // 2}
        for model_name in models:
            part = [r for r in subset if r["model"] == model_name]
            az = np.asarray([r["azimuth_error_to_dominant_deg"] for r in part]); el = np.asarray([r["elevation_error_to_dominant_deg"] for r in part])
            summary[f"{model_name}_azimuth_rmse_deg"] = float(np.sqrt(np.mean(az ** 2)))
            summary[f"{model_name}_elevation_rmse_deg"] = float(np.sqrt(np.mean(el ** 2)))
        diff = [r for r in differences if r["case_id"] == case_id]
        summary["model_azimuth_difference_rmse_deg"] = float(np.sqrt(np.mean(np.asarray([r["azimuth_difference_deg"] for r in diff]) ** 2)))
        summary["model_elevation_difference_rmse_deg"] = float(np.sqrt(np.mean(np.asarray([r["elevation_difference_deg"] for r in diff]) ** 2)))
        summary["model_difference_over_10deg_rate"] = float(np.mean(np.abs([r["azimuth_difference_deg"] for r in diff]) > 10.0))
        summaries.append(summary)
    with (output / "sea_clutter_geometry_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, dpi=160)
        first = sorted({r["case_id"] for r in rows})[0]
        for model_name in models:
            part = [r for r in rows if r["case_id"] == first and r["model"] == model_name]
            axes[0].plot([r["frame"] for r in part], [r["estimated_azimuth_deg"] for r in part], label=model_name)
            axes[1].plot([r["frame"] for r in part], [r["estimated_elevation_deg"] for r in part], label=model_name)
        axes[0].set_ylabel("Estimated azimuth (deg)"); axes[1].set_ylabel("Estimated elevation (deg)"); axes[1].set_xlabel("Frame")
        axes[0].set_title(f"Synthetic sea clutter geometry comparison: {first}")
        for ax in axes: ax.grid(True, alpha=0.25); ax.legend()
        fig.tight_layout(); fig.savefig(output / "sea_clutter_geometry_comparison.png"); plt.close(fig)
    except Exception:
        pass
    summary = {"status": "completed_synthetic_sea_clutter_geometry_comparison", "input_root": str(input_root.resolve()), "provisional_geometry": str(provisional_csv.resolve()), "case_count": len(case_paths), "random_seeds": {"surface": 4141, "impairment_base": 9000}, "models": list(models), "input_status": "v02_surface_height_truth_not_measured_radar_iq", "provisional_geometry_status": "geometric_endpoint_candidate_not_phase_center", "hardware_aoa_validated": False, "cases": summaries}
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.108 海杂波中的阵列几何比较", "", "使用同一 V0.2 海面高度场、同一帧抽样和同一随机误差种子，分别用理想半波长阵列与 V0.4.106 临时 PCB 端点几何生成合成海杂波通道，并进行稀疏导向矢量 AoA。", "", "| 海况 | 帧数 | 理想方位 RMSE | PCB候选方位 RMSE | 模型间方位差 RMSE | 方位差>10°比例 |", "|---|---:|---:|---:|---:|---:|"]
    lines.extend(f"| {r['case_id']} | {r['frames']} | {r['ideal_half_lambda_azimuth_rmse_deg']:.3f}° | {r['provisional_pcb_endpoint_azimuth_rmse_deg']:.3f}° | {r['model_azimuth_difference_rmse_deg']:.3f}° | {r['model_difference_over_10deg_rate']:.3f} |" for r in summaries)
    lines += ["", "## 指标如何理解", "", "这里的 RMSE 是估计主峰相对每帧最强合成海面微元角度的偏差；模型间差异是同一海面帧、同一噪声种子下两套阵列几何造成的角度变化。海杂波本身是多微元叠加，最强微元不是固定目标真值。", "", "## 证据边界", "", "输入只有动态海面高度场，反射强度、相位、可见性、通道误差均为合成假设；PCB 坐标是 RF 走线端点候选而非相位中心。结果用于说明阵列几何对海杂波 AoA 的敏感性，不能作为实船杂波角度、AoA 精度或探测性能结论。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--provisional", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.provisional.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
