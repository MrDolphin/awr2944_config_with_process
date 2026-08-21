"""Apply CA-CFAR and sparse-array AoA to V0.4.43 sea range-Doppler HDF5."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_39_sparse_array_solver import estimate_sparse_grid
from simulation.run_v04_cfar_point_cloud import ca_cfar
from simulation.v03 import FmcwConfig


def process_case(input_h5: Path, output_h5: Path, positions: tuple[np.ndarray, np.ndarray], config: FmcwConfig, pfa: float, max_detections_per_frame: int) -> tuple[dict, list[dict]]:
    with h5py.File(input_h5, "r") as handle:
        power = handle["/range_doppler/power_linear"][...]
        spectrum = handle["/range_doppler/spectrum_complex"][...]
        ranges = handle["/axes/range_m"][...]
        velocities = handle["/axes/velocity_mps"][...]
    all_points: list[dict] = []
    for frame in range(power.shape[0]):
        candidates, alpha = ca_cfar(power[frame], training=(4, 4), guard=(1, 1), pfa=pfa)
        candidates = sorted(candidates, key=lambda item: item[2], reverse=True)[:max_detections_per_frame]
        for d_idx, r_idx, peak, noise, threshold in candidates:
            channel = spectrum[frame, d_idx, r_idx]
            az, el, score = estimate_sparse_grid(config, channel, positions[0], positions[1], np.arange(-60.0, 60.01, 1.0), np.arange(-20.0, 20.01, 1.0))
            r = float(ranges[r_idx]); v = float(velocities[d_idx]); az_rad = np.deg2rad(az); el_rad = np.deg2rad(el)
            all_points.append({"frame": frame, "doppler_index": d_idx, "range_index": r_idx, "range_m": r, "velocity_mps": v, "azimuth_deg": az, "elevation_deg": el, "power_linear": peak, "noise_linear": noise, "threshold_linear": threshold, "aoa_score": score, "x_m": r * np.cos(el_rad) * np.sin(az_rad), "y_m": r * np.cos(el_rad) * np.cos(az_rad), "z_m": r * np.sin(el_rad)})
    output_h5.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_h5, "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-synthetic-sea-cfar-point-cloud-v0.4.44"; handle.attrs["input_h5"] = str(input_h5.resolve()); handle.attrs["channel_order_verified"] = False; handle.attrs["aoa_status"] = "sparse_grid_synthetic_not_hardware_validated"; handle.attrs["pfa"] = pfa; handle.attrs["cfar_alpha"] = alpha
        keys = list(all_points[0]) if all_points else ["empty"]
        for key in keys:
            handle.create_dataset(f"/point_cloud/{key}", data=np.asarray([point[key] for point in all_points]) if all_points else np.empty((0,)))
    case_id = input_h5.stem.replace("_range_doppler", "")
    summary = {"case_id": case_id, "input_h5": str(input_h5.resolve()), "output_h5": str(output_h5.resolve()), "frames": int(power.shape[0]), "point_count": len(all_points), "mean_points_per_frame": len(all_points) / max(power.shape[0], 1), "mean_range_m": float(np.mean([p["range_m"] for p in all_points])) if all_points else None, "mean_velocity_mps": float(np.mean([p["velocity_mps"] for p in all_points])) if all_points else None, "mean_aoa_score": float(np.mean([p["aoa_score"] for p in all_points])) if all_points else None, "pfa": pfa, "channel_order_verified": False}
    return summary, all_points


def run(input_root: Path, output: Path) -> dict:
    config = FmcwConfig(samples_per_chirp=128, chirps_per_frame=64); models = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config); paths = sorted(input_root.glob("*_range_doppler.h5")); summaries = []; points: list[dict] = []
    for path in paths:
        summary, case_points = process_case(path, output / f"{path.stem}_point_cloud.h5", models["pcb_centroid_candidate"], config, pfa=1e-3, max_detections_per_frame=32); summaries.append(summary); points.extend({"case_id": summary["case_id"], **point} for point in case_points)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "point_cloud.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = list(points[0]) if points else ["case_id", "frame"]; writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(points)
    with (output / "case_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    summary = {"status": "completed_synthetic_sea_cfar_point_cloud", "input_root": str(input_root.resolve()), "output_root": str(output.resolve()), "case_count": len(summaries), "pfa": 1e-3, "max_detections_per_frame": 32, "point_count": len(points), "point_cloud_status": "synthetic_sea_clutter_not_hardware_validated", "channel_order_verified": False, "cases": summaries}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.44 合成海杂波 CA-CFAR 三维点云", "", "本阶段读取 V0.4.43 的距离-多普勒功率谱和 4×4 复数通道谱，对每个海况、每个时间帧执行二维 CA-CFAR，并对每个保留检测点执行稀疏阵列 AoA。", "", "| 海况 | 检测点数 | 平均点/帧 | 平均距离(m) | 平均速度(m/s) | 平均 AoA 相关性 |", "|---|---:|---:|---:|---:|---:|"]
    lines.extend(f"| {row['case_id']} | {row['point_count']} | {row['mean_points_per_frame']:.2f} | {row['mean_range_m'] if row['mean_range_m'] is not None else float('nan'):.4f} | {row['mean_velocity_mps'] if row['mean_velocity_mps'] is not None else float('nan'):.6f} | {row['mean_aoa_score'] if row['mean_aoa_score'] is not None else float('nan'):.4f} |" for row in summaries)
    lines += ["", "## 点云坐标", "", "使用 x=R·cos(el)·sin(az)、y=R·cos(el)·cos(az)、z=R·sin(el)。每个 HDF5 的 /point_cloud 包含 frame、range、velocity、azimuth、elevation、power、noise、threshold、aoa_score 和 x/y/z。", "", "## 边界", "", "CFAR 输入和复数谱来自 V0.4.43 合成海杂波，不是 DCA1000 实测 IQ；PCB 阵列仍是铜区质心候选；CFAR 和 AoA 参数不是 TI SDK 默认实现。因此点云用于验证数据结构、指标和海况差异，不能作为实板虚警率或检测距离结论。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
