"""Feed V0.2 dynamic sea-surface truth into the sparse-array AoA solver.

This is a synthetic sea-microfacet stress test.  V0.2 contains surface height
truth, not measured radar IQ, so the output is explicitly not a sea-clutter
hardware validation result.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_39_sparse_array_solver import steering_matrix
from simulation.run_v04_40_calibration_noise_peaks import apply_impairments, estimate_with_peaks
from simulation.v03 import FmcwConfig


def load_surface(path: Path) -> dict:
    with h5py.File(path, "r") as handle:
        return {"time_s": handle["/axes/time_s"][...], "x_m": handle["/axes/x_m"][...], "y_m": handle["/axes/y_m"][...], "height_m": handle["/truth/height_m"][...], "height_m_install": float(np.asarray(handle["/installation/height_m"][...]).reshape(-1)[0]), "case_id": str(np.asarray(handle.attrs["case_id"]).reshape(-1)[0])}


def surface_snapshot(surface: dict, frame: int, rng: np.random.Generator, max_facets: int = 32) -> tuple[np.ndarray, np.ndarray, dict]:
    height = surface["height_m"][frame]
    yy, xx = np.meshgrid(surface["y_m"], surface["x_m"], indexing="ij")
    # Keep a regular but sparse set of sea facets so every case has a reproducible load.
    stride_y = max(1, len(surface["y_m"]) // 12)
    stride_x = max(1, len(surface["x_m"]) // 12)
    x = xx[::stride_y, ::stride_x].ravel()
    y = yy[::stride_y, ::stride_x].ravel()
    z = height[::stride_y, ::stride_x].ravel() - surface["height_m_install"]
    if len(x) > max_facets:
        indices = rng.choice(len(x), size=max_facets, replace=False)
        x, y, z = x[indices], y[indices], z[indices]
    horizontal = np.sqrt(x ** 2 + y ** 2)
    az = np.rad2deg(np.arctan2(x, y))
    el = np.rad2deg(np.arctan2(z, np.maximum(horizontal, 1e-6)))
    power = (1.0 + np.abs(z)) * (0.5 + rng.random(len(x))) / np.maximum(horizontal, 1.0) ** 2
    dominant = int(np.argmax(power))
    return np.column_stack((az, el, power)), np.column_stack((x, y, z)), {"facet_count": int(len(x)), "dominant_index": dominant}


def clutter_channel(config: FmcwConfig, facets: np.ndarray, positions: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    x_pos, y_pos = positions
    channel = np.zeros((4, 4), dtype=complex)
    for az, el, power in facets:
        steering = np.exp(1j * (2.0 * np.pi / config.wavelength_m) * (x_pos * np.sin(np.deg2rad(az)) * np.cos(np.deg2rad(el)) + y_pos * np.sin(np.deg2rad(el))))
        channel += np.sqrt(max(float(power), 0.0)) * steering
    return channel


def run_case(path: Path, model_name: str, positions: tuple[np.ndarray, np.ndarray], config: FmcwConfig, solver_az: np.ndarray, solver_el: np.ndarray, rng: np.random.Generator, snr_db: float = 20.0) -> list[dict]:
    surface = load_surface(path)
    rows: list[dict] = []
    previous = None
    for frame, time_s in enumerate(surface["time_s"]):
        facets, xyz, meta = surface_snapshot(surface, frame, rng)
        clean = clutter_channel(config, facets, positions)
        observed = apply_impairments(clean, rng, gain_std_db=1.0, phase_std_deg=5.0, snr_db=snr_db)
        estimate = estimate_with_peaks(config, observed, positions[0], positions[1], solver_az, solver_el)
        dominant = meta["dominant_index"]
        dominant_az, dominant_el = facets[dominant, 0], facets[dominant, 1]
        jump = 0.0 if previous is None else float(np.hypot(estimate["estimated_azimuth_deg"] - previous[0], estimate["estimated_elevation_deg"] - previous[1]))
        previous = (estimate["estimated_azimuth_deg"], estimate["estimated_elevation_deg"])
        rows.append({"case_id": surface["case_id"], "source_h5": str(path.resolve()), "sea_state": int(surface["case_id"].replace("ss", "").split("_")[0]), "model": model_name, "frame": frame, "time_s": float(time_s), "facet_count": meta["facet_count"], "dominant_azimuth_deg": float(dominant_az), "dominant_elevation_deg": float(dominant_el), "estimated_azimuth_deg": estimate["estimated_azimuth_deg"], "estimated_elevation_deg": estimate["estimated_elevation_deg"], "azimuth_error_to_dominant_deg": estimate["estimated_azimuth_deg"] - dominant_az, "elevation_error_to_dominant_deg": estimate["estimated_elevation_deg"] - dominant_el, "score": estimate["score"], "peak_to_second_db": estimate["peak_to_second_db"], "angle_jump_deg": jump})
    return rows


def run(input_root: Path, output: Path) -> dict:
    config = FmcwConfig()
    models = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config)
    solver_az = np.arange(-60.0, 60.01, 1.0); solver_el = np.arange(-20.0, 20.01, 1.0)
    case_paths = sorted(input_root.glob("ss*_seed101.h5"))
    if not case_paths:
        raise FileNotFoundError(f"No ss*_seed101.h5 files under {input_root}")
    rng = np.random.default_rng(4141)
    rows: list[dict] = []
    for path in case_paths:
        rows.extend(run_case(path, "pcb_centroid_candidate", models["pcb_centroid_candidate"], config, solver_az, solver_el, rng))
    output.mkdir(parents=True, exist_ok=True)
    with (output / "sea_clutter_aoa_frames.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summaries = []
    for case_id in sorted({row["case_id"] for row in rows}):
        subset = [row for row in rows if row["case_id"] == case_id]
        az = np.asarray([row["azimuth_error_to_dominant_deg"] for row in subset]); el = np.asarray([row["elevation_error_to_dominant_deg"] for row in subset]); jumps = np.asarray([row["angle_jump_deg"] for row in subset[1:]])
        summaries.append({"case_id": case_id, "frames": len(subset), "azimuth_rmse_to_dominant_deg": float(np.sqrt(np.mean(az ** 2))), "elevation_rmse_to_dominant_deg": float(np.sqrt(np.mean(el ** 2))), "combined_rmse_to_dominant_deg": float(np.sqrt(np.mean(np.concatenate((az, el)) ** 2))), "mean_peak_to_second_db": float(np.mean([row["peak_to_second_db"] for row in subset])), "minimum_peak_to_second_db": float(np.min([row["peak_to_second_db"] for row in subset])), "jump_rate_over_5deg": float(np.mean(jumps > 5.0)) if len(jumps) else 0.0, "mean_angle_jump_deg": float(np.mean(jumps)) if len(jumps) else 0.0})
    with (output / "sea_clutter_aoa_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    summary = {"status": "completed_synthetic_sea_clutter_aoa", "input_root": str(input_root.resolve()), "case_count": len(case_paths), "random_seed": 4141, "solver": "sparse_steering_vector_grid", "calibration": {"gain_std_db": 1.0, "phase_std_deg": 5.0, "snr_db": 20.0}, "input_status": "v02_surface_height_truth_not_measured_radar_iq", "pcb_model_status": "cad_copper_centroid_candidate_not_phase_center", "hardware_aoa_validated": False, "cases": summaries}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.41 合成海杂波微元 AoA 压力测试", "", "本阶段读取 V0.2 动态海面高度真值，将海面网格抽样为合成微元，用 PCB 候选阵列生成复数通道叠加，再加入 1 dB/5° 通道误差和 SNR=20 dB 噪声，使用稀疏阵列导向矢量搜索。", "", "| 海况 | 帧数 | 方位 RMSE(°) | 俯仰 RMSE(°) | 综合 RMSE(°) | 平均主次峰差(dB) | >5°跳峰率 |", "|---|---:|---:|---:|---:|---:|---:|"]
    lines.extend(f"| {row['case_id']} | {row['frames']} | {row['azimuth_rmse_to_dominant_deg']:.4f} | {row['elevation_rmse_to_dominant_deg']:.4f} | {row['combined_rmse_to_dominant_deg']:.4f} | {row['mean_peak_to_second_db']:.4f} | {row['jump_rate_over_5deg']:.4f} |" for row in summaries)
    lines += ["", "## 如何理解参考角度", "", "海杂波不是单一目标。本阶段用每一帧功率最大的合成海面微元作为参考角度，因此 RMSE 表示‘AoA 主峰相对当前最强微元’的偏差，而不是相对固定目标真值。", "", "## 边界", "", "V0.2 输入只有 MATLAB 海面高度场，不包含实测雷达复数回波；微元反射强度、相位和可见性是合成假设。PCB 坐标仍是铜区质心候选，校准误差和噪声也不是实测值。因此结果用于验证海况链路和指标定义，不能称为真实海杂波测量或 AWR2944P 性能。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.input_root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
