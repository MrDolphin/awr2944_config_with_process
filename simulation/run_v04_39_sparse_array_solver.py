"""Grid-search AoA solver for sparse/non-uniform virtual-array candidates.

Unlike the V0.4 phase-plane estimator, this solver compares the measured
complex channel vector with a steering vector for every candidate angle.  It
still requires a correct phase-centre model and is a synthetic geometry test,
not TI SDK or hardware validation.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.v03 import FmcwConfig
from simulation.v04 import generate_aoa_iq_with_positions


def steering_matrix(config: FmcwConfig, x: np.ndarray, y: np.ndarray, azimuths: np.ndarray, elevations: np.ndarray) -> np.ndarray:
    az, el = np.meshgrid(np.deg2rad(azimuths), np.deg2rad(elevations), indexing="ij")
    phase = (2.0 * np.pi / config.wavelength_m) * (az[..., None] * 0.0 + x.ravel()[None, None, :] * np.sin(az)[..., None] * np.cos(el)[..., None] + y.ravel()[None, None, :] * np.sin(el)[..., None])
    matrix = np.exp(1j * phase).reshape(-1, 16)
    return matrix / np.sqrt(16.0)


def estimate_sparse_grid(config: FmcwConfig, channel: np.ndarray, x: np.ndarray, y: np.ndarray, azimuths: np.ndarray, elevations: np.ndarray) -> tuple[float, float, float]:
    if channel.shape != (4, 4):
        raise ValueError("channel must have shape (4, 4)")
    steering = steering_matrix(config, x, y, azimuths, elevations)
    observation = channel.ravel()
    norm = np.linalg.norm(observation)
    if norm == 0:
        raise ValueError("channel must not be all zero")
    scores = np.abs(steering.conj() @ observation / norm) ** 2
    best = int(np.argmax(scores))
    return float(azimuths[best // len(elevations)]), float(elevations[best % len(elevations)]), float(scores[best])


def scan_pair(config: FmcwConfig, actual: tuple[np.ndarray, np.ndarray], assumed: tuple[np.ndarray, np.ndarray], azimuths: np.ndarray, elevations: np.ndarray, solver_azimuths: np.ndarray, solver_elevations: np.ndarray) -> tuple[dict, list[dict]]:
    rows: list[dict] = []
    for elevation in elevations:
        for azimuth in azimuths:
            iq = generate_aoa_iq_with_positions(config, slant_range_m=30.0, radial_velocity_mps=0.0, azimuth_deg=float(azimuth), elevation_deg=float(elevation), x_positions_m=actual[0], y_positions_m=actual[1])
            channel = np.mean(iq, axis=(0, 1))
            estimate = estimate_sparse_grid(config, channel, assumed[0], assumed[1], solver_azimuths, solver_elevations)
            rows.append({"truth_azimuth_deg": float(azimuth), "truth_elevation_deg": float(elevation), "estimated_azimuth_deg": estimate[0], "estimated_elevation_deg": estimate[1], "score": estimate[2], "azimuth_error_deg": estimate[0] - azimuth, "elevation_error_deg": estimate[1] - elevation})
    values = np.asarray([[row["azimuth_error_deg"], row["elevation_error_deg"]] for row in rows])
    return {"azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))), "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))), "azimuth_max_abs_error_deg": float(np.max(np.abs(values[:, 0]))), "elevation_max_abs_error_deg": float(np.max(np.abs(values[:, 1]))), "combined_rmse_deg": float(np.sqrt(np.mean(values ** 2)))}, rows


def run(mapping_path: Path, pcb_path: Path, output: Path) -> dict:
    config = FmcwConfig()
    models = model_positions(mapping_path, pcb_path, config)
    truth_az = np.arange(-60.0, 60.1, 20.0)
    truth_el = np.arange(-20.0, 20.1, 10.0)
    solver_az = np.arange(-60.0, 60.01, 1.0)
    solver_el = np.arange(-20.0, 20.01, 1.0)
    metrics: list[dict] = []
    details: list[dict] = []
    for actual_name, actual in models.items():
        for assumed_name, assumed in models.items():
            result, rows = scan_pair(config, actual, assumed, truth_az, truth_el, solver_az, solver_el)
            metrics.append({"actual_model": actual_name, "assumed_model": assumed_name, **result})
            details.extend({"actual_model": actual_name, "assumed_model": assumed_name, **row} for row in rows)
    output.mkdir(parents=True, exist_ok=True)
    for filename, rows in (("sparse_pairwise_metrics.csv", metrics), ("sparse_pairwise_errors.csv", details)):
        with (output / filename).open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    best = {actual: min((row for row in metrics if row["actual_model"] == actual), key=lambda row: row["combined_rmse_deg"]) for actual in models}
    summary = {"status": "completed_sparse_array_grid_solver", "models": list(models), "truth_grid": {"azimuth_deg": truth_az.tolist(), "elevation_deg": truth_el.tolist()}, "solver_grid": {"azimuth_deg": solver_az.tolist(), "elevation_deg": solver_el.tolist()}, "best_assumed_by_actual": best, "solver_status": "steering_vector_grid_search_unknown_complex_amplitude", "hardware_aoa_validated": False, "pcb_model_status": "cad_copper_centroid_candidate_not_phase_center"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.39 稀疏阵列导向矢量 AoA 扫描", "", "本阶段使用逐角度导向矢量匹配替代固定轴相位 unwrap，用于检查 PCB 多波长间距候选阵列的相位折叠风险。", "", "| 实际几何 | 估计几何 | 方位 RMSE(°) | 俯仰 RMSE(°) | 综合 RMSE(°) |", "|---|---|---:|---:|---:"]
    lines.extend(f"| {row['actual_model']} | {row['assumed_model']} | {row['azimuth_rmse_deg']:.4f} | {row['elevation_rmse_deg']:.4f} | {row['combined_rmse_deg']:.4f} |" for row in metrics)
    lines += ["", "## 解释", "", "导向矢量网格搜索对每个候选角度计算复数通道相关性，并消除未知整体复幅度；它不依赖沿阵列轴线的相位 unwrap。若同一模型仍出现多个近似峰，说明阵列存在真实的角度歧义，而不是简单的 unwrap 失败。", "", "## 边界", "", "PCB 仍是 RF 铜区质心候选，不是电气相位中心；输入为无噪声合成 IQ，未包含实测校准矩阵、互耦、方向图和海杂波。结果不能作为真实 AWR2944P AoA 精度。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--pcb", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.mapping.resolve(), args.pcb.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
