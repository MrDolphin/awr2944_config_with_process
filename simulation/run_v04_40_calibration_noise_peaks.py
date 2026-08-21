"""Stress the sparse-array AoA solver with calibration error and noise."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_39_sparse_array_solver import steering_matrix
from simulation.v03 import FmcwConfig
from simulation.v04 import generate_aoa_iq_with_positions


def estimate_with_peaks(config: FmcwConfig, channel: np.ndarray, x: np.ndarray, y: np.ndarray, azimuths: np.ndarray, elevations: np.ndarray) -> dict:
    steering = steering_matrix(config, x, y, azimuths, elevations)
    observation = channel.ravel()
    scores = np.abs(steering.conj() @ observation / max(np.linalg.norm(observation), 1e-15)) ** 2
    order = np.argsort(scores)[::-1]
    best, second = int(order[0]), int(order[1])
    n_el = len(elevations)
    best_az, best_el = float(azimuths[best // n_el]), float(elevations[best % n_el])
    second_az, second_el = float(azimuths[second // n_el]), float(elevations[second % n_el])
    peak_ratio_db = float(10.0 * np.log10(max(scores[best], 1e-15) / max(scores[second], 1e-15)))
    return {"estimated_azimuth_deg": best_az, "estimated_elevation_deg": best_el, "score": float(scores[best]), "second_peak_azimuth_deg": second_az, "second_peak_elevation_deg": second_el, "second_peak_score": float(scores[second]), "peak_to_second_db": peak_ratio_db}


def apply_impairments(channel: np.ndarray, rng: np.random.Generator, gain_std_db: float, phase_std_deg: float, snr_db: float | None) -> np.ndarray:
    gain_db = rng.normal(0.0, gain_std_db, channel.shape)
    phase = np.deg2rad(rng.normal(0.0, phase_std_deg, channel.shape))
    impaired = channel * (10.0 ** (gain_db / 20.0)) * np.exp(1j * phase)
    if snr_db is not None:
        noise_power = np.mean(np.abs(impaired) ** 2) / (10.0 ** (snr_db / 10.0))
        noise = np.sqrt(noise_power / 2.0) * (rng.normal(size=channel.shape) + 1j * rng.normal(size=channel.shape))
        impaired = impaired + noise
    return impaired


def run(mapping_path: Path, pcb_path: Path, output: Path) -> dict:
    config = FmcwConfig()
    models = model_positions(mapping_path, pcb_path, config)
    solver_az = np.arange(-60.0, 60.01, 1.0)
    solver_el = np.arange(-20.0, 20.01, 1.0)
    truth_az = np.arange(-40.0, 40.1, 20.0)
    truth_el = np.arange(-10.0, 10.1, 10.0)
    scenarios = [
        {"scenario": "ideal_clean", "model": "ideal_half_lambda", "gain_std_db": 0.0, "phase_std_deg": 0.0, "snr_db": None, "trials": 1},
        {"scenario": "pcb_clean", "model": "pcb_centroid_candidate", "gain_std_db": 0.0, "phase_std_deg": 0.0, "snr_db": None, "trials": 1},
        {"scenario": "pcb_calibration_1db_5deg", "model": "pcb_centroid_candidate", "gain_std_db": 1.0, "phase_std_deg": 5.0, "snr_db": None, "trials": 20},
        {"scenario": "pcb_snr20_calibration", "model": "pcb_centroid_candidate", "gain_std_db": 1.0, "phase_std_deg": 5.0, "snr_db": 20.0, "trials": 20},
        {"scenario": "pcb_snr10_calibration", "model": "pcb_centroid_candidate", "gain_std_db": 1.0, "phase_std_deg": 5.0, "snr_db": 10.0, "trials": 20},
    ]
    rng = np.random.default_rng(4040)
    rows: list[dict] = []
    for scenario in scenarios:
        x, y = models[scenario["model"]]
        for az in truth_az:
            for el in truth_el:
                iq = generate_aoa_iq_with_positions(config, slant_range_m=30.0, radial_velocity_mps=0.0, azimuth_deg=float(az), elevation_deg=float(el), x_positions_m=x, y_positions_m=y)
                clean = np.mean(iq, axis=(0, 1))
                for trial in range(int(scenario["trials"])):
                    channel = apply_impairments(clean, rng, float(scenario["gain_std_db"]), float(scenario["phase_std_deg"]), scenario["snr_db"])
                    estimate = estimate_with_peaks(config, channel, x, y, solver_az, solver_el)
                    rows.append({**scenario, "trial": trial, "truth_azimuth_deg": float(az), "truth_elevation_deg": float(el), **estimate, "azimuth_error_deg": estimate["estimated_azimuth_deg"] - az, "elevation_error_deg": estimate["estimated_elevation_deg"] - el})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "calibration_noise_peak_trials.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summaries = []
    for scenario in scenarios:
        subset = [row for row in rows if row["scenario"] == scenario["scenario"]]
        az_err = np.asarray([row["azimuth_error_deg"] for row in subset]); el_err = np.asarray([row["elevation_error_deg"] for row in subset])
        summaries.append({"scenario": scenario["scenario"], "model": scenario["model"], "trials": len(subset), "azimuth_rmse_deg": float(np.sqrt(np.mean(az_err ** 2))), "elevation_rmse_deg": float(np.sqrt(np.mean(el_err ** 2))), "combined_rmse_deg": float(np.sqrt(np.mean(np.concatenate((az_err, el_err)) ** 2))), "mean_peak_to_second_db": float(np.mean([row["peak_to_second_db"] for row in subset])), "min_peak_to_second_db": float(np.min([row["peak_to_second_db"] for row in subset])), "mean_score": float(np.mean([row["score"] for row in subset]))})
    with (output / "scenario_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    summary = {"status": "completed_calibration_noise_peak_sweep", "seed": 4040, "scenarios": summaries, "grid_deg": {"azimuth": solver_az.tolist(), "elevation": solver_el.tolist()}, "input_status": "synthetic_iq_only", "hardware_aoa_validated": False, "pcb_model_status": "cad_copper_centroid_candidate_not_phase_center"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.40 校准误差、噪声和多峰统计", "", "本阶段在稀疏阵列导向矢量求解器上加入通道幅度误差、通道相位误差和复高斯噪声，并记录主峰/次峰关系。", "", "| 场景 | 方位 RMSE(°) | 俯仰 RMSE(°) | 综合 RMSE(°) | 平均主次峰差(dB) | 最小主次峰差(dB) |", "|---|---:|---:|---:|---:|---:"]
    lines.extend(f"| {row['scenario']} | {row['azimuth_rmse_deg']:.4f} | {row['elevation_rmse_deg']:.4f} | {row['combined_rmse_deg']:.4f} | {row['mean_peak_to_second_db']:.4f} | {row['min_peak_to_second_db']:.4f} |" for row in summaries)
    lines += ["", "## 指标解释", "", "RMSE 表示角度估计偏差；主次峰差越小，表示存在更强的竞争角度峰，结果越不稳定；score 是观测通道与最佳导向矢量的归一化相关性。", "", "## 边界", "", "校准误差采用随机独立通道增益/相位扰动，不是 TI 实测校准矩阵；噪声是复高斯模型，不等于海杂波；PCB 仍是铜区质心候选，不是电气相位中心。因此结果用于误差预算和算法回归，不能作为真实 AWR2944P 性能指标。", ""]
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
