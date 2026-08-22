"""Grid beam-scan AoA for the four candidate virtual-array geometries."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_128_array_model_comparison import models
from simulation.v03 import FmcwConfig
from simulation.v04 import generate_aoa_iq_with_positions


def estimate_grid(channel: np.ndarray, config: FmcwConfig, positions: tuple[np.ndarray, np.ndarray], az_grid: np.ndarray, el_grid: np.ndarray) -> tuple[float, float, float]:
    x, y = positions
    az, el = np.meshgrid(np.deg2rad(az_grid), np.deg2rad(el_grid), indexing="ij")
    phase = (2.0 * np.pi / config.wavelength_m) * (x.ravel()[None, :] * (np.sin(az) * np.cos(el)).ravel()[:, None] + y.ravel()[None, :] * np.sin(el).ravel()[:, None])
    score = np.abs(np.exp(-1j * phase) @ channel.ravel())
    index = int(np.argmax(score)); ai, ei = np.unravel_index(index, (len(az_grid), len(el_grid)))
    return float(az_grid[ai]), float(el_grid[ei]), float(score[index] / max(np.sum(np.abs(channel)), 1e-12))


def run(mapping: Path, copper: Path, endpoint: Path, output: Path) -> dict:
    config = FmcwConfig(); model_map = models(mapping, copper, endpoint, config)
    az_grid = np.arange(-80.0, 80.1, 2.0); el_grid = np.arange(-40.0, 40.1, 2.0)
    truths_az = np.arange(-60.0, 60.1, 20.0); truths_el = np.arange(-20.0, 20.1, 10.0)
    rows = []
    for actual_name, actual in model_map.items():
        for assumed_name, assumed in model_map.items():
            for elevation in truths_el:
                for azimuth in truths_az:
                    iq = generate_aoa_iq_with_positions(config, slant_range_m=30.0, radial_velocity_mps=0.0, azimuth_deg=float(azimuth), elevation_deg=float(elevation), x_positions_m=actual[0], y_positions_m=actual[1])
                    est_az, est_el, peak = estimate_grid(np.mean(iq, axis=(0, 1)), config, assumed, az_grid, el_grid)
                    rows.append({"actual_model": actual_name, "assumed_model": assumed_name, "truth_azimuth_deg": azimuth, "truth_elevation_deg": elevation, "estimated_azimuth_deg": est_az, "estimated_elevation_deg": est_el, "azimuth_error_deg": est_az - azimuth, "elevation_error_deg": est_el - elevation, "normalized_beam_peak": peak})
    metrics = []
    for actual_name in model_map:
        for assumed_name in model_map:
            part = [row for row in rows if row["actual_model"] == actual_name and row["assumed_model"] == assumed_name]
            az_err = np.asarray([row["azimuth_error_deg"] for row in part]); el_err = np.asarray([row["elevation_error_deg"] for row in part])
            metrics.append({"actual_model": actual_name, "assumed_model": assumed_name, "azimuth_rmse_deg": float(np.sqrt(np.mean(az_err ** 2))), "elevation_rmse_deg": float(np.sqrt(np.mean(el_err ** 2))), "combined_rmse_deg": float(np.sqrt(np.mean(np.r_[az_err, el_err] ** 2))), "exact_grid_hit_fraction": float(np.mean((az_err == 0) & (el_err == 0)))})
    output.mkdir(parents=True, exist_ok=True)
    for name, data in (("grid_beam_errors.csv", rows), ("grid_beam_metrics.csv", metrics)):
        with (output / name).open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(data[0])); writer.writeheader(); writer.writerows(data)
    result = {"status": "completed_grid_beam_aoa", "models": list(model_map), "azimuth_grid_step_deg": 2.0, "elevation_grid_step_deg": 2.0, "truth_case_count": len(rows), "hardware_aoa_validated": False, "phase_center_ready": False}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.129 网格波束扫描 AoA", "", "本阶段用阵列流形网格扫描替代 V0.4.128 的相位平面 unwrap，扫描方位 -80°～80°（2°步长）和俯仰 -40°～40°（2°步长）。", "", "## 结果含义", "", "对每个候选角度计算阵列流形与 4×4 虚拟通道的相干积累，峰值对应估计角度。`grid_beam_metrics.csv` 给出四种模型两两配对的 RMSE 和真值落在网格上的命中比例。", "", "## 解释边界", "", "同模型自匹配若命中率高、RMSE 低，说明网格波束扫描消除了上一阶段 unwrap 的主要相位歧义；跨模型误差仍表示几何定义不同。由于 IQ 是合成的、没有噪声/互耦/方向图/校准误差，结果不能等同于真实 EVM AoA 性能。", "", "## 下一步", "", "将把稳定的网格 AoA 接到 V0.120 海面微面元点云，统计不同阵列模型下的海杂波角度分布与方向图权重差异，再用真实 DCA1000 IQ 验证。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--mapping", type=Path, required=True); parser.add_argument("--copper", type=Path, required=True); parser.add_argument("--endpoint", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.mapping.resolve(), args.copper.resolve(), args.endpoint.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
