"""Pairwise scan of candidate array-coordinate semantics and AoA error."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_positions, generate_aoa_iq_with_positions


def load_mapping(path: Path, config: FmcwConfig) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    with path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    models = {name: (np.zeros((4, 4)), np.zeros((4, 4))) for name in ("cfg_raw", "cfg_swap_row_column", "tx_rx_regular", "tx_rx_swapped")}
    for row in rows:
        rx, tx = int(row["rx_index"]), int(row["tx_index"])
        row_index, column = float(row["row"]), float(row["column"])
        az_step = float(row["azimuth_spacing_lambda"]) * config.wavelength_m
        el_step = float(row["elevation_spacing_lambda"]) * config.wavelength_m
        models["cfg_raw"][0][rx, tx] = column * az_step
        models["cfg_raw"][1][rx, tx] = row_index * el_step
        models["cfg_swap_row_column"][0][rx, tx] = row_index * el_step
        models["cfg_swap_row_column"][1][rx, tx] = column * az_step
        models["tx_rx_regular"][0][rx, tx] = tx * config.wavelength_m / 2.0
        models["tx_rx_regular"][1][rx, tx] = rx * config.wavelength_m / 2.0
        models["tx_rx_swapped"][0][rx, tx] = rx * config.wavelength_m / 2.0
        models["tx_rx_swapped"][1][rx, tx] = tx * config.wavelength_m / 2.0
    raw_x, raw_y = models["cfg_raw"]
    models["cfg_y_mirror"] = (raw_x.copy(), np.max(raw_y) - raw_y)
    regular_x, regular_y = models["tx_rx_regular"]
    models["regular_y_mirror"] = (regular_x.copy(), np.max(regular_y) - regular_y)
    return models


def scan_pair(config: FmcwConfig, actual: tuple[np.ndarray, np.ndarray], assumed: tuple[np.ndarray, np.ndarray], azimuths: np.ndarray, elevations: np.ndarray) -> dict:
    errors = []
    for elevation in elevations:
        for azimuth in azimuths:
            iq = generate_aoa_iq_with_positions(config, slant_range_m=30.0, radial_velocity_mps=0.0, azimuth_deg=float(azimuth), elevation_deg=float(elevation), x_positions_m=actual[0], y_positions_m=actual[1])
            estimated = estimate_aoa_from_positions(np.mean(iq, axis=(0, 1)), config, assumed[0], assumed[1])
            errors.append((estimated[0] - azimuth, estimated[1] - elevation))
    values = np.asarray(errors)
    return {"azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))), "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))), "azimuth_max_abs_error_deg": float(np.max(np.abs(values[:, 0]))), "elevation_max_abs_error_deg": float(np.max(np.abs(values[:, 1]))), "combined_rmse_deg": float(np.sqrt(np.mean(values ** 2)))}


def run(mapping_path: Path, output: Path) -> dict:
    config = FmcwConfig()
    models = load_mapping(mapping_path, config)
    azimuths = np.arange(-60.0, 60.1, 20.0)
    elevations = np.arange(-20.0, 20.1, 10.0)
    rows = []
    for actual_name, actual in models.items():
        for assumed_name, assumed in models.items():
            rows.append({"actual_model": actual_name, "assumed_model": assumed_name, **scan_pair(config, actual, assumed, azimuths, elevations)})
    output.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0])
    with (output / "geometry_semantics_pairwise.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    best_by_actual = {}
    for actual_name in models:
        candidates = [row for row in rows if row["actual_model"] == actual_name]
        best_by_actual[actual_name] = min(candidates, key=lambda row: row["combined_rmse_deg"])
    summary = {"status": "completed_geometry_semantics_pairwise_scan", "model_names": list(models), "azimuth_grid_deg": azimuths.tolist(), "elevation_grid_deg": elevations.tolist(), "best_assumed_by_actual": best_by_actual, "hardware_geometry_confirmed": False, "interpretation": "pairwise_semantic_screening_not_hardware_validation"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.34 阵列几何语义成对扫描", "", f"扫描方位：{azimuths.min():.0f}°～{azimuths.max():.0f}°；俯仰：{elevations.min():.0f}°～{elevations.max():.0f}°。", "", "本阶段不假设任何一个模型是真实 EVM 阵列，而是对候选‘实际几何—估计几何’组合逐对计算 AoA 误差。", "", "| 假设实际模型 | 最优估计模型 | 综合 RMSE(°) |", "|---|---|---:|"]
    for actual_name, row in best_by_actual.items():
        lines.append(f"| {actual_name} | {row['assumed_model']} | {row['combined_rmse_deg']:.4f} |")
    lines += ["", "## 结论边界", "", "对角度网格上的大误差说明对应坐标语义会发生空间混叠或相位展开失败；它不能单独证明哪一个模型是硬件真实模型。必须使用装配基准、Altium 网表/封装和已知角 DCA1000 数据选择模型。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.mapping.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
