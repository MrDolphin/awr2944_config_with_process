"""Scan PCB coordinate interpretations against the CFG ideal AoA estimator."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.v03 import FmcwConfig
from simulation.v04 import (
    estimate_aoa_from_positions,
    generate_aoa_iq_with_positions,
)

from simulation.run_v04_array_comparison import _load_cad


def _load_cfg_geometry(path: Path, config: FmcwConfig) -> tuple[np.ndarray, np.ndarray]:
    with path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    x = np.zeros((4, 4), dtype=float)
    y = np.zeros((4, 4), dtype=float)
    for row in rows:
        rx = int(row["rx_index"])
        tx = int(row["tx_index"])
        x[rx, tx] = float(row["column"]) * float(row["azimuth_spacing_lambda"]) * config.wavelength_m
        y[rx, tx] = float(row["row"]) * float(row["elevation_spacing_lambda"]) * config.wavelength_m
    return x, y


def transforms(x: np.ndarray, y: np.ndarray) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    xmax, ymax = float(np.max(x)), float(np.max(y))
    return {
        "raw": (x.copy(), y.copy()),
        "x_mirror": (xmax - x, y.copy()),
        "y_mirror": (x.copy(), ymax - y),
        "rotate_180": (xmax - x, ymax - y),
    }


def scan(config: FmcwConfig, actual_x: np.ndarray, actual_y: np.ndarray,
         assumed_x: np.ndarray, assumed_y: np.ndarray) -> dict[str, float]:
    errors: list[tuple[float, float]] = []
    for el in np.arange(-20.0, 20.1, 10.0):
        for az in np.arange(-60.0, 60.1, 20.0):
            iq = generate_aoa_iq_with_positions(
                config, slant_range_m=30.0, radial_velocity_mps=0.0,
                azimuth_deg=float(az), elevation_deg=float(el),
                x_positions_m=actual_x, y_positions_m=actual_y,
            )
            est_az, est_el = estimate_aoa_from_positions(
                np.mean(iq, axis=(0, 1)), config, assumed_x, assumed_y
            )
            errors.append((est_az - az, est_el - el))
    values = np.asarray(errors)
    return {
        "azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))),
        "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))),
        "azimuth_max_abs_error_deg": float(np.max(np.abs(values[:, 0]))),
        "elevation_max_abs_error_deg": float(np.max(np.abs(values[:, 1]))),
    }


def run(cad_csv: Path, geometry_csv: Path, output: Path) -> dict[str, dict[str, float]]:
    config = FmcwConfig()
    cfg_x, cfg_y = _load_cfg_geometry(geometry_csv, config)
    pcb_x, pcb_y = _load_cad(cad_csv, config)
    results: dict[str, dict[str, float]] = {}
    rows = []
    for name, (x, y) in transforms(pcb_x, pcb_y).items():
        metric = scan(config, x * 1e0, y * 1e0, cfg_x, cfg_y)
        metric["coordinate_status"] = "pcb_centroid_generated_cfg_ideal_estimator"
        results[name] = metric
        rows.append({"transform": name, **metric})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "transform_scan.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    payload = {"cad_source": str(cad_csv.resolve()), "geometry_source": str(geometry_csv.resolve()), "scan_azimuth_deg": [-60, 60],
               "scan_elevation_deg": [-20, 20], "results": results,
               "interpretation_status": "coordinate_transform_screening_only"}
    (output / "summary.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    best = min(results, key=lambda name: results[name]["azimuth_rmse_deg"] + results[name]["elevation_rmse_deg"])
    (output / "output_analysis.md").write_text(
        "# V0.4.13 坐标变换 AoA 筛查\n\n"
        f"在 PCB 坐标生成相位、CFG 理想阵列估计的条件下，四种坐标解释中综合 RMSE 最小的是 `{best}`。\n\n"
        "这只是坐标系筛查，不是相位中心验证：未包含真实幅相误差、馈电网络、天线方向图、校准矩阵或实测角度真值。"
        "只有当板面法向和安装方向得到资料或实测确认后，才能选择一个变换进入正式 AoA 模型。\n",
        encoding="utf-8")
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cad-csv", type=Path, required=True)
    parser.add_argument("--geometry-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = run(args.cad_csv.resolve(), args.geometry_csv.resolve(), args.output.resolve())
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
