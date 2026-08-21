"""Sensitivity-only AoA scan for differential TX/RX PCB route phase."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_positions, generate_aoa_iq_with_positions


def load_geometry(path: Path, config: FmcwConfig) -> tuple[np.ndarray, np.ndarray]:
    rows = list(csv.DictReader(path.open(encoding="utf-8", newline="")))
    x = np.zeros((4, 4), dtype=float); y = np.zeros((4, 4), dtype=float)
    for row in rows:
        rx, tx = int(row["rx_index"]), int(row["tx_index"])
        x[rx, tx] = float(row["column"]) * float(row["azimuth_spacing_lambda"]) * config.wavelength_m
        y[rx, tx] = float(row["row"]) * float(row["elevation_spacing_lambda"]) * config.wavelength_m
    return x, y


def load_lengths(path: Path) -> tuple[np.ndarray, np.ndarray]:
    tx = np.zeros(4); rx = np.zeros(4)
    for row in csv.DictReader(path.open(encoding="utf-8", newline="")):
        index = int(row["antenna"][-1]) - 1
        length = float(row["shortest_pad_to_region_mm"]) * 1e-3
        (tx if row["antenna"].startswith("TX") else rx)[index] = length
    return tx, rx


def scan(config, x, y, tx_lengths, rx_lengths, effective_eps: float) -> dict[str, float]:
    wavelength = config.wavelength_m
    total = tx_lengths[None, :] + rx_lengths[:, None]
    phase = 2 * np.pi / wavelength * (np.sqrt(effective_eps) - 1.0) * total
    phase -= np.mean(phase)
    errors = []
    for elevation in np.arange(-20.0, 20.1, 10.0):
        for azimuth in np.arange(-60.0, 60.1, 20.0):
            iq = generate_aoa_iq_with_positions(
                config, slant_range_m=30.0, radial_velocity_mps=0.0,
                azimuth_deg=float(azimuth), elevation_deg=float(elevation),
                x_positions_m=x, y_positions_m=y,
            )
            channel = np.mean(iq, axis=(0, 1)) * np.exp(1j * phase)
            est_az, est_el = estimate_aoa_from_positions(channel, config, x, y)
            errors.append((est_az - azimuth, est_el - elevation))
    values = np.asarray(errors)
    return {
        "effective_eps_r": effective_eps,
        "phase_rms_deg": float(np.rad2deg(np.sqrt(np.mean((phase - np.mean(phase)) ** 2)))),
        "azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))),
        "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))),
        "azimuth_max_abs_error_deg": float(np.max(np.abs(values[:, 0]))),
        "elevation_max_abs_error_deg": float(np.max(np.abs(values[:, 1]))),
        "interpretation_status": "sensitivity_only_not_calibrated_phase_model",
    }


def run(geometry_csv: Path, connectivity_csv: Path, output: Path) -> None:
    config = FmcwConfig()
    x, y = load_geometry(geometry_csv, config)
    tx, rx = load_lengths(connectivity_csv)
    results = [scan(config, x, y, tx, rx, eps) for eps in (1.0, 2.5, 3.0, 4.04)]
    output.mkdir(parents=True, exist_ok=True)
    with (output / "phase_sensitivity.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(results[0])); writer.writeheader(); writer.writerows(results)
    payload = {"geometry_source": str(geometry_csv.resolve()), "connectivity_source": str(connectivity_csv.resolve()),
               "tx_route_lengths_mm": (tx * 1000).tolist(), "rx_route_lengths_mm": (rx * 1000).tolist(), "results": results,
               "interpretation_status": "sensitivity_only_not_calibrated_phase_model"}
    (output / "summary.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.17 PCB 走线相位敏感性分析\n\n"
        "使用 TX/RX Pad 到 Region 的几何路径长度，按可配置有效介电常数生成差分相位扰动。"
        "该模型只做敏感性分析，不能替代微带线/全波仿真或实测校准。详细结果见 `phase_sensitivity.csv`。\n",
        encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--geometry-csv", type=Path, required=True)
    parser.add_argument("--connectivity-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.geometry_csv.resolve(), args.connectivity_csv.resolve(), args.output.resolve())
    print(f"Generated phase sensitivity scan in {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
