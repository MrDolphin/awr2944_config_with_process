"""A/B test direct, inverse and conjugate-inverse complex calibration directions."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.calibration import apply_channel_correction, complex_matrix, load_calibration
from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_positions, generate_aoa_iq_with_positions


def geometry(path: Path, config: FmcwConfig):
    rows = list(csv.DictReader(path.open(encoding="utf-8", newline="")))
    x = np.zeros((4, 4)); y = np.zeros((4, 4))
    for row in rows:
        rx, tx = int(row["rx_index"]), int(row["tx_index"])
        x[rx, tx] = float(row["column"]) * float(row["azimuth_spacing_lambda"]) * config.wavelength_m
        y[rx, tx] = float(row["row"]) * float(row["elevation_spacing_lambda"]) * config.wavelength_m
    return x, y


def scan(config, x, y, error, correction):
    values = []
    for elevation in np.arange(-20.0, 20.1, 10.0):
        for azimuth in np.arange(-60.0, 60.1, 20.0):
            iq = generate_aoa_iq_with_positions(config, slant_range_m=30.0, radial_velocity_mps=0.0,
                                                 azimuth_deg=float(azimuth), elevation_deg=float(elevation),
                                                 x_positions_m=x, y_positions_m=y)
            channel = apply_channel_correction(np.mean(iq, axis=(0, 1)) * error, correction)
            est_az, est_el = estimate_aoa_from_positions(channel, config, x, y)
            values.append((est_az - azimuth, est_el - elevation))
    values = np.asarray(values)
    return {"azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))),
            "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))),
            "azimuth_max_abs_error_deg": float(np.max(np.abs(values[:, 0]))),
            "elevation_max_abs_error_deg": float(np.max(np.abs(values[:, 1])))}


def run(geometry_csv: Path, cfg_calibration: Path, output: Path):
    config = FmcwConfig(); x, y = geometry(geometry_csv, config)
    cfg_matrix, cfg_meta = load_calibration(cfg_calibration)
    amplitude = np.array([[1.00, 0.98, 1.03, 1.01], [1.02, 0.97, 1.01, 1.04],
                          [0.99, 1.03, 0.96, 1.02], [1.01, 1.00, 1.04, 0.98]])
    phase_deg = np.array([[0.0, 3.0, 6.0, 9.0], [2.0, 5.0, 8.0, 11.0],
                          [-2.0, 1.0, 4.0, 7.0], [-4.0, -1.0, 2.0, 5.0]])
    measured_error = complex_matrix(amplitude, phase_deg)
    candidates = {
        "no_correction": np.ones((4, 4), dtype=complex),
        "direct_cfg_factor": measured_error,
        "inverse_cfg_factor": 1.0 / measured_error,
        "conjugate_inverse_cfg_factor": 1.0 / np.conj(measured_error),
    }
    rows = [{"case": name, **scan(config, x, y, measured_error, correction)}
            for name, correction in candidates.items()]
    output.mkdir(parents=True, exist_ok=True)
    with (output / "direction_scan.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    payload = {"cfg_calibration_source": str(cfg_calibration.resolve()), "cfg_metadata": cfg_meta,
               "synthetic_measured_error": {"amplitude": amplitude.tolist(), "phase_deg": phase_deg.tolist()},
               "results": rows, "interpretation_status": "direction_ab_only_synthetic_error"}
    (output / "summary.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.20 CFG 校准方向 A/B\n\n"
        "对同一组合成通道误差分别使用直接因子、复数逆和共轭逆。结果只验证数学方向，"
        "不能单独证明 TI 固件的校准约定；真实测量仍需角反射器 A/B。\n",
        encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--geometry-csv", type=Path, required=True)
    parser.add_argument("--cfg-calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.geometry_csv.resolve(), args.cfg_calibration.resolve(), args.output.resolve())
    print(f"Generated calibration direction A/B in {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
