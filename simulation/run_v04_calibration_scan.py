"""Synthetic AoA calibration-direction scan for the 4x4 virtual channel contract."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.calibration import apply_channel_correction, complex_matrix
from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_positions, generate_aoa_iq_with_positions


def load_geometry(path: Path, config: FmcwConfig):
    rows = list(csv.DictReader(path.open(encoding="utf-8", newline="")))
    x = np.zeros((4, 4)); y = np.zeros((4, 4))
    for row in rows:
        rx, tx = int(row["rx_index"]), int(row["tx_index"])
        x[rx, tx] = float(row["column"]) * float(row["azimuth_spacing_lambda"]) * config.wavelength_m
        y[rx, tx] = float(row["row"]) * float(row["elevation_spacing_lambda"]) * config.wavelength_m
    return x, y


def scan(config, x, y, raw_error: np.ndarray, correction: np.ndarray | None) -> dict[str, float]:
    errors = []
    for elevation in np.arange(-20.0, 20.1, 10.0):
        for azimuth in np.arange(-60.0, 60.1, 20.0):
            iq = generate_aoa_iq_with_positions(
                config, slant_range_m=30.0, radial_velocity_mps=0.0,
                azimuth_deg=float(azimuth), elevation_deg=float(elevation),
                x_positions_m=x, y_positions_m=y,
            )
            channel = np.mean(iq, axis=(0, 1)) * raw_error
            if correction is not None:
                channel = apply_channel_correction(channel, correction)
            est_az, est_el = estimate_aoa_from_positions(channel, config, x, y)
            errors.append((est_az - azimuth, est_el - elevation))
    values = np.asarray(errors)
    return {
        "azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))),
        "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))),
        "azimuth_max_abs_error_deg": float(np.max(np.abs(values[:, 0]))),
        "elevation_max_abs_error_deg": float(np.max(np.abs(values[:, 1]))),
    }


def run(geometry_csv: Path, output: Path) -> None:
    config = FmcwConfig()
    x, y = load_geometry(geometry_csv, config)
    amplitude_error = np.array([[1.00, 0.98, 1.03, 1.01], [1.02, 0.97, 1.01, 1.04],
                                [0.99, 1.03, 0.96, 1.02], [1.01, 1.00, 1.04, 0.98]])
    phase_error_deg = np.array([[0.0, 3.0, 6.0, 9.0], [2.0, 5.0, 8.0, 11.0],
                                [-2.0, 1.0, 4.0, 7.0], [-4.0, -1.0, 2.0, 5.0]])
    raw_error = complex_matrix(amplitude_error, phase_error_deg)
    ideal_correction = 1.0 / raw_error
    wrong_correction = complex_matrix(np.ones((4, 4)), -phase_error_deg * 0.5)
    results = {
        "no_calibration": scan(config, x, y, raw_error, None),
        "ideal_inverse_correction": scan(config, x, y, raw_error, ideal_correction),
        "wrong_half_phase_correction": scan(config, x, y, raw_error, wrong_correction),
    }
    output.mkdir(parents=True, exist_ok=True)
    calibration_payload = {
        "schema_version": "awr2944p-calibration-v0.1", "channel_order": "rx0..rx3 rows, tx0..tx3 columns",
        "amplitude": amplitude_error.tolist(), "phase_deg": phase_error_deg.tolist(),
        "calibration_status": "synthetic_injected_error_not_measured",
        "source": "V0.4.18 deterministic regression fixture",
    }
    (output / "synthetic_calibration.json").write_text(json.dumps(calibration_payload, indent=2), encoding="utf-8")
    (output / "summary.json").write_text(json.dumps({"results": results, "calibration": calibration_payload}, indent=2), encoding="utf-8")
    with (output / "calibration_scan.csv").open("w", encoding="utf-8", newline="") as handle:
        rows = [{"case": name, **metrics} for name, metrics in results.items()]
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    (output / "output_analysis.md").write_text(
        "# V0.4.18 复数通道校准扫描\n\n"
        "测试已知幅度和相位误差，比较无校准、理想逆补偿和错误半相位补偿。理想逆补偿应恢复到合成基线；"
        "该结果只验证补偿接口方向和数据契约，不代表真实 EVM 校准。\n",
        encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--geometry-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.geometry_csv.resolve(), args.output.resolve())
    print(f"Generated calibration scan in {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
