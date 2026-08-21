"""Generate a deterministic known-angle HDF5 fixture for contract regression only."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.v03 import FmcwConfig
from simulation.v04 import generate_aoa_iq_with_positions, generate_aoa_scatterer_iq


def load_geometry(path: Path, config: FmcwConfig) -> tuple[np.ndarray, np.ndarray]:
    import csv
    x = np.zeros((4, 4)); y = np.zeros((4, 4))
    with path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            rx, tx = int(row["rx_index"]), int(row["tx_index"])
            x[rx, tx] = float(row["column"]) * float(row["azimuth_spacing_lambda"]) * config.wavelength_m
            y[rx, tx] = float(row["row"]) * float(row["elevation_spacing_lambda"]) * config.wavelength_m
    return x, y


def run(output: Path, *, azimuth_deg: float, elevation_deg: float, slant_range_m: float, radial_velocity_mps: float, frames: int, samples: int, geometry_csv: Path | None = None) -> dict:
    config = FmcwConfig(samples_per_chirp=samples, chirps_per_frame=frames)
    if geometry_csv:
        x, y = load_geometry(geometry_csv, config)
        virtual = generate_aoa_iq_with_positions(config, slant_range_m=slant_range_m, radial_velocity_mps=radial_velocity_mps, azimuth_deg=azimuth_deg, elevation_deg=elevation_deg, x_positions_m=x, y_positions_m=y)
    else:
        virtual = generate_aoa_scatterer_iq(config, slant_range_m=slant_range_m, radial_velocity_mps=radial_velocity_mps, azimuth_deg=azimuth_deg, elevation_deg=elevation_deg)
    output.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output, "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-known-angle-fixture-v0.4.33"
        handle.attrs["source_type"] = "synthetic_known_angle_regression_only"
        handle.attrs["channel_order_verified"] = True
        handle.attrs["tx_sequence"] = "TX1,TX2,TX3,TX4"
        handle.attrs["calibration_status"] = "synthetic_unity_not_measured"
        handle.attrs["geometry_source"] = str(geometry_csv.resolve()) if geometry_csv else "simulation.v04.virtual_array_positions"
        handle.attrs["truth_azimuth_deg"] = azimuth_deg
        handle.attrs["truth_elevation_deg"] = elevation_deg
        handle.attrs["truth_range_m"] = slant_range_m
        handle.attrs["truth_velocity_mps"] = radial_velocity_mps
        handle.create_dataset("/decoded/iq", data=virtual, compression="gzip")
        handle.create_dataset("/recovered/virtual_iq", data=virtual, compression="gzip")
    summary = {"status": "completed_synthetic_known_angle_fixture", "output": str(output.resolve()), "virtual_shape": list(virtual.shape), "truth_azimuth_deg": azimuth_deg, "truth_elevation_deg": elevation_deg, "truth_range_m": slant_range_m, "truth_velocity_mps": radial_velocity_mps, "channel_order_verified": True, "calibration_status": "synthetic_unity_not_measured", "geometry_source": str(geometry_csv.resolve()) if geometry_csv else "simulation.v04.virtual_array_positions", "real_measurement": False}
    output.with_suffix(".json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--azimuth-deg", type=float, default=15.0)
    parser.add_argument("--elevation-deg", type=float, default=8.0)
    parser.add_argument("--range-m", type=float, default=30.0)
    parser.add_argument("--velocity-mps", type=float, default=0.0)
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--geometry-csv", type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.output.resolve(), azimuth_deg=args.azimuth_deg, elevation_deg=args.elevation_deg, slant_range_m=args.range_m, radial_velocity_mps=args.velocity_mps, frames=args.frames, samples=args.samples, geometry_csv=args.geometry_csv.resolve() if args.geometry_csv else None), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
