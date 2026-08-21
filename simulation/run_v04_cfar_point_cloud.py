"""2-D CA-CFAR and peak-cell AoA point-cloud extraction."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.v04 import estimate_aoa_from_positions


def geometry(path: Path, wavelength_m: float):
    with path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    x = np.zeros((4, 4)); y = np.zeros((4, 4))
    for row in rows:
        rx, tx = int(row["rx_index"]), int(row["tx_index"])
        x[rx, tx] = float(row["column"]) * float(row["azimuth_spacing_lambda"]) * wavelength_m
        y[rx, tx] = float(row["row"]) * float(row["elevation_spacing_lambda"]) * wavelength_m
    return x, y


def ca_cfar(power: np.ndarray, *, training: tuple[int, int], guard: tuple[int, int], pfa: float):
    if power.ndim != 2 or not (0.0 < pfa < 1.0):
        raise ValueError("power must be 2-D and pfa must be in (0,1)")
    tr_d, tr_r = training; gd, gr = guard
    rows, cols = power.shape
    detections = []
    alpha_count = (2 * (tr_d + gd) + 1) * (2 * (tr_r + gr) + 1) - (2 * gd + 1) * (2 * gr + 1)
    alpha = alpha_count * (pfa ** (-1.0 / alpha_count) - 1.0)
    for d in range(tr_d + gd, rows - tr_d - gd):
        for r in range(tr_r + gr, cols - tr_r - gr):
            outer = power[d - tr_d - gd:d + tr_d + gd + 1, r - tr_r - gr:r + tr_r + gr + 1]
            mask = np.ones_like(outer, dtype=bool)
            mask[tr_d:tr_d + 2 * gd + 1, tr_r:tr_r + 2 * gr + 1] = False
            noise = float(np.mean(outer[mask]))
            threshold = alpha * noise
            neighborhood = power[max(0, d - 1):d + 2, max(0, r - 1):r + 2]
            if power[d, r] > threshold and power[d, r] >= float(np.max(neighborhood)):
                detections.append((d, r, float(power[d, r]), noise, float(threshold)))
    return detections, alpha


def run(input_h5: Path, geometry_csv: Path, output_h5: Path, *, wavelength_m: float = 299792458.0 / 77e9,
        training: tuple[int, int] = (4, 4), guard: tuple[int, int] = (1, 1), pfa: float = 1e-3,
        max_detections: int = 256) -> dict:
    with h5py.File(input_h5, "r") as handle:
        power = handle["/range_doppler/power_linear"][...]
        spectrum = handle["/range_doppler/spectrum_complex"][...]
        ranges = handle["/axes/range_m"][...]
        velocities = handle["/axes/velocity_mps"][...]
    x, y = geometry(geometry_csv, wavelength_m)
    candidates, alpha = ca_cfar(power, training=training, guard=guard, pfa=pfa)
    candidates = sorted(candidates, key=lambda item: item[2], reverse=True)[:max_detections]
    points = []
    for d, r, peak, noise, threshold in candidates:
        azimuth, elevation = estimate_aoa_from_positions(spectrum[d, r], None, x, y) if False else (None, None)
        # AoA estimator only needs wavelength/config; import lazily to keep the output contract explicit.
        from simulation.v03 import FmcwConfig
        config = FmcwConfig(samples_per_chirp=len(ranges), chirps_per_frame=len(velocities))
        azimuth, elevation = estimate_aoa_from_positions(spectrum[d, r], config, x, y)
        range_m, velocity_mps = float(ranges[r]), float(velocities[d])
        az_rad, el_rad = np.deg2rad(azimuth), np.deg2rad(elevation)
        points.append({"doppler_index": d, "range_index": r, "range_m": range_m,
                       "velocity_mps": velocity_mps, "azimuth_deg": azimuth,
                       "elevation_deg": elevation, "power_linear": peak,
                       "noise_linear": noise, "threshold_linear": threshold,
                       "x_m": range_m * np.cos(el_rad) * np.sin(az_rad),
                       "y_m": range_m * np.cos(el_rad) * np.cos(az_rad),
                       "z_m": range_m * np.sin(el_rad)})
    output_h5.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_h5, "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-cfar-point-cloud-v0.4.23"
        handle.attrs["pfa"] = pfa; handle.attrs["cfar_alpha"] = alpha
        handle.attrs["channel_order_verified"] = False
        if points:
            for key in points[0]:
                handle.create_dataset(f"/point_cloud/{key}", data=np.asarray([p[key] for p in points]))
        else:
            handle.create_dataset("/point_cloud/empty", data=np.empty((0,)))
    summary = {"input_h5": str(input_h5.resolve()), "training_cells": training, "guard_cells": guard,
               "pfa": pfa, "cfar_alpha": alpha, "candidate_count": len(candidates),
               "point_count": len(points), "channel_order_verified": False,
               "aoa_status": "cfar_peak_point_cloud_not_hardware_validated"}
    output_h5.with_suffix(".json").write_text(json.dumps({"summary": summary, "points": points}, indent=2), encoding="utf-8")
    output_h5.with_suffix(".md").write_text(
        "# V0.4.23 CA-CFAR 三维点云\n\n"
        f"检测点数：{len(points)}；Pfa：{pfa:g}；训练单元：{training}；保护单元：{guard}。\n\n"
        "每个点包含距离、速度、方位、俯仰、功率、噪声、阈值和 x/y/z。真实硬件通道顺序与 AoA 仍待验证。\n",
        encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-h5", type=Path, required=True)
    parser.add_argument("--geometry-csv", type=Path, required=True)
    parser.add_argument("--output-h5", type=Path, required=True)
    parser.add_argument("--pfa", type=float, default=1e-3)
    parser.add_argument("--max-detections", type=int, default=256)
    args = parser.parse_args()
    print(json.dumps(run(args.input_h5.resolve(), args.geometry_csv.resolve(), args.output_h5.resolve(), pfa=args.pfa,
                         max_detections=args.max_detections), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
