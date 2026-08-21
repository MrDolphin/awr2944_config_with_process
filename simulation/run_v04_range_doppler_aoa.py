"""Process V0.4.21 HDF5 virtual IQ into range-Doppler and peak-cell AoA."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_positions


def geometry(path: Path, config: FmcwConfig):
    with path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    x = np.zeros((4, 4)); y = np.zeros((4, 4))
    for row in rows:
        rx, tx = int(row["rx_index"]), int(row["tx_index"])
        x[rx, tx] = float(row["column"]) * float(row["azimuth_spacing_lambda"]) * config.wavelength_m
        y[rx, tx] = float(row["row"]) * float(row["elevation_spacing_lambda"]) * config.wavelength_m
    return x, y


def process(virtual_iq: np.ndarray, config: FmcwConfig, x: np.ndarray, y: np.ndarray):
    if virtual_iq.ndim != 4 or virtual_iq.shape[2:] != (4, 4):
        raise ValueError("virtual IQ must have shape (frame, sample, 4, 4)")
    frames, samples = virtual_iq.shape[:2]
    range_window = np.hanning(samples)
    doppler_window = np.hanning(frames)
    spectrum = np.fft.fft(virtual_iq * range_window[None, :, None, None], axis=1)
    spectrum = np.fft.fftshift(np.fft.fft(spectrum * doppler_window[:, None, None, None], axis=0), axes=0)
    positive = np.arange(samples // 2)
    spectrum = spectrum[:, positive, :, :]
    power = np.mean(np.abs(spectrum) ** 2, axis=(2, 3))
    d_idx, r_idx = np.unravel_index(int(np.argmax(power)), power.shape)
    channel = spectrum[d_idx, r_idx]
    azimuth, elevation = estimate_aoa_from_positions(channel, config, x, y)
    range_axis = positive * config.sample_rate_hz / samples * config.propagation_speed_mps / (2.0 * config.slope_hz_per_s)
    velocity_axis = np.fft.fftshift(np.fft.fftfreq(frames, d=config.pulse_repetition_interval_s)) * config.wavelength_m / 2.0
    return {"spectrum": spectrum, "power": power, "range_axis_m": range_axis,
            "velocity_axis_mps": velocity_axis, "doppler_index": int(d_idx),
            "range_index": int(r_idx), "peak_range_m": float(range_axis[r_idx]),
            "peak_velocity_mps": float(velocity_axis[d_idx]), "peak_power_linear": float(power[d_idx, r_idx]),
            "estimated_azimuth_deg": float(azimuth), "estimated_elevation_deg": float(elevation)}


def run(input_h5: Path, geometry_csv: Path, output_h5: Path, *, use_calibrated: bool) -> dict:
    with h5py.File(input_h5, "r") as handle:
        dataset = "/calibrated/virtual_iq" if use_calibrated and "/calibrated/virtual_iq" in handle else "/recovered/virtual_iq"
        virtual = handle[dataset][...]
        source_attrs = {key: str(value) for key, value in handle.attrs.items()}
    config = FmcwConfig(samples_per_chirp=virtual.shape[1], chirps_per_frame=virtual.shape[0])
    x, y = geometry(geometry_csv, config)
    result = process(virtual, config, x, y)
    output_h5.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_h5, "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-range-doppler-aoa-v0.4.22"
        handle.attrs["input_dataset"] = dataset
        handle.attrs["channel_order_verified"] = False
        handle.create_dataset("/range_doppler/power_linear", data=result["power"], compression="gzip")
        handle.create_dataset("/range_doppler/spectrum_complex", data=result["spectrum"], compression="gzip")
        handle.create_dataset("/axes/range_m", data=result["range_axis_m"])
        handle.create_dataset("/axes/velocity_mps", data=result["velocity_axis_mps"])
        handle.create_dataset("/peak/channel_complex", data=result["spectrum"][result["doppler_index"], result["range_index"]])
    summary = {key: value for key, value in result.items() if key not in {"spectrum", "power", "range_axis_m", "velocity_axis_mps"}}
    summary.update({"input_h5": str(input_h5.resolve()), "input_dataset": dataset,
                    "input_attrs": source_attrs, "virtual_shape": list(virtual.shape),
                    "channel_order_verified": False, "aoa_status": "peak_cell_aoa_not_hardware_validated"})
    output_h5.with_suffix(".json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    output_h5.with_suffix(".md").write_text(
        "# V0.4.22 距离-多普勒-峰值单元 AoA\n\n"
        f"输入数据集：`{dataset}`；峰值距离 {result['peak_range_m']:.6f} m；峰值速度 {result['peak_velocity_mps']:.6f} m/s；"
        f"估计方位 {result['estimated_azimuth_deg']:.6f}°；估计俯仰 {result['estimated_elevation_deg']:.6f}°。\n\n"
        "峰值单元 AoA 仍受通道顺序、阵列坐标和校准状态限制，不能直接作为实板精度。\n",
        encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-h5", type=Path, required=True)
    parser.add_argument("--geometry-csv", type=Path, required=True)
    parser.add_argument("--output-h5", type=Path, required=True)
    parser.add_argument("--use-calibrated", action="store_true")
    args = parser.parse_args()
    print(json.dumps(run(args.input_h5.resolve(), args.geometry_csv.resolve(), args.output_h5.resolve(),
                         use_calibrated=args.use_calibrated), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
