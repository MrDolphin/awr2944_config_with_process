"""Estimate AoA from a per-channel complex Range-Doppler bin."""

from __future__ import annotations

import argparse, json
from pathlib import Path
import h5py
import numpy as np

from simulation.run_v04_75_coordinate_transform_validation import _load_cfg_geometry
from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_positions


def channel_range_doppler(virtual: np.ndarray):
    """Return complex RD spectrum shaped (doppler, positive_range, rx, tx)."""
    chirps, samples = virtual.shape[:2]
    range_window = np.hanning(samples)
    doppler_window = np.hanning(chirps)
    spectrum = np.fft.fft(virtual * range_window[None, :, None, None], axis=1)
    spectrum = np.fft.fftshift(np.fft.fft(spectrum * doppler_window[:, None, None, None], axis=0), axes=0)
    return spectrum[:, :samples // 2]


def run(input_h5: Path, mapping_csv: Path, output: Path) -> dict:
    with h5py.File(input_h5, "r") as handle:
        virtual = handle["/radar/virtual_iq"][...]
    if virtual.ndim != 4 or virtual.shape[2:] != (4, 4):
        raise ValueError(f"expected virtual IQ (frame,sample,4,4), got {virtual.shape}")
    config = FmcwConfig(samples_per_chirp=virtual.shape[1], chirps_per_frame=virtual.shape[0])
    x, y = _load_cfg_geometry(mapping_csv, config)
    spectrum = channel_range_doppler(virtual)
    power = np.mean(np.abs(spectrum) ** 2, axis=(2, 3))
    doppler_index, range_index = np.unravel_index(int(np.argmax(power)), power.shape)
    channel = spectrum[doppler_index, range_index]
    azimuth, elevation = estimate_aoa_from_positions(channel, config, x, y)
    output.mkdir(parents=True, exist_ok=True)
    with h5py.File(output / "complex_bin_aoa.h5", "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-complex-rd-bin-aoa-v0.4.82"
        handle.attrs["input_h5"] = str(input_h5.resolve())
        handle.attrs["channel_order_verified"] = False
        handle.create_dataset("/range_doppler/spectrum_complex", data=spectrum, compression="gzip")
        handle.create_dataset("/range_doppler/power_linear", data=power)
        handle.create_dataset("/peaks/doppler_index", data=doppler_index)
        handle.create_dataset("/peaks/range_index", data=range_index)
        handle.create_dataset("/peaks/azimuth_deg", data=azimuth)
        handle.create_dataset("/peaks/elevation_deg", data=elevation)
    summary = {"status":"completed_complex_bin_aoa", "input_h5":str(input_h5.resolve()), "virtual_shape":list(virtual.shape), "spectrum_shape":list(spectrum.shape), "peak_doppler_index":int(doppler_index), "peak_range_index":int(range_index), "estimated_azimuth_deg":float(azimuth), "estimated_elevation_deg":float(elevation), "channel_order_verified":False, "hardware_validated":False, "aoa_status":"complex_range_doppler_bin_exploratory"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text("# V0.4.82 距离-多普勒复数 bin AoA\n\n"
        "本阶段不再对全部时域 IQ 做平均，而是对每个 RX/TX 通道分别进行距离和多普勒 FFT，在平均功率最高的距离-多普勒单元取 4×4 复数通道矩阵，再估计 AoA。\n\n"
        f"虚拟 IQ：`{virtual.shape}`；复数谱：`{spectrum.shape}`；峰值 bin：doppler={doppler_index}, range={range_index}；AoA：az={azimuth:.4f}°、el={elevation:.4f}°。\n\n"
        "当前仍使用候选阵列映射，通道顺序和相位中心未经过硬件验证；该输出可作为后续 CFAR/点云的复数输入。\n", encoding="utf-8")
    return summary


def main():
    p=argparse.ArgumentParser(); p.add_argument("--input-h5",type=Path,required=True); p.add_argument("--mapping",type=Path,required=True); p.add_argument("--output",type=Path,required=True); a=p.parse_args(); print(json.dumps(run(a.input_h5.resolve(),a.mapping.resolve(),a.output.resolve()),indent=2,ensure_ascii=False))


if __name__ == "__main__": main()
