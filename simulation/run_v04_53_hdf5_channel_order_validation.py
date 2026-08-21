"""Rank RX/TX order candidates using a virtual-IQ HDF5 known-angle capture."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_39_sparse_array_solver import estimate_sparse_grid
from simulation.run_v04_46_target_in_sea_clutter import steering
from simulation.v03 import FmcwConfig
from simulation.v04 import virtual_array_positions


def load_virtual_iq(path: Path) -> tuple[np.ndarray, dict]:
    with h5py.File(path, "r") as handle:
        dataset = "/virtual/iq" if "/virtual/iq" in handle else "/recovered/virtual_iq" if "/recovered/virtual_iq" in handle else "/decoded/iq" if "/decoded/iq" in handle else None
        if dataset is None:
            raise ValueError("HDF5 must contain /virtual/iq, /recovered/virtual_iq or /decoded/iq")
        iq = handle[dataset][...]
        attrs = {key: handle.attrs[key].item() if hasattr(handle.attrs[key], "item") else handle.attrs[key] for key in handle.attrs}
    if iq.ndim != 4 or iq.shape[2:] != (4, 4):
        raise ValueError(f"virtual IQ must have shape (frame, sample, 4, 4), got {iq.shape}")
    return np.asarray(iq), {"dataset": dataset, **attrs}


def extract_peak_channel(iq: np.ndarray, config: FmcwConfig) -> tuple[np.ndarray, float, float, float]:
    range_window = np.hanning(iq.shape[1]); doppler_window = np.hanning(iq.shape[0]); spectrum = np.fft.fft(iq * range_window[None, :, None, None], axis=1); spectrum = np.fft.fftshift(np.fft.fft(spectrum * doppler_window[:, None, None, None], axis=0), axes=0); positive = np.arange(iq.shape[1] // 2); spectrum = spectrum[:, positive, :, :]; power = np.mean(np.abs(spectrum) ** 2, axis=(2, 3)); d, r = np.unravel_index(int(np.argmax(power)), power.shape); ranges = positive * config.sample_rate_hz / iq.shape[1] * config.propagation_speed_mps / (2.0 * config.slope_hz_per_s); velocities = np.fft.fftshift(np.fft.fftfreq(iq.shape[0], d=config.pulse_repetition_interval_s)) * config.wavelength_m / 2.0; return spectrum[d, r], float(ranges[r]), float(velocities[d]), float(power[d, r])


def validate(input_path: Path, mapping_path: Path | None, output: Path) -> dict:
    iq, metadata = load_virtual_iq(input_path); config = FmcwConfig(samples_per_chirp=iq.shape[1], chirps_per_frame=iq.shape[0]); x = np.zeros((4, 4)); y = np.zeros((4, 4)); geometry_source = "pcb_mapping"
    if mapping_path is None:
        if str(metadata.get("geometry_source", "")).startswith("simulation.v04") or metadata.get("source_type") in {"synthetic_known_angle_regression_only", "synthetic"}:
            x, y = virtual_array_positions(config); geometry_source = "simulation.v04.virtual_array_positions"
        else:
            raise ValueError("mapping is required unless HDF5 declares simulation.v04 geometry")
    else:
        with mapping_path.open(encoding="utf-8", newline="") as handle:
            for row in csv.DictReader(handle):
                rx, tx = int(row["rx_index"]), int(row["tx_index"]); x[rx, tx] = float(row["column"]) * float(row["azimuth_spacing_lambda"]) * config.wavelength_m; y[rx, tx] = float(row["row"]) * float(row["elevation_spacing_lambda"]) * config.wavelength_m
    channel, peak_range, peak_velocity, peak_power = extract_peak_channel(iq, config); truth_az = float(metadata.get("truth_azimuth_deg", 0.0)); truth_el = float(metadata.get("truth_elevation_deg", 0.0)); rows = []
    for rx_order in itertools.permutations(range(4)):
        for tx_order in itertools.permutations(range(4)):
            observed = channel[np.ix_(rx_order, tx_order)]; estimate = estimate_sparse_grid(config, observed, x, y, np.arange(-60.0, 60.01, 1.0), np.arange(-20.0, 20.01, 1.0)); rows.append({"rx_order": ",".join(map(str, rx_order)), "tx_order": ",".join(map(str, tx_order)), "estimated_azimuth_deg": estimate[0], "estimated_elevation_deg": estimate[1], "azimuth_error_deg": estimate[0] - truth_az, "elevation_error_deg": estimate[1] - truth_el, "score": estimate[2], "identity_order": rx_order == (0, 1, 2, 3) and tx_order == (0, 1, 2, 3)})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "hdf5_channel_order_candidates.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    ranked = sorted(rows, key=lambda row: abs(row["azimuth_error_deg"]) + abs(row["elevation_error_deg"]))
    summary = {"status": "completed_hdf5_known_angle_channel_order_validation", "input": str(input_path.resolve()), "input_metadata": metadata, "geometry_source": geometry_source, "peak_range_m": peak_range, "peak_velocity_mps": peak_velocity, "peak_power_linear": peak_power, "truth_azimuth_deg": truth_az, "truth_elevation_deg": truth_el, "best_candidates": ranked[:10], "identity_candidate": next(row for row in rows if row["identity_order"]), "source_is_hardware_measurement": metadata.get("source_type") not in {"synthetic_known_angle_regression_only", "synthetic"}, "channel_order_verified": False, "candidate_count": len(rows)}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.53 HDF5 已知角度通道顺序验证", "", f"输入数据集：`{metadata['dataset']}`；输入来源：`{metadata.get('source_type', 'unknown')}`；峰值距离：{peak_range:.6f} m；峰值速度：{peak_velocity:.6f} m/s。", "", "## 分析方法", "", "读取虚拟 IQ 后进行距离-Doppler FFT，在最大峰值单元取 4×4 复数通道；枚举 24×24 个 RX/TX 排列，用 PCB 候选几何进行稀疏导向矢量 AoA；按已知角度真值计算方位/俯仰误差。", "", f"- 真值：方位 {truth_az:.1f}°、俯仰 {truth_el:.1f}°", f"- 候选数：{len(rows)}", f"- 最佳候选误差和：{abs(ranked[0]['azimuth_error_deg']) + abs(ranked[0]['elevation_error_deg']):.3f}°", f"- 身份排列估计：方位 {summary['identity_candidate']['estimated_azimuth_deg']:.1f}°、俯仰 {summary['identity_candidate']['estimated_elevation_deg']:.1f}°", "", "## 最佳候选", "", "| RX 顺序 | TX 顺序 | 估计方位 | 估计俯仰 | 方位误差 | 俯仰误差 |", "|---|---|---:|---:|---:|---:|"]
    lines.extend(f"| {r['rx_order']} | {r['tx_order']} | {r['estimated_azimuth_deg']:.1f}° | {r['estimated_elevation_deg']:.1f}° | {r['azimuth_error_deg']:.1f}° | {r['elevation_error_deg']:.1f}° |" for r in ranked[:10])
    lines += ["", "## 边界", "", "输入 known-angle fixture 若标记为 synthetic，只能证明 HDF5 接口和排列扫描工作，不能证明真实硬件通道顺序。真实 DCA1000 数据必须配合 CFG、已知角反射器和实测校准 provenance。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input", type=Path, required=True); parser.add_argument("--mapping", type=Path); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(validate(args.input.resolve(), args.mapping.resolve() if args.mapping else None, args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
