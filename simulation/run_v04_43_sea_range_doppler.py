"""Generate synthetic sea-clutter range-Doppler complex IQ from V0.2 facets."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_39_sparse_array_solver import estimate_sparse_grid
from simulation.run_v04_42_physical_sea_microfacets import derive_frame, load_surface
from simulation.v03 import FmcwConfig


def generate_radar_iq(config: FmcwConfig, facets: np.ndarray, positions: tuple[np.ndarray, np.ndarray], phase0: np.ndarray) -> np.ndarray:
    fast_time = np.arange(config.samples_per_chirp) / config.sample_rate_hz
    slow_time = np.arange(config.chirps_per_frame) * config.pulse_repetition_interval_s
    iq = np.zeros((config.chirps_per_frame, config.samples_per_chirp, 4, 4), dtype=complex)
    for index, facet in enumerate(facets):
        r = float(facet["slant_range_m"]); fd = float(facet["doppler_hz"]); amp = np.sqrt(max(float(facet["scatter_proxy"]), 0.0))
        beat = 2.0 * config.slope_hz_per_s * r / config.propagation_speed_mps
        baseband = amp * np.exp(1j * (phase0[index] + 2.0 * np.pi * (fd * slow_time[:, None] + beat * fast_time[None, :])))
        az = np.deg2rad(float(facet["azimuth_deg"])); el = np.deg2rad(float(facet["elevation_deg"]))
        steering = np.exp(1j * (2.0 * np.pi / config.wavelength_m) * (positions[0] * np.sin(az) * np.cos(el) + positions[1] * np.sin(el)))
        iq += baseband[:, :, None, None] * steering[None, None, :, :]
    return iq


def range_doppler(iq: np.ndarray, config: FmcwConfig) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    range_window = np.hanning(config.samples_per_chirp); doppler_window = np.hanning(config.chirps_per_frame)
    spectrum = np.fft.fft(iq * range_window[None, :, None, None], axis=1)
    spectrum = np.fft.fftshift(np.fft.fft(spectrum * doppler_window[:, None, None, None], axis=0), axes=0)
    positive = np.arange(config.samples_per_chirp // 2); spectrum = spectrum[:, positive, :, :]
    power = np.mean(np.abs(spectrum) ** 2, axis=(2, 3))
    range_axis = positive * config.sample_rate_hz / config.samples_per_chirp * config.propagation_speed_mps / (2.0 * config.slope_hz_per_s)
    velocity_axis = np.fft.fftshift(np.fft.fftfreq(config.chirps_per_frame, d=config.pulse_repetition_interval_s)) * config.wavelength_m / 2.0
    return spectrum, power, range_axis, velocity_axis, positive


def run_case(path: Path, output: Path, positions: tuple[np.ndarray, np.ndarray], config: FmcwConfig, rng: np.random.Generator) -> dict:
    surface = load_surface(path); previous = None; phase0 = rng.uniform(-np.pi, np.pi, 48); frame_power = []; peak_rows = []
    for frame, time_s in enumerate(surface["time_s"]):
        facets, physics = derive_frame(surface, frame, previous, rng, max_facets=24); previous = surface["height_m"][frame]
        iq = generate_radar_iq(config, facets, positions, phase0); spectrum, power, range_axis, velocity_axis, _ = range_doppler(iq, config)
        doppler_index, range_index = np.unravel_index(int(np.argmax(power)), power.shape); channel = spectrum[doppler_index, range_index]
        estimate = estimate_sparse_grid(config, channel, positions[0], positions[1], np.arange(-60.0, 60.01, 1.0), np.arange(-20.0, 20.01, 1.0))
        frame_power.append(power.astype(np.float32)); peak_rows.append({"case_id": surface["case_id"], "frame": frame, "time_s": float(time_s), "range_m": float(range_axis[range_index]), "velocity_mps": float(velocity_axis[doppler_index]), "power_linear": float(power[doppler_index, range_index]), "estimated_azimuth_deg": estimate[0], "estimated_elevation_deg": estimate[1], "peak_score": estimate[2], "doppler_min_hz": physics["doppler_min_hz"], "doppler_max_hz": physics["doppler_max_hz"]})
    case_id = surface["case_id"]; case_file = output / f"{case_id}_range_doppler.h5"
    with h5py.File(case_file, "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-synthetic-sea-range-doppler-v0.4.43"; handle.attrs["input_status"] = "v02_height_truth_derived_microfacets"; handle.attrs["channel_order_verified"] = False
        handle.create_dataset("/range_doppler/power_linear", data=np.stack(frame_power), compression="gzip"); handle.create_dataset("/axes/range_m", data=range_axis); handle.create_dataset("/axes/velocity_mps", data=velocity_axis)
        for key in ("range_m", "velocity_mps", "power_linear", "estimated_azimuth_deg", "estimated_elevation_deg", "peak_score"):
            handle.create_dataset(f"/peaks/{key}", data=np.asarray([row[key] for row in peak_rows]))
    return {"case_id": case_id, "frames": len(peak_rows), "mean_peak_range_m": float(np.mean([row["range_m"] for row in peak_rows])), "mean_peak_velocity_mps": float(np.mean([row["velocity_mps"] for row in peak_rows])), "peak_velocity_std_mps": float(np.std([row["velocity_mps"] for row in peak_rows])), "mean_peak_power_linear": float(np.mean([row["power_linear"] for row in peak_rows])), "mean_aoa_score": float(np.mean([row["peak_score"] for row in peak_rows])), "range_doppler_file": str(case_file.resolve())}, peak_rows


def run(input_root: Path, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True); config = FmcwConfig(samples_per_chirp=128, chirps_per_frame=64); models = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config); paths = sorted(input_root.glob("ss*_seed101.h5")); rng = np.random.default_rng(4343); summaries = []; peaks: list[dict] = []
    for path in paths:
        summary, rows = run_case(path, output, models["pcb_centroid_candidate"], config, rng); summaries.append(summary); peaks.extend(rows)
    with (output / "peak_frames.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(peaks[0])); writer.writeheader(); writer.writerows(peaks)
    with (output / "case_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    summary = {"status": "completed_synthetic_sea_range_doppler", "input_root": str(input_root.resolve()), "random_seed": 4343, "config": {"samples_per_chirp": config.samples_per_chirp, "chirps_per_frame": config.chirps_per_frame, "pri_s": config.pulse_repetition_interval_s, "range_resolution_m": config.range_resolution_m, "velocity_resolution_mps": config.velocity_resolution_mps}, "input_status": "v02_height_truth_derived_microfacets_not_measured_iq", "channel_order_verified": False, "hardware_aoa_validated": False, "cases": summaries}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.43 合成海杂波距离-多普勒复数回波", "", "本阶段将 V0.4.42 微元的距离、径向速度、Doppler、散射代理和 PCB 候选阵列导向矢量叠加为 4×4 复数 IQ，进行距离-多普勒 FFT，并在峰值单元上运行稀疏 AoA。", "", f"距离分辨率：{config.range_resolution_m:.6f} m；速度分辨率：{config.velocity_resolution_mps:.6f} m/s；每个海况 41 个海面帧，每帧合成 {config.chirps_per_frame} 个雷达 chirp。", "", "| 海况 | 平均峰值距离(m) | 平均峰值速度(m/s) | 峰值速度标准差(m/s) | 平均峰值功率 |", "|---|---:|---:|---:|---:"]
    lines.extend(f"| {row['case_id']} | {row['mean_peak_range_m']:.4f} | {row['mean_peak_velocity_mps']:.6f} | {row['peak_velocity_std_mps']:.6f} | {row['mean_peak_power_linear']:.6e} |" for row in summaries)
    lines += ["", "## 输出含义", "", "每个海况的 HDF5 保存完整的 /range_doppler/power_linear（帧、Doppler、距离）和 /peaks 元数据，可以继续接入 CA-CFAR；peak_frames.csv 保存每帧峰值距离、速度、AoA 和峰值相关性。", "", "## 边界", "", "这是由 V0.2 高度场推导微元属性后生成的合成复数 IQ，不是 DCA1000 或 AWR2944P 实测 ADC。散射、水平速度、通道校准、方向图、互耦和船体遮挡仍是代理或未建模；因此距离-多普勒和 AoA 结果用于链路验证，不能作为实板性能结论。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
