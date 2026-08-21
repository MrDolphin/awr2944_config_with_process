"""Inject a known target into synthetic sea spectra and measure detection/AoA."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_39_sparse_array_solver import estimate_sparse_grid
from simulation.run_v04_cfar_point_cloud import ca_cfar
from simulation.v03 import FmcwConfig


SCENARIOS = [
    {"scenario_id": "snr10_pfa1e2", "target_snr_db": 10.0, "pfa": 1e-2, "training": (2, 2), "guard": (1, 1)},
    {"scenario_id": "snr10_pfa1e3", "target_snr_db": 10.0, "pfa": 1e-3, "training": (2, 2), "guard": (1, 1)},
    {"scenario_id": "snr15_pfa1e2", "target_snr_db": 15.0, "pfa": 1e-2, "training": (2, 2), "guard": (1, 1)},
    {"scenario_id": "snr15_pfa1e3", "target_snr_db": 15.0, "pfa": 1e-3, "training": (2, 2), "guard": (1, 1)},
    {"scenario_id": "snr20_pfa1e2", "target_snr_db": 20.0, "pfa": 1e-2, "training": (2, 2), "guard": (1, 1)},
    {"scenario_id": "snr20_pfa1e3", "target_snr_db": 20.0, "pfa": 1e-3, "training": (2, 2), "guard": (1, 1)},
]


def steering(config: FmcwConfig, positions: tuple[np.ndarray, np.ndarray], azimuth_deg: float, elevation_deg: float) -> np.ndarray:
    az = np.deg2rad(azimuth_deg); el = np.deg2rad(elevation_deg)
    return np.exp(1j * (2.0 * np.pi / config.wavelength_m) * (positions[0] * np.sin(az) * np.cos(el) + positions[1] * np.sin(el)))


def inject_target(spectrum: np.ndarray, power: np.ndarray, frame: int, doppler_index: int, range_index: int, target_steering: np.ndarray, target_snr_db: float) -> tuple[np.ndarray, np.ndarray, float]:
    modified = spectrum.copy(); frame_power = power.copy()
    # Estimate noise with the same 2-D CA-CFAR training window used below.
    # The previous slice-relative masking could mask the whole window and
    # produced an all-NaN median, making the injected target weaker than the
    # local CFAR threshold.
    tr_d = tr_r = 2; gd = gr = 1
    d0, d1 = doppler_index - tr_d - gd, doppler_index + tr_d + gd + 1
    r0, r1 = range_index - tr_r - gr, range_index + tr_r + gr + 1
    outer = frame_power[d0:d1, r0:r1].copy()
    mask = np.ones_like(outer, dtype=bool)
    mask[tr_d:tr_d + 2 * gd + 1, tr_r:tr_r + 2 * gr + 1] = False
    training_cells = outer[mask]
    finite = training_cells[np.isfinite(training_cells) & (training_cells > 0)]
    noise_power = float(np.mean(finite)) if finite.size else float(np.median(frame_power[frame_power > 0]))
    alpha_count = (2 * (tr_d + gd) + 1) * (2 * (tr_r + gr) + 1) - (2 * gd + 1) * (2 * gr + 1)
    cfar_alpha = alpha_count * ((1e-2) ** (-1.0 / alpha_count) - 1.0)
    amplitude = np.sqrt(noise_power * 10.0 ** (target_snr_db / 10.0))
    # Replace the selected cell with a controlled coherent target return. This
    # makes the requested SNR reproducible and isolates CFAR/AoA behaviour;
    # surrounding cells remain the original synthetic sea clutter.
    modified[frame, doppler_index, range_index] = amplitude * target_steering
    frame_power[doppler_index, range_index] = float(np.mean(np.abs(modified[frame, doppler_index, range_index]) ** 2))
    return modified, frame_power, noise_power


def run_case(path: Path, scenario: dict, positions: tuple[np.ndarray, np.ndarray], config: FmcwConfig, target_az: float, target_el: float, target_range: float, target_velocity: float) -> dict:
    with h5py.File(path, "r") as handle:
        spectrum = handle["/range_doppler/spectrum_complex"][...]
        power = handle["/range_doppler/power_linear"][...]
        ranges = handle["/axes/range_m"][...]; velocities = handle["/axes/velocity_mps"][...]
    target_r = int(np.argmin(np.abs(ranges - target_range))); target_d = int(np.argmin(np.abs(velocities - target_velocity))); target_s = steering(config, positions, target_az, target_el)
    frame_rows = []
    for frame in range(power.shape[0]):
        modified, frame_power, noise_power = inject_target(spectrum, power[frame], frame, target_d, target_r, target_s, scenario["target_snr_db"])
        found, alpha = ca_cfar(frame_power, training=scenario["training"], guard=scenario["guard"], pfa=scenario["pfa"])
        found = sorted(found, key=lambda item: item[2], reverse=True)
        target_hits = [item for item in found if abs(item[0] - target_d) <= 1 and abs(item[1] - target_r) <= 1]
        target_hit = target_hits[0] if target_hits else None
        az_err = el_err = None
        if target_hit:
            channel = modified[frame, target_hit[0], target_hit[1]]
            estimate = estimate_sparse_grid(config, channel, positions[0], positions[1], np.arange(-60.0, 60.01, 1.0), np.arange(-20.0, 20.01, 1.0))
            az_err = estimate[0] - target_az; el_err = estimate[1] - target_el
        frame_rows.append({"case_id": path.stem.replace("_range_doppler", ""), "scenario_id": scenario["scenario_id"], "frame": frame, "target_range_m": float(ranges[target_r]), "target_velocity_mps": float(velocities[target_d]), "target_snr_db": scenario["target_snr_db"], "target_hit": bool(target_hit), "detection_count": len(found), "false_alarm_count": len(found) - int(bool(target_hit)), "target_azimuth_error_deg": az_err, "target_elevation_error_deg": el_err, "noise_power_linear": noise_power, "cfar_alpha": alpha})
    hits = np.asarray([row["target_hit"] for row in frame_rows], dtype=bool); az = np.asarray([row["target_azimuth_error_deg"] for row in frame_rows if row["target_azimuth_error_deg"] is not None]); el = np.asarray([row["target_elevation_error_deg"] for row in frame_rows if row["target_elevation_error_deg"] is not None]);
    return {"case_id": frame_rows[0]["case_id"], "scenario_id": scenario["scenario_id"], "target_snr_db": scenario["target_snr_db"], "pfa": scenario["pfa"], "frames": len(frame_rows), "target_range_m": float(ranges[target_r]), "target_velocity_mps": float(velocities[target_d]), "detection_probability": float(np.mean(hits)), "mean_false_alarms_per_frame": float(np.mean([row["false_alarm_count"] for row in frame_rows])), "target_azimuth_rmse_deg": float(np.sqrt(np.mean(az ** 2))) if len(az) else None, "target_elevation_rmse_deg": float(np.sqrt(np.mean(el ** 2))) if len(el) else None}


def run(input_root: Path, output: Path) -> dict:
    config = FmcwConfig(samples_per_chirp=128, chirps_per_frame=64); models = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config); paths = sorted(input_root.glob("*_range_doppler.h5")); target = {"az": 10.0, "el": 2.0, "range": 20.0, "velocity": 1.0}; rows = []
    for path in paths:
        for scenario in SCENARIOS:
            rows.append(run_case(path, scenario, models["pcb_centroid_candidate"], config, target["az"], target["el"], target["range"], target["velocity"]))
    output.mkdir(parents=True, exist_ok=True)
    with (output / "target_detection_sweep.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summary = {"status": "completed_target_in_synthetic_sea_clutter", "input_root": str(input_root.resolve()), "target": target, "scenario_count": len(SCENARIOS), "case_count": len(paths), "input_status": "synthetic_sea_spectrum_not_measured_iq", "hardware_validated": False, "rows": rows}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.46 海杂波背景下已知目标检测", "", f"注入目标：方位 {target['az']:.1f}°、俯仰 {target['el']:.1f}°、距离约 {target['range']:.1f} m、速度约 {target['velocity']:.1f} m/s。扫描目标 SNR=10/15/20 dB 和 Pfa=1e-2/1e-3。", "", "| 海况 | 场景 | 目标 SNR | 检测概率 | 平均虚警点/帧 | 方位 RMSE | 俯仰 RMSE |", "|---|---|---:|---:|---:|---:|---:|"]
    lines.extend(f"| {row['case_id']} | {row['scenario_id']} | {row['target_snr_db']:.0f} dB | {row['detection_probability']:.4f} | {row['mean_false_alarms_per_frame']:.3f} | {row['target_azimuth_rmse_deg'] if row['target_azimuth_rmse_deg'] is not None else float('nan'):.4f}° | {row['target_elevation_rmse_deg'] if row['target_elevation_rmse_deg'] is not None else float('nan'):.4f}° |" for row in rows)
    lines += ["", "## 指标解释", "", "检测概率定义为 41 帧中目标所在距离-Doppler 单元附近至少有一个 CFAR 检测点的比例；虚警点为同一帧中除目标命中外的其他检测点；AoA RMSE 只在目标命中的帧上计算。", "", "## 边界", "", "目标和海杂波都来自合成谱，PCB 阵列是铜区质心候选，CFAR/AoA 不是 TI SDK 默认实现。结果用于检测概率和 AoA 指标链路回归，不能作为实测探测概率、虚警率或雷达性能结论。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
