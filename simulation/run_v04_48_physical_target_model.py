"""Physical-relative target echo model for sea-clutter detection experiments."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_39_sparse_array_solver import estimate_sparse_grid
from simulation.run_v04_46_target_in_sea_clutter import steering
from simulation.run_v04_cfar_point_cloud import ca_cfar
from simulation.v03 import FmcwConfig


def local_cfar_noise(power: np.ndarray, doppler_index: int, range_index: int, training=(2, 2), guard=(1, 1)) -> float:
    td, tr = training; gd, gr = guard
    d0, d1 = doppler_index - td - gd, doppler_index + td + gd + 1
    r0, r1 = range_index - tr - gr, range_index + tr + gr + 1
    outer = power[d0:d1, r0:r1]
    mask = np.ones_like(outer, dtype=bool)
    mask[td:td + 2 * gd + 1, tr:tr + 2 * gr + 1] = False
    values = outer[mask]
    values = values[np.isfinite(values) & (values > 0)]
    positive = power[np.isfinite(power) & (power > 0)]
    return float(np.mean(values)) if values.size else float(np.median(positive))


def target_amplitude(noise_power: float, target_snr_db: float, target_range_m: float, *, reference_range_m: float = 20.0, rcs_m2: float = 1.0, reference_rcs_m2: float = 1.0) -> float:
    """Return amplitude using a relative monostatic radar-equation scale.

    The absolute gain is calibrated to the local synthetic clutter noise at
    the reference range.  The relative target power then follows
    ``sigma/R^4``; this keeps the model dimensionally interpretable without
    pretending that the synthetic spectrum is calibrated ADC power.
    """
    if target_range_m <= 0.0 or rcs_m2 <= 0.0 or reference_rcs_m2 <= 0.0:
        raise ValueError("range and RCS must be positive")
    power = noise_power * 10.0 ** (target_snr_db / 10.0)
    power *= (rcs_m2 / reference_rcs_m2) * (reference_range_m / target_range_m) ** 4
    return float(np.sqrt(power))


def inject_physical_target(spectrum: np.ndarray, power: np.ndarray, frame: int, ranges: np.ndarray, velocities: np.ndarray, target_steering: np.ndarray, config: FmcwConfig, *, target_snr_db: float, target_range_m: float, target_velocity_mps: float, rcs_m2: float = 1.0, range_sigma_bins: float = 0.65, doppler_sigma_bins: float = 0.65) -> tuple[np.ndarray, np.ndarray, dict]:
    modified = spectrum.copy(); frame_power = power.copy()
    target_r = int(np.argmin(np.abs(ranges - target_range_m))); target_d = int(np.argmin(np.abs(velocities - target_velocity_mps)))
    noise_power = local_cfar_noise(power, target_d, target_r)
    amplitude = target_amplitude(noise_power, target_snr_db, float(ranges[target_r]), rcs_m2=rcs_m2)
    # A compact 2-D Gaussian models finite range/Doppler main-lobe width.
    d_indices = range(max(0, target_d - 3), min(len(velocities), target_d + 4))
    r_indices = range(max(0, target_r - 3), min(len(ranges), target_r + 4))
    for d in d_indices:
        for r in r_indices:
            kernel = np.exp(-0.5 * (((d - target_d) / doppler_sigma_bins) ** 2 + ((r - target_r) / range_sigma_bins) ** 2))
            phase = np.exp(1j * 2.0 * np.pi * (2.0 * target_velocity_mps / config.wavelength_m) * (frame * config.chirps_per_frame * config.pulse_repetition_interval_s))
            modified[frame, d, r] += amplitude * kernel * phase * target_steering
            frame_power[d, r] = float(np.mean(np.abs(modified[frame, d, r]) ** 2))
    return modified, frame_power, {"target_range_index": target_r, "target_doppler_index": target_d, "noise_power_linear": noise_power, "target_amplitude": amplitude}


def run_case(path: Path, scenario: dict, positions: tuple[np.ndarray, np.ndarray], config: FmcwConfig, target: dict) -> dict:
    with h5py.File(path, "r") as handle:
        spectrum = handle["/range_doppler/spectrum_complex"][...]
        power = handle["/range_doppler/power_linear"][...]
        ranges = handle["/axes/range_m"][...]; velocities = handle["/axes/velocity_mps"][...]
    target_s = steering(config, positions, target["azimuth_deg"], target["elevation_deg"]); rows = []
    for frame in range(power.shape[0]):
        modified, frame_power, meta = inject_physical_target(spectrum, power[frame], frame, ranges, velocities, target_s, config, target_snr_db=scenario["target_snr_db"], target_range_m=target["range_m"], target_velocity_mps=target["velocity_mps"], rcs_m2=target.get("rcs_m2", 1.0))
        found, alpha = ca_cfar(frame_power, training=scenario["training"], guard=scenario["guard"], pfa=scenario["pfa"]); found = sorted(found, key=lambda item: item[2], reverse=True)
        hits = [item for item in found if abs(item[0] - meta["target_doppler_index"]) <= 2 and abs(item[1] - meta["target_range_index"]) <= 2]; hit = hits[0] if hits else None; az_err = el_err = None
        if hit:
            estimate = estimate_sparse_grid(config, modified[frame, hit[0], hit[1]], positions[0], positions[1], np.arange(-60.0, 60.01, 1.0), np.arange(-20.0, 20.01, 1.0)); az_err = float(estimate[0] - target["azimuth_deg"]); el_err = float(estimate[1] - target["elevation_deg"])
        rows.append({"frame": frame, "target_hit": bool(hit), "false_alarm_count": len(found) - int(bool(hit)), "target_range_m": float(ranges[meta["target_range_index"]]), "target_velocity_mps": float(velocities[meta["target_doppler_index"]]), "target_azimuth_error_deg": az_err, "target_elevation_error_deg": el_err, "noise_power_linear": meta["noise_power_linear"], "target_amplitude": meta["target_amplitude"], "cfar_alpha": alpha})
    az = np.asarray([r["target_azimuth_error_deg"] for r in rows if r["target_azimuth_error_deg"] is not None]); el = np.asarray([r["target_elevation_error_deg"] for r in rows if r["target_elevation_error_deg"] is not None])
    return {"case_id": path.stem.replace("_range_doppler", ""), "scenario_id": scenario["scenario_id"], "target_snr_db": scenario["target_snr_db"], "pfa": scenario["pfa"], "target_range_truth_m": target["range_m"], "target_velocity_truth_mps": target["velocity_mps"], "target_rcs_m2": target.get("rcs_m2", 1.0), "frames": len(rows), "detection_probability": float(np.mean([r["target_hit"] for r in rows])), "mean_false_alarms_per_frame": float(np.mean([r["false_alarm_count"] for r in rows])), "target_azimuth_rmse_deg": float(np.sqrt(np.mean(az ** 2))) if len(az) else None, "target_elevation_rmse_deg": float(np.sqrt(np.mean(el ** 2))) if len(el) else None, "mean_target_amplitude": float(np.mean([r["target_amplitude"] for r in rows]))}


def run(input_root: Path, output: Path) -> dict:
    config = FmcwConfig(samples_per_chirp=128, chirps_per_frame=64); models = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config); cases = ("ss2_normal", "ss3_upper"); scenarios = [{"scenario_id": f"snr{snr:g}_pfa{pfa:g}", "target_snr_db": snr, "pfa": pfa, "training": (2, 2), "guard": (1, 1)} for snr in (0.0, 5.0, 10.0, 15.0) for pfa in (1e-2, 1e-3)]; targets = [{"target_id": "r08", "range_m": 8.0, "velocity_mps": 1.0, "azimuth_deg": 10.0, "elevation_deg": 2.0, "rcs_m2": 1.0}, {"target_id": "r20", "range_m": 20.0, "velocity_mps": 1.0, "azimuth_deg": 10.0, "elevation_deg": 2.0, "rcs_m2": 1.0}, {"target_id": "r40", "range_m": 40.0, "velocity_mps": 1.0, "azimuth_deg": 10.0, "elevation_deg": 2.0, "rcs_m2": 1.0}, {"target_id": "r60", "range_m": 60.0, "velocity_mps": 1.0, "azimuth_deg": 10.0, "elevation_deg": 2.0, "rcs_m2": 1.0}]; rows = []
    for case in cases:
        for target in targets:
            for scenario in scenarios:
                row = run_case(input_root / f"{case}_range_doppler.h5", scenario, models["pcb_centroid_candidate"], config, target); row.update({"target_id": target["target_id"], "case_id": case}); rows.append(row)
    output.mkdir(parents=True, exist_ok=True); fields = list(rows[0])
    with (output / "physical_target_sweep.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    summary = {"status": "completed_relative_physical_target_model", "row_count": len(rows), "input_status": "synthetic_sea_spectrum_not_measured_iq", "hardware_validated": False, "radar_equation_status": "relative_sigma_over_r4_only", "rows": rows}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.48 相对物理目标回波模型", "", "目标功率按相对单站雷达方程使用 `sigma/R^4` 缩放，目标回波以有限距离-Doppler 高斯主瓣叠加到原海杂波复数谱；参考绝对增益仍由合成谱局部噪声标定。", "", "| 海况 | 目标距离 | SNR | Pfa | 检测概率 | 虚警点/帧 | 平均目标幅度 |", "|---|---:|---:|---:|---:|---:|---:|"]
    lines.extend(f"| {r['case_id']} | {r['target_range_truth_m']:.0f} m | {r['target_snr_db']:.0f} dB | {r['pfa']:g} | {r['detection_probability']:.3f} | {r['mean_false_alarms_per_frame']:.3f} | {r['mean_target_amplitude']:.3e} |" for r in rows)
    lines += ["", "## 边界", "", "本阶段已加入距离相对衰减、RCS参数和局部距离-Doppler扩展，但仍不是绝对功率标定模型，也未加入真实目标RCS测量、天线增益、系统损耗、噪声系数、目标微动和实测IQ。因此检测概率用于模型趋势和接口验证，不作为实测探测距离结论。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
