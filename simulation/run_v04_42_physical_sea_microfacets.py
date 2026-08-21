"""Derive physical proxy attributes from V0.2 sea-surface height truth.

The input is still a height field, not measured radar IQ.  This stage derives
surface normals, finite-difference velocity, radial Doppler and an explicit
front-face scattering proxy before feeding the microfacets to AoA.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_39_sparse_array_solver import steering_matrix
from simulation.run_v04_40_calibration_noise_peaks import apply_impairments, estimate_with_peaks
from simulation.v03 import FmcwConfig


def load_surface(path: Path) -> dict:
    with h5py.File(path, "r") as handle:
        return {"time_s": handle["/axes/time_s"][...], "x_m": handle["/axes/x_m"][...], "y_m": handle["/axes/y_m"][...], "height_m": handle["/truth/height_m"][...], "height_m_install": float(np.asarray(handle["/installation/height_m"][...]).reshape(-1)[0]), "case_id": str(np.asarray(handle.attrs["case_id"]).reshape(-1)[0]), "wind_speed_mps": float(np.asarray(handle["/case/wind_speed_mps"][...]).reshape(-1)[0]), "wind_direction_deg": float(np.asarray(handle["/case/wind_direction_deg"][...]).reshape(-1)[0])}


def derive_frame(surface: dict, frame: int, previous_frame: np.ndarray | None, rng: np.random.Generator, max_facets: int = 48) -> tuple[np.ndarray, dict]:
    height = surface["height_m"][frame]
    x_axis, y_axis = surface["x_m"], surface["y_m"]
    dx = float(np.mean(np.diff(x_axis))); dy = float(np.mean(np.diff(y_axis)))
    dz_dy, dz_dx = np.gradient(height, dy, dx)
    normal = np.stack((-dz_dx, -dz_dy, np.ones_like(height)), axis=-1)
    normal /= np.maximum(np.linalg.norm(normal, axis=-1, keepdims=True), 1e-12)
    if previous_frame is None:
        velocity_z = np.zeros_like(height)
    else:
        dt = float(np.median(np.diff(surface["time_s"])))
        velocity_z = (height - previous_frame) / max(dt, 1e-9)
    yy, xx = np.meshgrid(y_axis, x_axis, indexing="ij")
    radar_height = surface["height_m_install"]
    rx = -xx; ry = -yy; rz = radar_height - height
    slant = np.sqrt(rx ** 2 + ry ** 2 + rz ** 2)
    los = np.stack((rx, ry, rz), axis=-1) / np.maximum(slant[..., None], 1e-12)
    incidence = np.sum(normal * los, axis=-1)
    radial_velocity = velocity_z * los[..., 2]
    wavelength = 299792458.0 / 77e9
    doppler_hz = 2.0 * radial_velocity / wavelength
    scatter_proxy = np.maximum(incidence, 0.0) ** 2 / np.maximum(slant, 1.0) ** 2
    flat_power = scatter_proxy.ravel()
    count = min(max_facets, flat_power.size)
    selected = np.argpartition(flat_power, -count)[-count:]
    selected = selected[np.argsort(flat_power[selected])[::-1]]
    rows = []
    for index in selected:
        iy, ix = np.unravel_index(int(index), height.shape)
        az = float(np.rad2deg(np.arctan2(xx[iy, ix], yy[iy, ix])))
        el = float(np.rad2deg(np.arctan2(height[iy, ix] - radar_height, np.hypot(xx[iy, ix], yy[iy, ix]))))
        rows.append({"x_m": float(xx[iy, ix]), "y_m": float(yy[iy, ix]), "z_m": float(height[iy, ix]), "azimuth_deg": az, "elevation_deg": el, "normal_x": float(normal[iy, ix, 0]), "normal_y": float(normal[iy, ix, 1]), "normal_z": float(normal[iy, ix, 2]), "incidence_cos": float(incidence[iy, ix]), "slant_range_m": float(slant[iy, ix]), "vertical_velocity_mps": float(velocity_z[iy, ix]), "radial_velocity_mps": float(radial_velocity[iy, ix]), "doppler_hz": float(doppler_hz[iy, ix]), "scatter_proxy": float(scatter_proxy[iy, ix])})
    return np.asarray(rows, dtype=object), {"grid_dx_m": dx, "grid_dy_m": dy, "mean_abs_radial_velocity_mps": float(np.mean(np.abs(radial_velocity))), "rms_radial_velocity_mps": float(np.sqrt(np.mean(radial_velocity ** 2))), "doppler_min_hz": float(np.min(doppler_hz)), "doppler_max_hz": float(np.max(doppler_hz)), "illuminated_fraction": float(np.mean(incidence > 0.0)), "selected_facet_count": count}


def microfacet_channel(config: FmcwConfig, facets: np.ndarray, positions: tuple[np.ndarray, np.ndarray], time_s: float, phase0: np.ndarray) -> np.ndarray:
    x_pos, y_pos = positions
    channel = np.zeros((4, 4), dtype=complex)
    for index, facet in enumerate(facets):
        az, el, amplitude, doppler = float(facet["azimuth_deg"]), float(facet["elevation_deg"]), float(np.sqrt(max(facet["scatter_proxy"], 0.0))), float(facet["doppler_hz"])
        steering = np.exp(1j * (2.0 * np.pi / config.wavelength_m) * (x_pos * np.sin(np.deg2rad(az)) * np.cos(np.deg2rad(el)) + y_pos * np.sin(np.deg2rad(el))))
        channel += amplitude * np.exp(1j * (phase0[index] + 2.0 * np.pi * doppler * time_s)) * steering
    return channel


def run(input_root: Path, output: Path) -> dict:
    config = FmcwConfig(); models = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config); positions = models["pcb_centroid_candidate"]
    solver_az = np.arange(-60.0, 60.01, 1.0); solver_el = np.arange(-20.0, 20.01, 1.0)
    paths = sorted(input_root.glob("ss*_seed101.h5")); rng = np.random.default_rng(4242)
    frame_rows: list[dict] = []; facet_rows: list[dict] = []; case_summaries: list[dict] = []
    for path in paths:
        surface = load_surface(path); previous = None; phase0 = rng.uniform(-np.pi, np.pi, 48); estimates = []
        for frame, time_s in enumerate(surface["time_s"]):
            facets, physics = derive_frame(surface, frame, previous, rng); previous = surface["height_m"][frame]
            channel = microfacet_channel(config, facets, positions, float(time_s), phase0)
            observed = apply_impairments(channel, rng, gain_std_db=1.0, phase_std_deg=5.0, snr_db=20.0)
            estimate = estimate_with_peaks(config, observed, positions[0], positions[1], solver_az, solver_el)
            estimates.append((estimate["estimated_azimuth_deg"], estimate["estimated_elevation_deg"]))
            frame_rows.append({"case_id": surface["case_id"], "frame": frame, "time_s": float(time_s), "wind_speed_mps": surface["wind_speed_mps"], "wind_direction_deg": surface["wind_direction_deg"], "estimated_azimuth_deg": estimate["estimated_azimuth_deg"], "estimated_elevation_deg": estimate["estimated_elevation_deg"], "peak_to_second_db": estimate["peak_to_second_db"], **physics})
            for facet_index, facet in enumerate(facets):
                facet_rows.append({"case_id": surface["case_id"], "frame": frame, "time_s": float(time_s), "facet_index": facet_index, **{key: facet[key] for key in ("x_m", "y_m", "z_m", "azimuth_deg", "elevation_deg", "normal_x", "normal_y", "normal_z", "incidence_cos", "slant_range_m", "vertical_velocity_mps", "radial_velocity_mps", "doppler_hz", "scatter_proxy")}})
        angles = np.asarray(estimates); jumps = np.sqrt(np.sum(np.diff(angles, axis=0) ** 2, axis=1))
        case_summaries.append({"case_id": surface["case_id"], "frames": len(estimates), "wind_speed_mps": surface["wind_speed_mps"], "wind_direction_deg": surface["wind_direction_deg"], "mean_abs_radial_velocity_mps": float(np.mean([row["mean_abs_radial_velocity_mps"] for row in frame_rows if row["case_id"] == surface["case_id"]])), "rms_radial_velocity_mps": float(np.sqrt(np.mean([row["rms_radial_velocity_mps"] ** 2 for row in frame_rows if row["case_id"] == surface["case_id"]]))), "doppler_min_hz": float(np.min([row["doppler_min_hz"] for row in frame_rows if row["case_id"] == surface["case_id"]])), "doppler_max_hz": float(np.max([row["doppler_max_hz"] for row in frame_rows if row["case_id"] == surface["case_id"]])), "mean_peak_to_second_db": float(np.mean([row["peak_to_second_db"] for row in frame_rows if row["case_id"] == surface["case_id"]])), "jump_rate_over_5deg": float(np.mean(jumps > 5.0)) if len(jumps) else 0.0})
    output.mkdir(parents=True, exist_ok=True)
    for filename, rows in (("microfacet_frames.csv", frame_rows), ("microfacet_attributes.csv", facet_rows), ("case_summary.csv", case_summaries)):
        with (output / filename).open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summary = {"status": "completed_physical_proxy_sea_microfacets", "input_root": str(input_root.resolve()), "random_seed": 4242, "wavelength_m": 299792458.0 / 77e9, "model": "pcb_centroid_candidate", "input_status": "v02_height_truth_with_derived_velocity_normal_doppler", "scattering_status": "front_face_incidence_over_range_squared_proxy", "hardware_aoa_validated": False, "cases": case_summaries}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.42 海面微元法向、速度和 Doppler 代理", "", "本阶段从 V0.2 高度场 H(x,y,t) 推导空间梯度、海面法向、时间差分速度、径向速度和双程 Doppler，并使用入射角/距离平方的前向散射代理生成复数微元叠加。", "", "| 海况 | RMS 径向速度(m/s) | Doppler 范围(Hz) | 平均主次峰差(dB) | >5° 跳峰率 |", "|---|---:|---:|---:|---:|"]
    lines.extend(f"| {row['case_id']} | {row['rms_radial_velocity_mps']:.6f} | {row['doppler_min_hz']:.2f}～{row['doppler_max_hz']:.2f} | {row['mean_peak_to_second_db']:.4f} | {row['jump_rate_over_5deg']:.4f} |" for row in case_summaries)
    lines += ["", "## 物理量含义", "", "法向由 z=f(x,y) 的梯度得到；垂向速度由相邻时间帧的高度差分得到；径向速度是垂向速度在雷达到海面视线方向上的投影；Doppler 使用 2*v_r/λ；散射代理取正面入射余弦平方再除以距离平方。", "", "## 边界", "", "V0.2 没有实测速度、波面法向校准、散射系数或复数 IQ，因此速度、Doppler 和散射仍是从高度场推导的代理，不是实测海杂波模型。还未加入水平流速、真实海面谱、极化、粗糙度、互耦和船体遮挡。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
