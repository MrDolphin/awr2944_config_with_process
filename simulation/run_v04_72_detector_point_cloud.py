"""Project V0.4.71 detector cells into bounded AoA point-cloud CSVs."""

from __future__ import annotations

import argparse, csv, json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_45_cfar_sweep import SCENARIOS
from simulation.run_v04_69_cfar_cell_evidence import _cell_evidence
from simulation.run_v04_71_detector_comparison import _local_max, _os_threshold, _valid_coords
from simulation.run_v04_cfar_point_cloud import geometry
from simulation.v04 import estimate_aoa_from_positions
from simulation.v03 import FmcwConfig

DETECTORS = ("ca_cfar_local_peak", "ca_threshold_only", "os_cfar_local_peak", "fixed_energy_local_peak")


def _retain(detector, plane, d, r, scenario, fixed_threshold):
    cell = float(plane[d, r]); local = _local_max(plane, d, r)
    if detector == "ca_cfar_local_peak":
        evidence = _cell_evidence(plane, d, r, scenario)
        return bool(evidence["would_detect"]), float(evidence["noise_linear"]), float(evidence["threshold_linear"])
    if detector == "ca_threshold_only":
        evidence = _cell_evidence(plane, d, r, scenario)
        return bool(evidence["above_threshold"]), float(evidence["noise_linear"]), float(evidence["threshold_linear"])
    threshold = _os_threshold(plane, d, r, scenario) if detector == "os_cfar_local_peak" else fixed_threshold
    return bool(cell > threshold and local), float("nan"), float(threshold)


def run(input_root: Path, geometry_csv: Path, output: Path, max_points_per_group: int = 256) -> dict:
    rows, summaries = [], []
    nonfinite_aoa_count = 0
    wavelength = 299792458.0 / 77e9
    x, y = geometry(geometry_csv, wavelength)
    for path in sorted(input_root.glob("*_range_doppler.h5")):
        case_id = path.stem.replace("_range_doppler", "")
        with h5py.File(path, "r") as h:
            power = h["/range_doppler/power_linear"][...]
            spectrum = h["/range_doppler/spectrum_complex"][...]
            ranges = h["/axes/range_m"][...]
            velocities = h["/axes/velocity_mps"][...]
        config = FmcwConfig(samples_per_chirp=len(ranges), chirps_per_frame=len(velocities))
        fixed_threshold = float(np.percentile(power.reshape(-1), 99.0))
        for scenario in SCENARIOS:
            for detector in DETECTORS:
                candidates, total_valid = [], 0
                for frame in range(power.shape[0]):
                    plane = power[frame]
                    for d, r in _valid_coords(plane, scenario):
                        total_valid += 1
                        keep, noise, threshold = _retain(detector, plane, d, r, scenario, fixed_threshold)
                        if keep:
                            candidates.append((float(plane[d, r]), frame, d, r, noise, threshold))
                candidates.sort(reverse=True)
                stored = candidates[:max_points_per_group]
                for peak, frame, d, r, noise, threshold in stored:
                    with np.errstate(invalid="ignore", divide="ignore"):
                        az, el = estimate_aoa_from_positions(spectrum[frame, d, r], config, x, y)
                    if not np.isfinite(az) or not np.isfinite(el):
                        nonfinite_aoa_count += 1
                    rr, vv = float(ranges[r]), float(velocities[d])
                    azr, elr = np.deg2rad(az), np.deg2rad(el)
                    rows.append({"case_id": case_id, "scenario_id": scenario["scenario_id"], "detector": detector,
                                 "frame": frame, "doppler_index": d, "range_index": r, "range_m": rr,
                                 "velocity_mps": vv, "azimuth_deg": float(az), "elevation_deg": float(el),
                                 "power_linear": peak, "noise_linear": noise, "threshold_linear": threshold,
                                 "x_m": rr*np.cos(elr)*np.sin(azr), "y_m": rr*np.cos(elr)*np.cos(azr), "z_m": rr*np.sin(elr)})
                azs = [row["azimuth_deg"] for row in rows[-len(stored):]] if stored else []
                els = [row["elevation_deg"] for row in rows[-len(stored):]] if stored else []
                summaries.append({"case_id": case_id, "scenario_id": scenario["scenario_id"], "detector": detector,
                                  "valid_cells": total_valid, "detection_count": len(candidates),
                                  "stored_point_count": len(stored), "detections_per_frame": len(candidates)/max(power.shape[0],1),
                                  "mean_azimuth_deg": float(np.mean(azs)) if azs else float("nan"),
                                  "std_azimuth_deg": float(np.std(azs)) if azs else float("nan"),
                                  "mean_elevation_deg": float(np.mean(els)) if els else float("nan"),
                                  "std_elevation_deg": float(np.std(els)) if els else float("nan"),
                                  "geometry_status": "candidate_mapping_not_phase_center_calibrated"})
    output.mkdir(parents=True, exist_ok=True)
    def write_csv(name, data):
        fields = list(data[0]) if data else ["case_id"]
        with (output/name).open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(data)
    write_csv("detector_point_cloud.csv", rows); write_csv("detector_point_cloud_summary.csv", summaries)
    meta = {"status":"completed_detector_point_cloud", "case_count":len({r["case_id"] for r in summaries}),
            "scenario_count":len(SCENARIOS), "detectors":list(DETECTORS), "max_points_per_group":max_points_per_group,
            "geometry_source":str(geometry_csv.resolve()), "nonfinite_aoa_count":nonfinite_aoa_count,
            "hardware_validated":False,
            "os_cfar_status":"exploratory_order_statistic_not_ti_sdk_equivalent"}
    (output/"summary.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    (output/"output_analysis.md").write_text("# V0.4.72 检测器 AoA 点云\n\n"
        "本阶段将 V0.4.71 在相同距离-多普勒谱上保留的检测单元转换为方位、俯仰和三维坐标。`detection_count` 是全部保留单元，CSV 仅保存每组功率最高的前 N 个点；因此 `stored_point_count` 不代表检测器总数。\n\n"
        "## 结果解读\n\n- 比较同一海况、同一 CFAR 场景下四种 detector 的 detection_count，可看检测规则对海杂波保留量的影响。\n- 比较 azimuth/elevation 的均值和标准差，可看保留点云的空间偏置与离散程度。\n- `x_m/y_m/z_m` 采用雷达坐标系：x 为右舷横向、y 为前向、z 为上向；距离和速度来自合成谱坐标轴。\n\n## 证据边界\n\n输入为 MATLAB 合成 HDF5，不是 DCA1000 实测 IQ；阵列坐标仍是候选映射而非电气相位中心；OS-CFAR 为探索性实现，不等价于 TI SDK。\n", encoding="utf-8")
    return meta


def main():
    p = argparse.ArgumentParser(); p.add_argument("--input-root", type=Path, required=True); p.add_argument("--geometry", type=Path, required=True); p.add_argument("--output", type=Path, required=True); p.add_argument("--max-points-per-group", type=int, default=256); a=p.parse_args()
    print(json.dumps(run(a.input_root.resolve(), a.geometry.resolve(), a.output.resolve(), a.max_points_per_group), indent=2, ensure_ascii=False))


if __name__ == "__main__": main()
