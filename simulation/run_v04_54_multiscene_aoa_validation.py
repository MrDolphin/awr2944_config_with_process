"""Aggregate RX/TX order candidates across multiple known-angle HDF5 scenes."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_53_hdf5_channel_order_validation import load_virtual_iq, extract_peak_channel
from simulation.run_v04_39_sparse_array_solver import estimate_sparse_grid
from simulation.v04 import virtual_array_positions
from simulation.v03 import FmcwConfig


def run(inputs: list[Path], output: Path) -> dict:
    if not inputs:
        raise ValueError("at least one HDF5 scene is required")
    scene_rows = []
    candidates = list(itertools.product(itertools.permutations(range(4)), itertools.permutations(range(4))))
    for input_path in inputs:
        iq, metadata = load_virtual_iq(input_path); config = FmcwConfig(samples_per_chirp=iq.shape[1], chirps_per_frame=iq.shape[0]); x, y = virtual_array_positions(config); channel, peak_range, peak_velocity, peak_power = extract_peak_channel(iq, config); truth_az = float(metadata.get("truth_azimuth_deg", 0.0)); truth_el = float(metadata.get("truth_elevation_deg", 0.0))
        for candidate_index, (rx_order, tx_order) in enumerate(candidates):
            estimate = estimate_sparse_grid(config, channel[np.ix_(rx_order, tx_order)], x, y, np.arange(-60.0, 60.01, 1.0), np.arange(-20.0, 20.01, 1.0)); scene_rows.append({"scene": input_path.name, "candidate_index": candidate_index, "rx_order": ",".join(map(str, rx_order)), "tx_order": ",".join(map(str, tx_order)), "truth_azimuth_deg": truth_az, "truth_elevation_deg": truth_el, "estimated_azimuth_deg": estimate[0], "estimated_elevation_deg": estimate[1], "azimuth_error_deg": estimate[0] - truth_az, "elevation_error_deg": estimate[1] - truth_el, "score": estimate[2], "peak_range_m": peak_range, "peak_velocity_mps": peak_velocity, "peak_power_linear": peak_power})
    aggregate = []
    for candidate_index in range(len(candidates)):
        rows = [row for row in scene_rows if row["candidate_index"] == candidate_index]; az = np.asarray([row["azimuth_error_deg"] for row in rows]); el = np.asarray([row["elevation_error_deg"] for row in rows]); aggregate.append({"candidate_index": candidate_index, "rx_order": rows[0]["rx_order"], "tx_order": rows[0]["tx_order"], "azimuth_rmse_deg": float(np.sqrt(np.mean(az ** 2))), "elevation_rmse_deg": float(np.sqrt(np.mean(el ** 2))), "combined_rmse_deg": float(np.sqrt(np.mean(np.concatenate((az, el)) ** 2))), "max_abs_azimuth_error_deg": float(np.max(np.abs(az))), "max_abs_elevation_error_deg": float(np.max(np.abs(el))), "mean_score": float(np.mean([row["score"] for row in rows])), "identity_order": rows[0]["rx_order"] == "0,1,2,3" and rows[0]["tx_order"] == "0,1,2,3"})
    aggregate.sort(key=lambda row: row["combined_rmse_deg"]); output.mkdir(parents=True, exist_ok=True)
    for name, rows in (("multi_scene_errors.csv", scene_rows), ("multi_scene_candidate_metrics.csv", aggregate)):
        with (output / name).open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summary = {"status": "completed_multiscene_known_angle_validation", "scene_count": len(inputs), "scene_files": [str(path.resolve()) for path in inputs], "candidate_count": len(aggregate), "best_candidates": aggregate[:10], "identity_candidate": next(row for row in aggregate if row["identity_order"]), "source_is_hardware_measurement": False, "channel_order_verified": False, "geometry_source": "simulation.v04.virtual_array_positions"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.54 多场景已知角度 AoA 联合验证", "", f"场景数：{len(inputs)}；RX/TX 候选数：{len(aggregate)}。每个候选在所有场景上联合计算方位和俯仰 RMSE。", "", "## 最佳候选", "", "| RX 顺序 | TX 顺序 | 方位 RMSE | 俯仰 RMSE | 综合 RMSE | 最大方位误差 | 最大俯仰误差 |", "|---|---|---:|---:|---:|---:|---:|"]
    lines.extend(f"| {row['rx_order']} | {row['tx_order']} | {row['azimuth_rmse_deg']:.3f}° | {row['elevation_rmse_deg']:.3f}° | {row['combined_rmse_deg']:.3f}° | {row['max_abs_azimuth_error_deg']:.1f}° | {row['max_abs_elevation_error_deg']:.1f}° |" for row in aggregate[:10])
    identity = summary["identity_candidate"]; lines += ["", f"身份排列 `{identity['rx_order']}/{identity['tx_order']}` 的综合 RMSE：{identity['combined_rmse_deg']:.3f}°。", "", "## 如何解释", "", "只有在多个角度和距离场景上都稳定误差较小的候选，才值得进入真实硬件验证；单场景偶然接近不能证明通道顺序正确。当前场景均为合成 known-angle fixture，几何源为 simulation.v04，不能替代实测 DCA1000 和 TI SDK AoA。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-glob", type=str, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); inputs = sorted(Path().glob(args.input_glob)); print(json.dumps(run(inputs, args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
