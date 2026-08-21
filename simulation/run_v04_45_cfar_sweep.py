"""Sweep CA-CFAR settings on synthetic sea range-Doppler power spectra."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_cfar_point_cloud import ca_cfar


SCENARIOS = [
    {"scenario_id": "pfa1e2_train2", "pfa": 1e-2, "training": (2, 2), "guard": (1, 1)},
    {"scenario_id": "pfa1e3_train2", "pfa": 1e-3, "training": (2, 2), "guard": (1, 1)},
    {"scenario_id": "pfa1e4_train2", "pfa": 1e-4, "training": (2, 2), "guard": (1, 1)},
    {"scenario_id": "pfa1e2_train4", "pfa": 1e-2, "training": (4, 4), "guard": (1, 1)},
    {"scenario_id": "pfa1e3_train4", "pfa": 1e-3, "training": (4, 4), "guard": (1, 1)},
    {"scenario_id": "pfa1e4_train4", "pfa": 1e-4, "training": (4, 4), "guard": (1, 1)},
    {"scenario_id": "pfa1e2_train8", "pfa": 1e-2, "training": (8, 8), "guard": (2, 2)},
    {"scenario_id": "pfa1e3_train8", "pfa": 1e-3, "training": (8, 8), "guard": (2, 2)},
    {"scenario_id": "pfa1e4_train8", "pfa": 1e-4, "training": (8, 8), "guard": (2, 2)},
]


def load_case(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    with h5py.File(path, "r") as handle:
        return handle["/range_doppler/power_linear"][...], handle["/axes/range_m"][...], handle["/axes/velocity_mps"][...]


def run_case(path: Path, scenario: dict) -> dict:
    power, ranges, velocities = load_case(path)
    detections = []; alpha = None
    for frame in range(power.shape[0]):
        found, alpha = ca_cfar(power[frame], training=scenario["training"], guard=scenario["guard"], pfa=scenario["pfa"])
        for d_idx, r_idx, peak, noise, threshold in found:
            detections.append({"frame": frame, "doppler_index": d_idx, "range_index": r_idx, "range_m": float(ranges[r_idx]), "velocity_mps": float(velocities[d_idx]), "power_linear": peak, "noise_linear": noise, "threshold_linear": threshold})
    case_id = path.stem.replace("_range_doppler", "")
    power_values = np.asarray([row["power_linear"] for row in detections], dtype=float)
    return {"case_id": case_id, "scenario_id": scenario["scenario_id"], "pfa": scenario["pfa"], "training_doppler": scenario["training"][0], "training_range": scenario["training"][1], "guard_doppler": scenario["guard"][0], "guard_range": scenario["guard"][1], "cfar_alpha": alpha, "frames": int(power.shape[0]), "point_count": len(detections), "points_per_frame": len(detections) / max(power.shape[0], 1), "mean_power_linear": float(np.mean(power_values)) if len(power_values) else None, "max_power_linear": float(np.max(power_values)) if len(power_values) else None, "mean_range_m": float(np.mean([row["range_m"] for row in detections])) if detections else None, "mean_velocity_mps": float(np.mean([row["velocity_mps"] for row in detections])) if detections else None}


def run(input_root: Path, output: Path) -> dict:
    paths = sorted(input_root.glob("*_range_doppler.h5")); rows = []
    for path in paths:
        for scenario in SCENARIOS:
            rows.append(run_case(path, scenario))
    output.mkdir(parents=True, exist_ok=True)
    with (output / "cfar_sweep.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    with (output / "case_scenario_matrix.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["case_id", "scenario_id", "point_count", "points_per_frame", "mean_power_linear", "mean_range_m", "mean_velocity_mps"]); writer.writeheader(); writer.writerows([{key: row[key] for key in writer.fieldnames} for row in rows])
    summary = {"status": "completed_cfar_parameter_sweep", "input_root": str(input_root.resolve()), "scenario_count": len(SCENARIOS), "case_count": len(paths), "purpose": "separate_cfar_threshold_sensitivity_from_sea_state_effect", "hardware_validated": False, "rows": rows}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.45 CA-CFAR 参数扫描", "", "本阶段固定 V0.4.43 合成距离-多普勒功率谱，只改变 Pfa、训练单元和保护单元，区分 CFAR 门限效应与海况/散射模型效应。", "", "| 海况 | CFAR 场景 | Pfa | 训练单元 | 检测点数 | 点/帧 | 平均功率 |", "|---|---|---:|---|---:|---:|---:|"]
    lines.extend(f"| {row['case_id']} | {row['scenario_id']} | {row['pfa']:.0e} | {row['training_doppler']}×{row['training_range']} | {row['point_count']} | {row['points_per_frame']:.3f} | {row['mean_power_linear'] if row['mean_power_linear'] is not None else float('nan'):.3e} |" for row in rows)
    lines += ["", "## 分析方法", "", "先固定同一海况的所有输入，只横向比较 CFAR 场景；再固定 CFAR 场景，纵向比较 ss0～ss3。若同一海况的检测点数随 Pfa/训练窗显著变化，说明 V0.4.44 的零点现象部分来自门限配置，不能直接归因于海况。", "", "## 边界", "", "输入仍是 V0.4.43 合成海杂波功率谱，不是实测 IQ；本阶段只统计 CFAR，不重新估计 AoA；结果不能作为真实虚警率或探测距离。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
