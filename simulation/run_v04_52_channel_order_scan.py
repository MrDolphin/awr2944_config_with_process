"""Enumerate RX/TX channel-order candidates on a known-angle AoA fixture."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_39_sparse_array_solver import estimate_sparse_grid
from simulation.run_v04_46_target_in_sea_clutter import steering
from simulation.v03 import FmcwConfig


def permute_channel(channel: np.ndarray, rx_order: tuple[int, ...], tx_order: tuple[int, ...]) -> np.ndarray:
    return channel[np.ix_(rx_order, tx_order)]


def run(mapping: Path, pcb: Path, output: Path) -> dict:
    config = FmcwConfig(samples_per_chirp=128, chirps_per_frame=64); positions = model_positions(mapping, pcb, config)["pcb_centroid_candidate"]
    solver_az = np.arange(-60.0, 60.01, 1.0); solver_el = np.arange(-20.0, 20.01, 1.0); truths = ((10.0, 2.0), (30.0, 5.0)); rows = []
    for truth_az, truth_el in truths:
        channel = steering(config, positions, truth_az, truth_el)
        for rx_order in itertools.permutations(range(4)):
            for tx_order in itertools.permutations(range(4)):
                observed = permute_channel(channel, rx_order, tx_order); estimate = estimate_sparse_grid(config, observed, positions[0], positions[1], solver_az, solver_el)
                rows.append({"truth_azimuth_deg": truth_az, "truth_elevation_deg": truth_el, "rx_order": ",".join(map(str, rx_order)), "tx_order": ",".join(map(str, tx_order)), "estimated_azimuth_deg": estimate[0], "estimated_elevation_deg": estimate[1], "azimuth_error_deg": estimate[0] - truth_az, "elevation_error_deg": estimate[1] - truth_el, "score": estimate[2], "identity_order": rx_order == (0, 1, 2, 3) and tx_order == (0, 1, 2, 3)})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "channel_order_candidates.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    correct = [row for row in rows if row["identity_order"]]; wrong = [row for row in rows if not row["identity_order"]]
    summary = {"status": "completed_known_angle_channel_order_scan", "candidate_count": len(rows), "truths": truths, "correct_order_rows": correct, "wrong_order_azimuth_rmse_deg": float(np.sqrt(np.mean([row["azimuth_error_deg"] ** 2 for row in wrong]))), "wrong_order_elevation_rmse_deg": float(np.sqrt(np.mean([row["elevation_error_deg"] ** 2 for row in wrong]))), "wrong_order_fraction_within_2deg": float(np.mean([abs(row["azimuth_error_deg"]) <= 2.0 and abs(row["elevation_error_deg"]) <= 2.0 for row in wrong])), "hardware_validated": False, "channel_order_verified": False, "geometry_status": "pcb_copper_centroid_candidate"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    best = sorted(wrong, key=lambda row: abs(row["azimuth_error_deg"]) + abs(row["elevation_error_deg"]))[:10]
    lines = ["# V0.4.52 已知角度 RX/TX 通道顺序扫描", "", "本阶段使用 PCB 铜区质心候选阵列生成两个已知角度复数通道（10°/2°、30°/5°），枚举 24×24 个 RX/TX 排列，并用同一稀疏导向矢量 AoA 求解器估计角度。", "", "## 如何解释", "", "`channel_order_candidates.csv` 每行是一种 RX/TX 排列；`azimuth_error_deg` 和 `elevation_error_deg` 是相对于已知角度的误差。正确排列只能由真实 CFG、LVDS 语义和已知角度实测共同确认。", "", "## 结果摘要", "", f"- 候选数：{len(rows)}（两个真值各 576 种排列）", f"- 身份排列估计：方位 {correct[0]['estimated_azimuth_deg']:.1f}°、俯仰 {correct[0]['estimated_elevation_deg']:.1f}°（仅合成回归）", f"- 错误排列方位 RMSE：{summary['wrong_order_azimuth_rmse_deg']:.3f}°", f"- 错误排列俯仰 RMSE：{summary['wrong_order_elevation_rmse_deg']:.3f}°", f"- 错误排列仍落在 ±2° 的比例：{summary['wrong_order_fraction_within_2deg']:.3f}", "", "## 最容易误判的错误排列（前10）", "", "| 真值 | RX 顺序 | TX 顺序 | 估计方位 | 估计俯仰 | 方位误差 | 俯仰误差 |", "|---|---|---|---:|---:|---:|---:|"]
    lines.extend(f"| {row['truth_azimuth_deg']:.0f}°/{row['truth_elevation_deg']:.0f}° | {row['rx_order']} | {row['tx_order']} | {row['estimated_azimuth_deg']:.1f}° | {row['estimated_elevation_deg']:.1f}° | {row['azimuth_error_deg']:.1f}° | {row['elevation_error_deg']:.1f}° |" for row in best)
    lines += ["", "## 结论和边界", "", "通道顺序错误会改变复数阵列相位关系，导致 AoA 偏差；即使某些错误排列偶然接近真值，也不能证明排列正确。当前使用的是合成导向矢量和候选几何，不是 TI SDK 或实测角度精度。真实硬件必须用已知角度角反射器和 DCA1000 原始 IQ 锁定 RX/TX 顺序。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--mapping", type=Path, required=True); parser.add_argument("--pcb", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.mapping.resolve(), args.pcb.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
