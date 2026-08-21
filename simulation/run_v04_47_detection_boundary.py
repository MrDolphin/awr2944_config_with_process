"""Sweep target detectability boundaries over SNR, range, velocity and AoA."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_46_target_in_sea_clutter import run_case
from simulation.v03 import FmcwConfig


PFA_VALUES = (1e-2, 1e-3)
SEA_CASES = ("ss2_normal", "ss3_upper")
BASELINE = {"snr_db": 10.0, "range_m": 20.0, "velocity_mps": 1.0, "azimuth_deg": 10.0, "elevation_deg": 2.0}
SWEEP_VALUES = {
    "snr_db": (0.0, 5.0, 10.0, 15.0, 20.0),
    "range_m": (8.0, 20.0, 40.0, 60.0, 80.0),
    "velocity_mps": (-3.0, 0.0, 1.0, 3.0, 6.0),
    "azimuth_deg": (-30.0, -10.0, 10.0, 30.0),
    "elevation_deg": (-4.0, 0.0, 4.0),
}


def scenario(variable: str, value: float, pfa: float) -> dict:
    return {
        "scenario_id": f"{variable}_{value:g}_pfa{pfa:g}",
        "target_snr_db": value if variable == "snr_db" else BASELINE["snr_db"],
        "pfa": pfa,
        "training": (2, 2),
        "guard": (1, 1),
    }


def run(input_root: Path, output: Path) -> dict:
    config = FmcwConfig(samples_per_chirp=128, chirps_per_frame=64)
    models = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config)
    positions = models["pcb_centroid_candidate"]
    rows: list[dict] = []
    for case_id in SEA_CASES:
        path = input_root / f"{case_id}_range_doppler.h5"
        for variable, values in SWEEP_VALUES.items():
            for value in values:
                target = dict(BASELINE)
                target[variable] = value
                for pfa in PFA_VALUES:
                    result = run_case(path, scenario(variable, value, pfa), positions, config, target["azimuth_deg"], target["elevation_deg"], target["range_m"], target["velocity_mps"])
                    result.update({"sweep_variable": variable, "sweep_value": value, "target_azimuth_truth_deg": target["azimuth_deg"], "target_elevation_truth_deg": target["elevation_deg"], "target_range_truth_m": target["range_m"], "target_velocity_truth_mps": target["velocity_mps"]})
                    rows.append(result)
    output.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0])
    with (output / "detection_boundary_sweep.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    summary = {"status": "completed_detection_boundary_sweep", "input_root": str(input_root.resolve()), "sea_cases": SEA_CASES, "pfa_values": PFA_VALUES, "baseline": BASELINE, "sweep_values": SWEEP_VALUES, "row_count": len(rows), "input_status": "synthetic_sea_spectrum_not_measured_iq", "hardware_validated": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.47 目标检测边界扫描", "", "本阶段在 V0.4.43 合成距离-Doppler 海杂波谱上，分别扫描目标 SNR、距离、径向速度、方位和俯仰；海况选取 `ss2_normal` 与有效波高 1 m 的 `ss3_upper`。每个点使用 41 帧、CA-CFAR 训练窗 `(2,2)`、保护窗 `(1,1)`，并比较 `Pfa=10^-2/10^-3`。", "", "## 结果读取", "", "`detection_probability` 是目标单元 ±1 网格内的命中帧比例；`mean_false_alarms_per_frame` 是扣除目标命中后的平均检测点数；AoA RMSE 只在命中帧上统计。", "", "## 每组扫描的摘要", "", "| 海况 | 扫描变量 | Pfa | 最小检测概率 | 最大检测概率 | 最大虚警点/帧 |", "|---|---|---:|---:|---:|---:|"]
    for case in SEA_CASES:
        for variable in SWEEP_VALUES:
            subset = [r for r in rows if r["case_id"] == case and r["sweep_variable"] == variable]
            for pfa in PFA_VALUES:
                group = [r for r in subset if r["pfa"] == pfa]
                lines.append(f"| {case} | {variable} | {pfa:g} | {min(r['detection_probability'] for r in group):.3f} | {max(r['detection_probability'] for r in group):.3f} | {max(r['mean_false_alarms_per_frame'] for r in group):.3f} |")
    lines += ["", "## 初步结论", "", "扫描结果用于确定后续重点工况：检测概率下降的位置是目标检测边界候选，虚警点随海况和 Pfa 的变化用于 CFAR 参数权衡。由于目标单元仍为受控相干注入，结果不能替代实测探测距离、真实目标 RCS、TI SDK AoA 或海试检测概率。", "", "## 输出文件", "", "- `detection_boundary_sweep.csv`：逐海况、逐扫描点、逐 Pfa 的完整指标。", "- `summary.json`：输入与扫描配置。", "- 本文件：面向汇报的解释和边界。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
