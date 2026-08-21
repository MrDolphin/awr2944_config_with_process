"""Combine pre-CFAR energy baselines with the V0.45 CFAR sweep."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_45_cfar_sweep import SCENARIOS, run_case


def _energy(path: Path) -> dict:
    with h5py.File(path, "r") as handle:
        power = np.asarray(handle["/range_doppler/power_linear"][...], dtype=float)
        spectrum = handle["/range_doppler/spectrum_complex"]
        spectrum_shape = list(spectrum.shape)
    values = power.reshape(-1)
    median = float(np.median(values))
    return {
        "power_mean_linear": float(np.mean(values)),
        "power_median_linear": median,
        "power_p95_linear": float(np.percentile(values, 95)),
        "power_p99_linear": float(np.percentile(values, 99)),
        "power_p999_linear": float(np.percentile(values, 99.9)),
        "power_max_linear": float(np.max(values)),
        "max_to_median_db": float(10.0 * np.log10(max(np.max(values), 1e-30) / max(median, 1e-30))),
        "power_shape": list(power.shape),
        "spectrum_shape": spectrum_shape,
    }


def run(input_root: Path, output: Path) -> dict:
    paths = sorted(input_root.glob("*_range_doppler.h5"))
    if not paths:
        raise ValueError("no range_doppler HDF5 files found")
    energy_rows: list[dict] = []
    sweep_rows: list[dict] = []
    for path in paths:
        case_id = path.stem.replace("_range_doppler", "")
        energy_rows.append({"case_id": case_id, **_energy(path)})
        for scenario in SCENARIOS:
            sweep_rows.append(run_case(path, scenario))
    output.mkdir(parents=True, exist_ok=True)
    with (output / "pre_cfar_energy.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(energy_rows[0])); writer.writeheader(); writer.writerows(energy_rows)
    with (output / "cfar_energy_matrix.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = list(sweep_rows[0]) + ["power_median_linear", "power_p99_linear", "max_to_median_db"]
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader()
        energy = {row["case_id"]: row for row in energy_rows}
        for row in sweep_rows:
            source = energy[row["case_id"]]
            writer.writerow({**row, "power_median_linear": source["power_median_linear"], "power_p99_linear": source["power_p99_linear"], "max_to_median_db": source["max_to_median_db"]})
    zero_cases = [row["case_id"] for row in energy_rows if not any(item["case_id"] == row["case_id"] and item["point_count"] > 0 for item in sweep_rows)]
    summary = {
        "status": "completed_cfar_energy_matrix",
        "case_count": len(paths), "scenario_count": len(SCENARIOS),
        "zero_detection_cases_across_all_scenarios": zero_cases,
        "energy_only_cases_need_uncfar_interpretation": zero_cases,
        "hardware_validated": False,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# V0.4.68 CFAR 参数与未 CFAR 能量矩阵",
        "",
        f"海况数：{len(paths)}；CFAR 场景数：{len(SCENARIOS)}；所有场景均无检测点的案例：{zero_cases}。",
        "",
        "## 分析方法",
        "",
        "`pre_cfar_energy.csv` 统计未经过 CFAR 的功率线性值和复数谱形状；`cfar_energy_matrix.csv` 将同一海况的功率基线与 Pfa/训练窗/保护窗组合后的检测点数放在一起。",
        "",
        "判断顺序：先看某海况的 P95/P99/P99.9 和 max_to_median_db，再看各 CFAR 场景的 point_count。如果未 CFAR 能量有明显长尾但所有 CFAR 都是零点，优先怀疑门限/训练窗；如果功率整体接近噪声且无长尾，才说明当前仿真散射能量不足以形成检测。",
        "",
        "## 证据边界",
        "",
        "本阶段使用 V0.43 合成功率谱和复数谱，不是 DCA1000 实测 IQ；不能据此报告真实虚警率或检测距离。零检测仍不等于零海杂波，必须结合未 CFAR 能量分布解释。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
