"""Apply the coarse, source-traced EVM pattern tables to V0.120 facets."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def _pattern(path: Path, angle_key: str) -> tuple[np.ndarray, np.ndarray]:
    angles = []; gains = []
    with path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            angles.append(abs(float(row[angle_key]))); gains.append(float(row["gain_dBi"]))
    order = np.argsort(angles)
    return np.asarray(angles)[order], np.asarray(gains)[order]


def run(physics_csv: Path, az_pattern: Path, el_pattern: Path, output: Path) -> dict:
    az_axis, az_gain = _pattern(az_pattern, "azimuth_deg")
    el_axis, el_gain = _pattern(el_pattern, "elevation_deg")
    with physics_csv.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        az = abs(float(row["azimuth_deg"])); el = abs(float(row["elevation_down_deg"]))
        gain_az = float(np.interp(az, az_axis, az_gain)); gain_el = float(np.interp(el, el_axis, el_gain))
        row["pattern_az_gain_dBi"] = gain_az; row["pattern_el_gain_dBi"] = gain_el; row["pattern_total_gain_dBi"] = gain_az + gain_el; row["pattern_weight_linear"] = 10.0 ** ((gain_az + gain_el) / 10.0); row["pattern_weighted_scatter_3db"] = float(row["masked_scatter_3db"]) * row["pattern_weight_linear"]; row["pattern_weighted_scatter_6db"] = float(row["masked_scatter_6db"]) * row["pattern_weight_linear"]
    output.mkdir(parents=True, exist_ok=True)
    with (output / "pattern_weighted_clutter.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summaries = []
    for case_id in sorted({row["case_id"] for row in rows}):
        part = [row for row in rows if row["case_id"] == case_id]
        summaries.append({"case_id": case_id, "rows": len(part), "unweighted_scatter_3db_sum": float(sum(float(row["masked_scatter_3db"]) for row in part)), "pattern_weighted_scatter_3db_sum": float(sum(float(row["pattern_weighted_scatter_3db"]) for row in part)), "unweighted_scatter_6db_sum": float(sum(float(row["masked_scatter_6db"]) for row in part)), "pattern_weighted_scatter_6db_sum": float(sum(float(row["pattern_weighted_scatter_6db"]) for row in part)), "mean_pattern_gain_dBi": float(np.mean([float(row["pattern_total_gain_dBi"]) for row in part]))})
    with (output / "case_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    result = {"status": "completed_coarse_pattern_weighted_clutter", "physics_source": str(physics_csv.resolve()), "azimuth_pattern": str(az_pattern.resolve()), "elevation_pattern": str(el_pattern.resolve()), "pattern_evidence": "coarse_digitization_from_TI_EVM_user_guide_figures", "row_count": len(rows), "hardware_aoa_validated": False, "summaries": summaries}
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        labels = [row["case_id"] for row in summaries]; x = np.arange(len(labels)); width = .35
        figure, axis = plt.subplots(figsize=(9, 4), constrained_layout=True); axis.bar(x - width / 2, [row["unweighted_scatter_3db_sum"] for row in summaries], width, label="unweighted"); axis.bar(x + width / 2, [row["pattern_weighted_scatter_3db_sum"] for row in summaries], width, label="coarse pattern weighted"); axis.set_xticks(x, labels, rotation=20); axis.set_ylabel("relative 3 dB scatter sum"); axis.set_title("Coarse EVM pattern effect on clutter proxy"); axis.legend(); axis.grid(True, axis="y", alpha=.25); figure.savefig(output / "pattern_weighted_comparison.png", dpi=160); plt.close(figure)
    except Exception:
        pass
    lines = ["# V0.4.124 粗粒度 EVM 方向图加权海杂波代理", "", "本阶段将硬件资料包中的方位/俯仰方向图 CSV 插值到 V0.120 微元角度，并把 dBi 增益转换为相对线性权重，比较加权前后的散射代理。", "", "## 方向图证据等级", "", "输入 CSV 的来源是 TI EVM User Guide Figure 2-19/2-20 的粗粒度数字化，文件状态为 `coarse_digitization`。它可用于接口和敏感性分析，不是完整实测方向图。", "", "## 如何解释", "", "方向图加权后，离开主瓣的微元相对权重会降低；这能说明天线方向性如何改变海杂波候选分布，但 dBi 权重没有包含双程传播、极化、互耦、安装罩体和真实相位。", "", "## 证据边界", "", "结果仍来自 V0.120 高度场/散射代理，不是 AWR2944P 实测功率或点云；不能用于声称实板方向图、探测距离或虚警率。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--physics", type=Path, required=True); parser.add_argument("--az-pattern", type=Path, required=True); parser.add_argument("--el-pattern", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.physics.resolve(), args.az_pattern.resolve(), args.el_pattern.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
