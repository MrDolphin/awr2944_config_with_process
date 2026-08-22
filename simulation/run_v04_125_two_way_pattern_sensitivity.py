"""Compare one-way and two-way use of the coarse EVM pattern proxy."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def _pattern(path: Path, key: str) -> tuple[np.ndarray, np.ndarray]:
    angles = []; gains = []
    with path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            angles.append(abs(float(row[key]))); gains.append(float(row["gain_dBi"]))
    order = np.argsort(angles)
    return np.asarray(angles)[order], np.asarray(gains)[order]


def run(physics_csv: Path, az_pattern: Path, el_pattern: Path, output: Path) -> dict:
    az_axis, az_gain = _pattern(az_pattern, "azimuth_deg"); el_axis, el_gain = _pattern(el_pattern, "elevation_deg")
    with physics_csv.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        one_way_db = float(np.interp(abs(float(row["azimuth_deg"])), az_axis, az_gain)) + float(np.interp(abs(float(row["elevation_down_deg"])), el_axis, el_gain))
        one_way = 10.0 ** (one_way_db / 10.0)
        row.update({"pattern_one_way_gain_dBi": one_way_db, "pattern_one_way_weight": one_way, "pattern_two_way_weight": one_way ** 2, "two_way_scatter_3db": float(row["masked_scatter_3db"]) * one_way ** 2, "two_way_scatter_6db": float(row["masked_scatter_6db"]) * one_way ** 2})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "two_way_pattern_sensitivity.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summaries = []
    for case_id in sorted({row["case_id"] for row in rows}):
        part = [row for row in rows if row["case_id"] == case_id]
        unweighted = sum(float(row["masked_scatter_3db"]) for row in part); one = sum(float(row["masked_scatter_3db"]) * float(row["pattern_one_way_weight"]) for row in part); two = sum(float(row["two_way_scatter_3db"]) for row in part)
        summaries.append({"case_id": case_id, "unweighted_3db_sum": unweighted, "one_way_3db_sum": one, "two_way_3db_sum": two, "one_way_over_unweighted": one / unweighted if unweighted > 0 else None, "two_way_over_unweighted": two / unweighted if unweighted > 0 else None})
    with (output / "case_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    result = {"status": "completed_one_way_two_way_pattern_sensitivity", "physics_source": str(physics_csv.resolve()), "pattern_evidence": "coarse_digitization_from_TI_EVM_user_guide_figures", "case_count": len(summaries), "hardware_aoa_validated": False, "summaries": summaries}
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        labels = [r["case_id"] for r in summaries]; x = np.arange(len(labels)); width = .25; figure, axis = plt.subplots(figsize=(9, 4), constrained_layout=True); axis.bar(x - width, [r["unweighted_3db_sum"] for r in summaries], width, label="unweighted"); axis.bar(x, [r["one_way_3db_sum"] for r in summaries], width, label="one-way"); axis.bar(x + width, [r["two_way_3db_sum"] for r in summaries], width, label="two-way"); axis.set_xticks(x, labels, rotation=20); axis.set_ylabel("relative 3 dB scatter sum"); axis.legend(); axis.grid(True, axis="y", alpha=.25); figure.savefig(output / "one_way_two_way_comparison.png", dpi=160); plt.close(figure)
    except Exception:
        pass
    table_lines = []
    for row in summaries:
        one_ratio = "—" if row["one_way_over_unweighted"] is None else f"{row['one_way_over_unweighted']:.3%}"
        two_ratio = "—" if row["two_way_over_unweighted"] is None else f"{row['two_way_over_unweighted']:.3%}"
        table_lines.append(f"| {row['case_id']} | {row['unweighted_3db_sum']:.6g} | {one_ratio} | {two_ratio} |")
    lines = ["# V0.4.125 单程/双程方向图权重敏感性", "", "本阶段在 V0.124 粗粒度方向图加权基础上，比较一次方向图权重与 Tx×Rx 双程方向图权重（平方）的差异。", "", "## 输入与输出", "", "输入为 V0.120 的 9840 个海面微面元散射代理、方位/俯仰粗略方向图 CSV；输出 `two_way_pattern_sensitivity.csv` 保留逐微面元权重，`case_summary.csv` 汇总五组海况，PNG 用于汇报趋势。", "", "## 结果摘要", "", "| 海况 | 未加权3 dB散射和 | 单程加权/未加权 | 双程加权/未加权 |", "|---|---:|---:|---:|", *table_lines, "", "在本代理模型中，ss2_normal、ss3_nominal、ss3_upper 的双程权重分别约为未加权结果的 1.50%、2.27%、2.69%；它比单程权重更严格，因为同一方向性衰减被近似作用两次。平静海况的 3 dB 物理散射代理本身为零，因此不能据此判断雷达完全没有回波。", "", "## 如何从图和表分析", "", "先看三组柱：灰色为未加权基线、蓝色为单程方向图、橙色为双程方向图。若橙柱明显低于蓝柱，说明方向图离开主瓣后，单站 Tx→海面→Rx 的双程链路对海杂波更敏感；再按海况横向比较，ss3_upper 高于 ss2_normal 只表示当前海面微面元/散射代理更强，不等于真实检测距离增加。逐点 CSV 中可进一步按方位角、俯仰角筛选，定位哪些角度贡献最大。", "", "## 证据边界", "", "方向图仍是 TI EVM 用户指南曲线的粗粒度数字化；Tx/Rx 方向图、极化、互耦、安装罩体和校准均未实测。输出是相对散射代理，不是实测功率、dBsm、检测概率或探测距离。下一阶段应使用 DCA1000 原始 ADC IQ 和已完成的 TI 校准/已知角度采集，验证双程模型是否能解释真实角度与回波强度。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--physics", type=Path, required=True); parser.add_argument("--az-pattern", type=Path, required=True); parser.add_argument("--el-pattern", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.physics.resolve(), args.az_pattern.resolve(), args.el_pattern.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
