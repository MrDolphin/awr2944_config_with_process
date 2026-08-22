"""Join target detection-boundary results with sea-clutter physics proxies."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def run(boundary_csv: Path, physics_csv: Path, output: Path) -> dict:
    with boundary_csv.open(encoding="utf-8", newline="") as handle:
        boundary = list(csv.DictReader(handle))
    with physics_csv.open(encoding="utf-8", newline="") as handle:
        physics = {row["case_id"]: row for row in csv.DictReader(handle)}
    rows = []
    for item in boundary:
        p = physics.get(item["case_id"])
        if p is None:
            continue
        rows.append({**item, "three_db_facets": p["three_db_facets"], "six_db_facets": p["six_db_facets"], "mean_three_db_scatter_proxy": p["mean_three_db_scatter_proxy"], "three_db_doppler_min_hz": p["three_db_doppler_min_hz"], "three_db_doppler_max_hz": p["three_db_doppler_max_hz"], "rms_three_db_radial_velocity_mps": p["rms_three_db_radial_velocity_mps"]})
    if not rows:
        raise ValueError("no common case IDs")
    output.mkdir(parents=True, exist_ok=True)
    with (output / "detection_physics_join.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summaries = []
    for case_id in sorted({row["case_id"] for row in rows}):
        part = [row for row in rows if row["case_id"] == case_id]
        summaries.append({"case_id": case_id, "rows": len(part), "max_detection_probability": max(float(row["detection_probability"]) for row in part), "min_detection_probability": min(float(row["detection_probability"]) for row in part), "max_false_alarms_per_frame": max(float(row["mean_false_alarms_per_frame"]) for row in part), "three_db_facets": physics[case_id]["three_db_facets"], "mean_three_db_scatter_proxy": physics[case_id]["mean_three_db_scatter_proxy"], "three_db_doppler_min_hz": physics[case_id]["three_db_doppler_min_hz"], "three_db_doppler_max_hz": physics[case_id]["three_db_doppler_max_hz"]})
    with (output / "case_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    result = {"status": "completed_detection_physics_join", "boundary_source": str(boundary_csv.resolve()), "physics_source": str(physics_csv.resolve()), "row_count": len(rows), "case_count": len(summaries), "hardware_validated": False, "interpretation": "descriptive synthetic join; target injection and clutter proxies are not measured"}
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        figure, axis = plt.subplots(figsize=(7, 4), constrained_layout=True)
        for row in summaries: axis.scatter(float(row["three_db_facets"]), float(row["max_detection_probability"]), s=60, label=row["case_id"])
        axis.set_xlabel("3 dB masked microfacet count"); axis.set_ylabel("maximum target detection probability"); axis.set_title("Synthetic target detection vs sea-clutter proxy"); axis.grid(True, alpha=.25); axis.legend(fontsize=8); figure.savefig(output / "detection_physics_join.png", dpi=160); plt.close(figure)
    except Exception:
        pass
    lines = ["# V0.4.122 目标检测边界与海杂波物理代理联表", "", "本阶段将 V0.47 的目标注入检测边界与 V0.120 的海况微元散射/Doppler代理按海况关联。", "", "## 分析方法", "", "先固定海况比较 SNR、距离、速度、方位和俯仰扫描；再查看同一海况的微元数量、散射代理和 Doppler 范围。这样可以避免把目标 SNR 变化误认为海况效应，也避免把微元几何数量直接解释为检测概率。", "", "## 证据边界", "", "目标检测结果来自受控合成谱注入，海杂波字段来自高度场推导代理；两者都不是实测 ADC IQ。不能据此给出实船探测距离、虚警率或目标 RCS 结论。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--boundary", type=Path, required=True); parser.add_argument("--physics", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.boundary.resolve(), args.physics.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
