"""Join synthetic CFAR sweep results with masked microfacet physics summaries."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def run(cfar_csv: Path, physics_csv: Path, output: Path) -> dict:
    with cfar_csv.open(encoding="utf-8", newline="") as handle:
        cfar = list(csv.DictReader(handle))
    with physics_csv.open(encoding="utf-8", newline="") as handle:
        physics = {row["case_id"]: row for row in csv.DictReader(handle)}
    joined = []
    for row in cfar:
        p = physics.get(row["case_id"])
        if p is None:
            continue
        joined.append({**row, "three_db_facets": p["three_db_facets"], "six_db_facets": p["six_db_facets"], "mean_three_db_scatter_proxy": p["mean_three_db_scatter_proxy"], "three_db_doppler_min_hz": p["three_db_doppler_min_hz"], "three_db_doppler_max_hz": p["three_db_doppler_max_hz"], "rms_three_db_radial_velocity_mps": p["rms_three_db_radial_velocity_mps"]})
    if not joined:
        raise ValueError("no common case_id between CFAR and physics summaries")
    output.mkdir(parents=True, exist_ok=True)
    with (output / "cfar_physics_join.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(joined[0])); writer.writeheader(); writer.writerows(joined)
    selected = [row for row in joined if row["scenario_id"] == "pfa1e3_train4"]
    summary = {"status": "completed_cfar_physics_join", "cfar_source": str(cfar_csv.resolve()), "physics_source": str(physics_csv.resolve()), "row_count": len(joined), "selected_scenario": "pfa1e3_train4", "selected_rows": len(selected), "hardware_aoa_validated": False, "interpretation": "descriptive join of synthetic CFAR and physics proxies; no causal detection claim"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        figure, axis = plt.subplots(figsize=(7, 4), constrained_layout=True)
        axis.scatter([float(row["three_db_facets"]) for row in selected], [float(row["point_count"]) / float(row["frames"]) for row in selected], s=50)
        for row in selected: axis.annotate(row["case_id"], (float(row["three_db_facets"]), float(row["point_count"]) / float(row["frames"])), fontsize=8)
        axis.set_xlabel("3 dB masked microfacet count (all frames)"); axis.set_ylabel("CFAR detections/frame"); axis.set_title("Synthetic CFAR vs masked physics (pfa=1e-3, train=4x4)"); axis.grid(True, alpha=.25); figure.savefig(output / "cfar_physics_join.png", dpi=160); plt.close(figure)
    except Exception:
        pass
    lines = ["# V0.4.121 CFAR 与微元物理代理联表", "", "本阶段将 V0.45 的合成 CFAR 参数扫描结果与 V0.120 的波束掩膜微元统计按海况 case_id 关联。", "", "## 如何分析", "", "先在同一海况内比较不同 CFAR 场景，判断点数变化是否由门限/训练窗造成；再固定 `pfa1e3_train4`，横向比较海况与 3 dB 微元数、散射代理、Doppler 代理。图中点数/帧是 CFAR 输出候选，不是实测虚警率。", "", "## 证据边界", "", "两侧均为合成模型：CFAR 输入来自 V0.43 合成 Range-Doppler，物理字段来自 V0.42/V0.120 高度场推导。联表只用于解释变量关系和发现下一步实验设计，不代表因果关系或 AWR2944P 性能。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--cfar", type=Path, required=True); parser.add_argument("--physics", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.cfar.resolve(), args.physics.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
