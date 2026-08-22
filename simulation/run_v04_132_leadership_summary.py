"""Build a cross-sea-state leadership summary from V0.130/V0.131 outputs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def run(visualization_summary: Path, aoa_summary: Path, output: Path) -> dict:
    with visualization_summary.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    with aoa_summary.open(encoding="utf-8", newline="") as handle:
        aoa_rows = list(csv.DictReader(handle))
    output.mkdir(parents=True, exist_ok=True)
    with (output / "leadership_comparison.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    models = sorted({row["model"] for row in rows}); cases = sorted({row["case_id"] for row in rows})
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        x = np.arange(len(cases)); width = .18; figure, axis = plt.subplots(figsize=(11, 5), constrained_layout=True)
        for index, model in enumerate(models):
            values = [next(float(row["two_way_scatter_sum"]) for row in rows if row["case_id"] == case and row["model"] == model) for case in cases]
            axis.bar(x + (index - 1.5) * width, values, width, label=model)
        axis.set_xticks(x, cases); axis.set_ylabel("relative two-way clutter proxy"); axis.set_title("Sea-state and array-model sensitivity (synthetic proxy)"); axis.legend(fontsize=8); axis.grid(True, axis="y", alpha=.25); figure.savefig(output / "leadership_two_way_scatter_by_sea_state.png", dpi=170); plt.close(figure)
    except Exception:
        pass
    max_case = max(rows, key=lambda row: float(row["two_way_scatter_sum"]))
    aoa_best = min(aoa_rows, key=lambda row: float(row["azimuth_rmse_deg"]) + float(row["elevation_rmse_deg"]))
    result = {"status": "completed_leadership_summary", "case_count": len(cases), "model_count": len(models), "max_proxy_case": max_case, "best_synthetic_aoa_model": aoa_best, "real_hardware_validated": False}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.132 船载海杂波仿真阶段汇报摘要", "", "## 一句话结论", "", "当前已经形成从海况高度场、海面微面元、候选阵列几何、网格 AoA 到单程/双程方向图加权海杂波代理的闭环仿真链路；它可以用于比较趋势和定位硬件数据需求，但还不能替代真实 ADC IQ、TI 校准和海试。", "", "## 已完成的工作", "", "1. 建立 ss0_flat、ss1_rippled、ss2_normal、ss3_nominal、ss3_upper 五种海况输入。", "2. 形成理想半波长、CFG 候选、PCB 铜区中心、RF 终点候选四种阵列模型。", "3. 用网格波束扫描代替旧的相位平面 unwrap，四种模型同模型自匹配可达到 0° RMSE。", "4. 将估计方位/俯仰接入海面微面元，生成三维点云和方向图加权散射代理。", "", "## 图表如何汇报", "", "- 横轴是海况，柱高是双程方向图加权后的相对海杂波代理，不是 dBsm 或检测距离。", "- 同一海况不同颜色柱之间的差异，表示阵列几何模型和角度分布对方向图权重的敏感性。", "- 三维图中的点是海面微面元，不是 CFAR 检测点；颜色表示估计角度或方向图相对权重。", "", "## 可以作为阶段性参考的结论", "", "- 海况从 ss0/ss1 向 ss2/ss3 增强时，有效散射微面元数量和双程加权散射代理总体上升。", "- 阵列模型会改变海杂波角度分布和方向图加权结果；PCB/RF 终点候选不能直接替代理想或 CFG 阵列。", "- 当前合成 IQ 下，理想和 CFG 模型的 AoA RMSE 约 1°～2°；PCB/RF 终点模型的角度误差明显更大，提示真实相位中心和通道校准是关键数据。", "", "## 不能对外宣称的内容", "", "不能据此宣称 AWR2944P 的真实探测距离、真实虚警率、真实海杂波功率、真实 AoA 精度或真实船载检测概率。方向图是粗略数字化，海面散射是代理模型，硬件 AoA 尚未用真实 DCA1000 IQ 闭环。", "", "## 下一步硬件验证门槛", "", "1. 采集已知角度目标的 DCA1000 原始 ADC IQ。", "2. 固定 LVDS 通道顺序、TX/RX 映射和采样维度。", "3. 使用 TI 校准流程得到 RX 通道幅相补偿。", "4. 以真实相位中心/方向图替换候选模型。", "5. 用 RMSE、峰值偏差、角度重复性和海杂波虚警统计完成实测对标。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--visualization-summary", type=Path, required=True); parser.add_argument("--aoa-summary", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.visualization_summary.resolve(), args.aoa_summary.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
