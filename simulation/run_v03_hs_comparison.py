"""Compare V0.3.3 weighted sea-facet echoes across Hs cases."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt

from simulation.run_v03_weighted_multiscatterer import run_weighted_multi_scatterer


CASES = (("ss2_normal", 0.30), ("ss3_nominal", 0.85), ("ss3_upper", 1.00))


def run_comparison(*, truth_root: Path, results_root: Path, run_id: str) -> Path:
    run_dirs = []
    summaries = []
    for case_id, hs_m in CASES:
        truth = truth_root / f"{case_id}_seed101_truth.h5"
        output = run_weighted_multi_scatterer(
            input_truth=truth, results_root=results_root,
            run_id=f"{run_id}_{case_id}", stride=20, max_scatterers=64,
        )
        run_dirs.append(output)
        summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
        summary["target_hs_m"] = hs_m
        summaries.append(summary)

    comparison_dir = results_root / "comparisons" / run_id
    comparison_dir.mkdir(parents=True, exist_ok=False)
    (comparison_dir / "summary.json").write_text(
        json.dumps(summaries, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4), constrained_layout=True)
    for axis, summary, run_dir in zip(axes, summaries, run_dirs):
        image = plt.imread(run_dir / "figures" / "range_doppler.png")
        axis.imshow(image)
        axis.set_title(f"Hs={summary['target_hs_m']:.2f} m\npeak={summary['peak_range_m']:.2f} m")
        axis.axis("off")
    fig.suptitle("V0.3.4 weighted sea-facet comparison (controlled model)")
    fig.savefig(comparison_dir / "hs_comparison.png", dpi=160)
    plt.close(fig)

    lines = [
        "# V0.3.4 海况对比输出分析", "",
        "## 输入和控制变量", "",
        "本对比固定 MATLAB seed=101、船首向波基线、V0.2 网格、时间索引 0、",
        "散射抽样步长 20、最多 64 个微元、77 GHz/1 GHz FMCW 和 V0.3.3 权重模型。",
        "唯一改变的是目标有效波高 Hs：0.30、0.85、1.00 m。", "",
        "## 图文数据分析方法", "",
        "每个子图横向代表斜距，纵向代表径向速度，颜色越亮代表相对 Range-Doppler 功率越高。",
        "先比较亮区的距离跨度，再比较速度跨度和最强峰位置；表中正面照射比例用于判断"
        "局部几何筛选是否改变了有效散射微元数量。峰值是 FFT 网格最强单元，不是目标真实连续值。", "",
        "| 工况 | Hs (m) | 微元数 | 正面比例 | 距离范围 (m) | 掠射角范围 (°) | 最强峰距 (m) | 最强峰速 (m/s) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in summaries:
        lines.append(
            f"| {item['case_id']} | {item['target_hs_m']:.2f} | {item['scatterer_count']} | "
            f"{item['positive_weight_fraction']:.1%} | "
            f"{item['range_m']['min']:.2f}～{item['range_m']['max']:.2f} | "
            f"{item['grazing_angle_deg']['min']:.2f}～{item['grazing_angle_deg']['max']:.2f} | "
            f"{item['peak_range_m']:.3f} | {item['peak_velocity_mps']:.3f} |"
        )
    lines += [
        "", "## 当前结论和边界", "",
        "本次比较用于验证海况参数进入几何 truth 后，经过同一散射权重模型是否引起"
        "Range-Doppler 图形和统计量变化。若峰值仍落在相同速度 bin，不代表海面没有运动，"
        "而是当前速度分辨率约 0.304 m/s，低于多数海面网格斜距变化率。",
        "当前结果不是绝对海杂波功率、RCS 或探测距离结论；还未加入真实极化、海水介电常数、"
        "Bragg 散射、全局遮挡、天线方向图和平台姿态。下一步应把三工况结果与 DCA1000 原始 IQ"
        "的统计量对齐，再校准散射幅度和 Doppler 模型。", "",
        "图片：`hs_comparison.png`；机器可读汇总：`summary.json`。",
    ]
    (comparison_dir / "output_analysis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return comparison_dir


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--truth-root", type=Path, required=True)
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    output = run_comparison(truth_root=args.truth_root.resolve(), results_root=args.results_root.resolve(), run_id=args.run_id)
    print(f"Generated V0.3.4 Hs comparison into {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
