"""Compare V0.3.5 Doppler resolution using 64 and 256 chirps."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from simulation.run_v03_weighted_multiscatterer import run_weighted_multi_scatterer


def run_comparison(*, input_truth: Path, results_root: Path, run_id: str) -> Path:
    outputs = []
    for chirps in (64, 256):
        out = run_weighted_multi_scatterer(
            input_truth=input_truth, results_root=results_root,
            run_id=f"{run_id}_chirps{chirps}", stride=20, max_scatterers=64,
            chirps_per_frame=chirps,
        )
        outputs.append(json.loads((out / "summary.json").read_text(encoding="utf-8")))
    comparison = results_root / "comparisons" / run_id
    comparison.mkdir(parents=True, exist_ok=False)
    (comparison / "summary.json").write_text(json.dumps(outputs, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# V0.3.5 Doppler 分辨率对比分析", "",
        "输入海况、海面 truth、散射抽样、权重模型和 FMCW 带宽均固定，唯一改变的是每帧慢时间 Chirp 数。",
        "64 Chirp 的速度分辨率约 0.304 m/s，256 Chirp 约 0.076 m/s；因此本对比用于区分"
        "‘海面运动确实很小’与‘FFT 分辨率不足’。", "",
        "| Chirp 数 | 速度分辨率 (m/s) | 最强峰距离 (m) | 最强峰速度 (m/s) |",
        "|---:|---:|---:|---:|",
    ]
    for item in outputs:
        lines.append(f"| {item['chirps_per_frame']} | {item['velocity_resolution_mps']:.6f} | {item['peak_range_m']:.3f} | {item['peak_velocity_mps']:.6f} |")
    lines += [
        "", "## 结论边界", "",
        "如果 256 Chirp 仍然显示速度峰接近 0 m/s，只能说明当前抽取微元的 Eulerian 斜距变化率"
        "在该观测时窗内仍较小；它不是水质点轨道速度，也不是实测 Doppler 杂波谱。"
        "若速度亮区变窄或出现非零 bin，则说明原先 64 Chirp 主要受分辨率限制。",
        "增加 Chirp 数会增加帧时长（256×100 µs=25.6 ms），真实船体运动期间可能引入姿态变化，"
        "所以后续必须同时比较 CPI 时长、平台运动和频率稳定性。", "",
        "每个子运行目录均保存独立 HDF5、Range-Doppler 图和 output_analysis.md。",
    ]
    (comparison / "output_analysis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return comparison


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-truth", type=Path, required=True)
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    print(run_comparison(input_truth=args.input_truth.resolve(), results_root=args.results_root.resolve(), run_id=args.run_id))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
