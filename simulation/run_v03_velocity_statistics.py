"""Run V0.3.6 velocity-spectrum statistics across three Hs cases."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v03_weighted_multiscatterer import run_weighted_multi_scatterer


CASES = (("ss2_normal", 0.30), ("ss3_nominal", 0.85), ("ss3_upper", 1.00))


def _stats(run_dir: Path, hs_m: float) -> dict[str, float | str]:
    with h5py.File(run_dir / "data" / "weighted_multi_scatterer.h5", "r") as handle:
        power = np.asarray(handle["/range_doppler/power_linear"], dtype=float)
        velocity = np.asarray(handle["/axes/velocity_mps"], dtype=float)
        ranges = np.asarray(handle["/axes/range_m"], dtype=float)
    spectrum = np.sum(power, axis=1)
    spectrum = np.maximum(spectrum, 0.0)
    total = float(np.sum(spectrum))
    if total <= 0.0:
        raise ValueError("velocity spectrum has no positive power")
    centroid = float(np.sum(velocity * spectrum) / total)
    rms = float(np.sqrt(np.sum(np.square(velocity - centroid) * spectrum) / total))
    peak = float(np.max(spectrum))
    above = velocity[spectrum >= peak / 2.0]
    width_3db = float(np.max(above) - np.min(above)) if above.size else 0.0
    near = float(np.sum(power[:, ranges <= 30.0]))
    far = float(np.sum(power[:, ranges >= 60.0]))
    return {
        "case_id": run_dir.name,
        "target_hs_m": hs_m,
        "velocity_resolution_mps": float(abs(velocity[1] - velocity[0])),
        "peak_velocity_mps": float(velocity[int(np.argmax(spectrum))]),
        "velocity_centroid_mps": centroid,
        "velocity_rms_mps": rms,
        "velocity_width_3db_mps": width_3db,
        "near_0_30m_power_linear": near,
        "far_60_150m_power_linear": far,
        "near_far_power_ratio_db": float(10.0 * np.log10(max(near, 1e-30) / max(far, 1e-30))),
    }


def run_statistics(*, truth_root: Path, results_root: Path, run_id: str) -> Path:
    rows = []
    for case_id, hs_m in CASES:
        output = run_weighted_multi_scatterer(
            input_truth=truth_root / f"{case_id}_seed101_truth.h5",
            results_root=results_root, run_id=f"{run_id}_{case_id}",
            stride=20, max_scatterers=64, chirps_per_frame=256,
        )
        rows.append(_stats(output, hs_m))
    comparison = results_root / "comparisons" / run_id
    comparison.mkdir(parents=True, exist_ok=False)
    (comparison / "summary.json").write_text(json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# V0.3.6 海况速度谱统计分析", "",
        "固定 256 Chirp、seed=101、抽样网格、权重模型和 FMCW 参数，仅改变 Hs。",
        "速度谱由 Range-Doppler 功率沿距离维求和得到；质心和 RMS 为功率加权统计，3 dB 宽度"
        "是速度谱高于峰值一半的连续网格跨度。近/远功率比用于观察距离衰减下的能量分布。", "",
        "| 工况 | Hs (m) | 峰值速度 | 速度质心 | 速度 RMS | 3 dB 宽度 | 近/远功率比 |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['case_id']} | {row['target_hs_m']:.2f} | {row['peak_velocity_mps']:.6f} | "
            f"{row['velocity_centroid_mps']:.6f} | {row['velocity_rms_mps']:.6f} | "
            f"{row['velocity_width_3db_mps']:.6f} | {row['near_far_power_ratio_db']:.2f} dB |"
        )
    lines += [
        "", "## 如何解释", "",
        "速度 RMS 或 3 dB 宽度增大，才可以作为‘速度维杂波展宽’的候选证据；单个峰值移动不能单独证明海况引起了 Doppler 展宽。",
        "若三种海况统计量接近，可能是当前 Eulerian 斜距变化率、散射点抽样和相干叠加模型限制，而不是海面物理没有差异。",
        "这些指标仍不是水质点轨道速度、绝对 RCS 或实测海杂波谱。下一步需要加入非相干散射相位、真实散射系数并与 DCA1000 IQ 统计对齐。", "",
        "每个工况的独立运行目录包含 HDF5、Range-Doppler 图和 output_analysis.md；本目录保存跨工况汇总。",
    ]
    (comparison / "output_analysis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return comparison


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--truth-root", type=Path, required=True)
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    print(run_statistics(truth_root=args.truth_root.resolve(), results_root=args.results_root.resolve(), run_id=args.run_id))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
