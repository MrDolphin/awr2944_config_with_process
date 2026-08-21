"""Evaluate PCB-to-radar coordinate-transform candidates with known-angle and sea stress tests."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_41_sea_clutter_aoa import run_case
from simulation.run_v04_86_geometry_comparison import _estimate, _load_candidate
from simulation.v03 import FmcwConfig
from simulation.v04 import generate_aoa_iq_with_positions


def transform_candidates(positions: tuple[np.ndarray, np.ndarray]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    x, y = positions
    cx, cy = float(np.mean(x)), float(np.mean(y))
    x0, y0 = x - cx, y - cy
    return {
        "identity": (x0, y0),
        "mirror_x": (-x0, y0),
        "mirror_y": (x0, -y0),
        "rotate_180": (-x0, -y0),
    }


def known_angle_metrics(config: FmcwConfig, actual: tuple[np.ndarray, np.ndarray], assumed: tuple[np.ndarray, np.ndarray]) -> dict:
    solver_az = np.arange(-60.0, 60.01, 1.0)
    solver_el = np.arange(-20.0, 20.01, 1.0)
    errors = []
    for elevation in np.arange(-20.0, 20.1, 10.0):
        for azimuth in np.arange(-60.0, 60.1, 20.0):
            iq = generate_aoa_iq_with_positions(
                config,
                slant_range_m=30.0,
                radial_velocity_mps=0.0,
                azimuth_deg=float(azimuth),
                elevation_deg=float(elevation),
                x_positions_m=actual[0],
                y_positions_m=actual[1],
            )
            estimated_az, estimated_el, _ = _estimate(config, np.mean(iq, axis=(0, 1)), assumed[0], assumed[1], solver_az, solver_el)
            errors.append((estimated_az - azimuth, estimated_el - elevation))
    values = np.asarray(errors, dtype=float)
    return {
        "azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))),
        "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))),
        "combined_rmse_deg": float(np.sqrt(np.mean(values ** 2))),
        "azimuth_max_abs_deg": float(np.max(np.abs(values[:, 0]))),
        "elevation_max_abs_deg": float(np.max(np.abs(values[:, 1]))),
    }


def _summarize(rows: list[dict]) -> dict:
    az = np.asarray([row["azimuth_error_to_dominant_deg"] for row in rows], dtype=float)
    el = np.asarray([row["elevation_error_to_dominant_deg"] for row in rows], dtype=float)
    jumps = np.asarray([row["angle_jump_deg"] for row in rows[1:]], dtype=float)
    return {
        "frames": len(rows),
        "azimuth_rmse_to_dominant_deg": float(np.sqrt(np.mean(az ** 2))),
        "elevation_rmse_to_dominant_deg": float(np.sqrt(np.mean(el ** 2))),
        "combined_rmse_to_dominant_deg": float(np.sqrt(np.mean(np.concatenate((az, el)) ** 2))),
        "jump_rate_over_5deg": float(np.mean(jumps > 5.0)) if len(jumps) else 0.0,
        "mean_angle_jump_deg": float(np.mean(jumps)) if len(jumps) else 0.0,
    }


def run(candidate_csv: Path, input_root: Path, output: Path) -> dict:
    config = FmcwConfig()
    raw = _load_candidate(candidate_csv)
    candidates = transform_candidates(raw)
    solver_az = np.arange(-60.0, 60.01, 1.0)
    solver_el = np.arange(-20.0, 20.01, 1.0)
    known_rows = []
    sea_rows = []
    paths = sorted(input_root.glob("ss*_seed101.h5"))
    if not paths:
        raise FileNotFoundError(f"no ss*_seed101.h5 under {input_root}")
    for name, assumed in candidates.items():
        known_rows.append({"transform": name, **known_angle_metrics(config, raw, assumed)})
        for index, path in enumerate(paths):
            rows = run_case(path, name, assumed, config, solver_az, solver_el, np.random.default_rng(8900 + index))
            sea_rows.append({"transform": name, "case_id": rows[0]["case_id"], **_summarize(rows)})
    output.mkdir(parents=True, exist_ok=True)
    for filename, rows in (("known_angle_transform_metrics.csv", known_rows), ("sea_clutter_transform_metrics.csv", sea_rows)):
        with (output / filename).open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    best_known = min(known_rows, key=lambda row: row["combined_rmse_deg"])
    summary = {
        "status": "completed_coordinate_transform_stress_screening",
        "candidate_csv": str(candidate_csv.resolve()),
        "input_root": str(input_root.resolve()),
        "transform_count": len(candidates),
        "sea_case_count": len(paths),
        "best_known_angle_transform": best_known["transform"],
        "best_known_angle_combined_rmse_deg": best_known["combined_rmse_deg"],
        "known_angle_rows": known_rows,
        "sea_clutter_rows": sea_rows,
        "coordinate_status": "software_screening_only",
        "hardware_validated": False,
        "measurement_required": "corner_reflector_or_other_known-angle capture",
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(1, 2, figsize=(12, 4), constrained_layout=True)
    labels = [row["transform"] for row in known_rows]
    axes[0].bar(labels, [row["combined_rmse_deg"] for row in known_rows], color="#4472c4")
    axes[0].set_title("Known-angle transform screen")
    axes[0].set_ylabel("combined RMSE (deg)")
    axes[0].tick_params(axis="x", rotation=25)
    sea_by_transform = {name: [] for name in candidates}
    for row in sea_rows:
        sea_by_transform[row["transform"]].append(row["combined_rmse_to_dominant_deg"])
    axes[1].boxplot([sea_by_transform[name] for name in labels], tick_labels=labels)
    axes[1].set_title("Sea-clutter transform sensitivity")
    axes[1].set_ylabel("combined RMSE to dominant facet (deg)")
    axes[1].tick_params(axis="x", rotation=25)
    figure.savefig(output / "coordinate_transform_stress.png", dpi=160)
    plt.close(figure)
    lines = ["# V0.4.89 PCB→雷达坐标变换候选压力测试", "", "## 测试设计", "", "以 V0.4.85 PCB RF 区域虚拟阵列作为原始局部坐标，围绕阵列质心测试 identity、mirror_x、mirror_y、rotate_180 四种方向语义。已知角测试使用同一 PCB 原始坐标生成合成 IQ，再用各候选坐标估计；海杂波测试使用 V0.2 动态海面高度真值、相同海况和固定随机种子。", "", "## 已知角结果", "", "| 候选 | 方位 RMSE(°) | 俯仰 RMSE(°) | 综合 RMSE(°) |", "|---|---:|---:|---:|"]
    lines.extend(f"| {row['transform']} | {row['azimuth_rmse_deg']:.3f} | {row['elevation_rmse_deg']:.3f} | {row['combined_rmse_deg']:.3f} |" for row in known_rows)
    lines += ["", f"已知角合成测试中综合 RMSE 最小的是 `{best_known['transform']}`。这只说明在当前合成模型中该候选与生成坐标最一致，不能证明真实 PCB 安装方向。", "", "## 海杂波结果如何阅读", "", "- `sea_clutter_transform_metrics.csv` 每行对应一个坐标候选和一个海况。综合 RMSE 是 AoA 主峰相对当前最强海面微元的偏差，不是固定目标定位误差。", "- `jump_rate_over_5deg` 是相邻帧角跳变超过 5° 的比例，用来比较坐标解释对海杂波角度稳定性的敏感性。", "- 四个候选必须在同一海况、同一随机种子下比较；不能用不同随机海面或不同噪声实现的结果直接下结论。", "", "## 工程结论与边界", "", "本阶段只能筛查坐标符号/镜像的影响，不能选择真实硬件变换。要冻结坐标，仍需角反射器或已知方位目标采集；同时还需要 TI 校准、天线方向图和电气相位中心。当前输入是 V0.2 海面高度真值与合成通道，不是 DCA1000 实测 IQ。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-csv", type=Path, required=True)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.candidate_csv.resolve(), args.input_root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
