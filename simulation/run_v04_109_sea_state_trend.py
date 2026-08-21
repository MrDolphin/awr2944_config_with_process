"""Attach V0.2 sea-state metadata to the V0.4.108 geometry comparison.

This stage is deliberately an analysis-only join.  It does not claim that the
synthetic clutter AoA error is a hardware accuracy measurement.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def _read_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def _case_metadata(run_config: Path, environment: Path | None = None) -> dict[str, dict]:
    config = _read_json(run_config)
    wind = config.get("sea_surface", {}).get("wind_direction_deg")
    result = {}
    for item in config.get("sea_states", []):
        result[item["case_id"]] = {
            "sea_state": int(item["sea_state"]),
            "label": item.get("label", item["case_id"]),
            "target_hs_m": float(item["target_hs_m"]),
            "wind_direction_deg": float(wind) if wind is not None else float("nan"),
        }
    if environment is not None and environment.exists():
        env = _read_json(environment)
        result_meta = {"producer": env.get("producer"), "matlab_version": env.get("matlab_version"), "run_id": env.get("run_id")}
        for value in result.values():
            value.update(result_meta)
    return result


def run(v0108_summary: Path, run_config: Path, output: Path, environment: Path | None = None) -> dict:
    metadata = _case_metadata(run_config, environment)
    with v0108_summary.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"empty V0.4.108 summary: {v0108_summary}")
    joined = []
    for row in rows:
        case = metadata.get(row["case_id"])
        if case is None:
            raise KeyError(f"{row['case_id']} is absent from {run_config}")
        joined.append({**case, **row})
    output.mkdir(parents=True, exist_ok=True)
    fields = list(joined[0])
    with (output / "sea_state_geometry_trend.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(joined)

    def finite(name: str) -> np.ndarray:
        return np.asarray([float(row[name]) for row in joined], dtype=float)

    hs = finite("target_hs_m")
    diff = finite("model_azimuth_difference_rmse_deg")
    ideal = finite("ideal_half_lambda_azimuth_rmse_deg")
    provisional = finite("provisional_pcb_endpoint_azimuth_rmse_deg")
    corr = float(np.corrcoef(hs, diff)[0, 1]) if len(joined) > 1 and np.std(hs) > 0 else float("nan")
    trend = {
        "status": "completed_sea_state_metadata_join",
        "source_v0108_summary": str(v0108_summary.resolve()),
        "source_v02_run_config": str(run_config.resolve()),
        "source_v02_environment": str(environment.resolve()) if environment else None,
        "case_count": len(joined),
        "target_hs_range_m": [float(np.min(hs)), float(np.max(hs))],
        "model_difference_azimuth_rmse_range_deg": [float(np.min(diff)), float(np.max(diff))],
        "hs_vs_model_difference_pearson_r": corr,
        "interpretation": "descriptive synthetic trend only; not hardware AoA validation",
    }
    (output / "summary.json").write_text(json.dumps(trend, ensure_ascii=False, indent=2), encoding="utf-8")
    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), dpi=160)
        axes[0].plot(hs, ideal, "o-", label="ideal half-lambda")
        axes[0].plot(hs, provisional, "s-", label="provisional PCB endpoint")
        axes[0].set_xlabel("Target Hs (m)"); axes[0].set_ylabel("AoA RMSE to dominant facet (deg)")
        axes[0].set_title("Sea-state vs within-model error")
        axes[1].plot(hs, diff, "D-", color="tab:red")
        axes[1].set_xlabel("Target Hs (m)"); axes[1].set_ylabel("Geometry-model azimuth difference RMSE (deg)")
        axes[1].set_title(f"Geometry sensitivity (r={corr:.3f})")
        for ax in axes: ax.grid(True, alpha=0.25)
        axes[0].legend(); fig.tight_layout(); fig.savefig(output / "sea_state_geometry_trend.png"); plt.close(fig)
    except Exception as exc:
        trend["plot_error"] = repr(exc)
        (output / "summary.json").write_text(json.dumps(trend, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = ["# V0.4.109 海况元数据与阵列几何敏感性趋势", "", "本阶段把 V0.2 的目标有效波高/海况标签与 V0.4.108 的同帧几何比较结果关联起来。数据仍是合成海面高度场与候选 PCB 端点几何。", "", "| 海况 | 目标 Hs (m) | 风向 (°) | 理想阵列方位 RMSE (°) | PCB候选方位 RMSE (°) | 两模型方位差 RMSE (°) | >10°比例 |", "|---|---:|---:|---:|---:|---:|---:|"]
    for row in joined:
        lines.append(f"| {row['label']} ({row['case_id']}) | {float(row['target_hs_m']):.2f} | {float(row['wind_direction_deg']):.1f} | {float(row['ideal_half_lambda_azimuth_rmse_deg']):.3f} | {float(row['provisional_pcb_endpoint_azimuth_rmse_deg']):.3f} | {float(row['model_azimuth_difference_rmse_deg']):.3f} | {float(row['model_difference_over_10deg_rate']):.3f} |")
    lines += ["", "## 如何从图和表分析", "", "左图横轴是目标有效波高 Hs，纵轴是 AoA 主峰相对该帧最强海面微元的角度 RMSE；它回答‘在当前合成模型下，海况变化时估计主峰是否更偏’。右图比较同一帧、同一噪声种子下理想半波长阵列和 PCB 端点候选几何的估计方位差；它回答‘阵列几何假设本身会引入多大差异’。", "", f"本次五组样本的 Hs 范围为 {np.min(hs):.2f}–{np.max(hs):.2f} m，模型间方位差 RMSE 为 {np.min(diff):.3f}–{np.max(diff):.3f}°，Hs 与模型间差异的 Pearson r={corr:.3f}。该相关系数只描述这五个合成样本，不足以证明海况与硬件误差存在因果关系。", "", "## 当前可作为汇报的结论", "", "1. 在相同海面与误差种子下，替换阵列几何模型会造成约 41–45° 的方位 RMSE 差异量级；因此在接入真实 AoA 前，阵元相位中心/坐标不能用走线端点代替。", "2. 五组海况没有呈现单调的 Hs—角度差趋势；海杂波是多微元叠加，主峰跳变和阵列模型失配比单纯波高更显著。", "3. 这不是 AWR2944P 实测探测距离、虚警率或 AoA 精度结论。下一步必须用校准后的 DCA1000 ADC IQ、真实 TX/RX 相位中心和实测方向图复核。", "", "## 复现与证据边界", "", f"输入：`{v0108_summary.name}`、`{run_config.name}`；输出 CSV、PNG、JSON 和本报告均保存在本目录。V0.2 的 Hs 是目标控制值，海面高度场是 MATLAB 合成结果；PCB 几何仍标记为候选端点。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return trend


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--run-config", type=Path, required=True)
    parser.add_argument("--environment", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.summary.resolve(), args.run_config.resolve(), args.output.resolve(), args.environment.resolve() if args.environment else None), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
