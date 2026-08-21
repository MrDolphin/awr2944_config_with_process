"""Generate presentation-ready plots from the validated V0.48/V0.49/V0.54 CSV outputs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read_csv(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def run(physical_csv: Path, overlap_csv: Path, order_csv: Path, output: Path) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    physical = read_csv(physical_csv); overlap = read_csv(overlap_csv); order = read_csv(order_csv); output.mkdir(parents=True, exist_ok=True)
    # Figure 1: target detection vs Doppler velocity at low SNR.
    fig, ax = plt.subplots(figsize=(9, 5))
    for case in sorted({row["case_id"] for row in overlap}):
        rows = [row for row in overlap if row["case_id"] == case and row["target_snr_db"] == "5.0" and row["pfa"] == "0.01"]
        rows.sort(key=lambda row: float(row["target_velocity_truth_mps"])); ax.plot([float(row["target_velocity_truth_mps"]) for row in rows], [float(row["detection_probability"]) for row in rows], marker="o", label=case)
    ax.set_xlabel("Target radial velocity (m/s)"); ax.set_ylabel("Detection probability"); ax.set_ylim(-0.05, 1.05); ax.grid(True, alpha=0.3); ax.legend(); ax.set_title("V0.49 Low-SNR target vs sea-clutter Doppler"); fig.tight_layout(); fig.savefig(output / "doppler_overlap_detection.png", dpi=180); plt.close(fig)
    # Figure 2: physical relative range attenuation.
    fig, ax = plt.subplots(figsize=(9, 5))
    for case in sorted({row["case_id"] for row in physical}):
        for snr in ("0.0", "5.0", "10.0", "15.0"):
            rows = [row for row in physical if row["case_id"] == case and row["target_snr_db"] == snr and row["pfa"] == "0.01"]; rows.sort(key=lambda row: float(row["target_range_truth_m"]))
            ax.plot([float(row["target_range_truth_m"]) for row in rows], [float(row["detection_probability"]) for row in rows], marker="o", label=f"{case}, {snr} dB")
    ax.set_xlabel("Target range (m)"); ax.set_ylabel("Detection probability"); ax.set_ylim(-0.05, 1.05); ax.grid(True, alpha=0.3); ax.legend(fontsize=8, ncol=2); ax.set_title("V0.48 relative sigma/R^4 range boundary"); fig.tight_layout(); fig.savefig(output / "physical_range_detection.png", dpi=180); plt.close(fig)
    # Figure 3: multi-scene channel candidate rank.
    order_sorted = sorted(order, key=lambda row: float(row["combined_rmse_deg"]))[:20]; fig, ax = plt.subplots(figsize=(10, 5)); labels = [f"{row['rx_order']}|{row['tx_order']}" for row in order_sorted]; values = [float(row["combined_rmse_deg"]) for row in order_sorted]; ax.bar(np.arange(len(values)), values, color=["#1f77b4" if row["identity_order"] == "True" else "#ff7f0e" for row in order_sorted]); ax.set_xticks(np.arange(len(values))); ax.set_xticklabels(labels, rotation=75, ha="right", fontsize=7); ax.set_ylabel("Combined AoA RMSE (deg)"); ax.set_title("V0.54 multi-scene RX/TX candidate ranking"); ax.grid(axis="y", alpha=0.3); fig.tight_layout(); fig.savefig(output / "channel_order_rmse_rank.png", dpi=180); plt.close(fig)
    summary = {"status": "completed_v057_report_figures", "inputs": {"physical_csv": str(physical_csv.resolve()), "overlap_csv": str(overlap_csv.resolve()), "order_csv": str(order_csv.resolve())}, "figures": ["doppler_overlap_detection.png", "physical_range_detection.png", "channel_order_rmse_rank.png"], "source_status": "synthetic_and_relative_models_not_hardware_measurement"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.57 仿真结果汇报图", "", "本目录将 V0.48 距离衰减、V0.49 Doppler 重叠和 V0.54 通道候选结果生成统一图片。", "", "## 图片分析", "", "### doppler_overlap_detection.png", "横轴是目标径向速度，纵轴是检测概率。5 dB 低 SNR 下，速度接近海杂波集中区域时曲线出现局部上升/下降，说明 Doppler 分布会影响目标判决；这仍是合成海杂波趋势。", "", "### physical_range_detection.png", "横轴是距离，纵轴是检测概率。曲线体现相对 σ/R⁴ 衰减；距离越远，目标幅度越低。不同 SNR 曲线用于观察检测边界，不能直接当作真实最大探测距离。", "", "### channel_order_rmse_rank.png", "横轴是 RX/TX 排列候选，纵轴是多场景联合 AoA RMSE。蓝色身份排列和橙色错误排列用于比较。错误排列偶然接近并不等于硬件顺序已确认。", "", "## 使用边界", "", "输入来自合成/相对物理模型，未使用实测 DCA1000 IQ；图片适合说明模型趋势和算法接口，不适合宣称实板探测距离或真实 AoA 精度。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8"); return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--physical-csv", type=Path, required=True); parser.add_argument("--overlap-csv", type=Path, required=True); parser.add_argument("--order-csv", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.physical_csv.resolve(), args.overlap_csv.resolve(), args.order_csv.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
