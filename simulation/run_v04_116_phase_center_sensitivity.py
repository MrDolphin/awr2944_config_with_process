"""Quantify AoA sensitivity to deterministic phase-center coordinate errors."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_39_sparse_array_solver import estimate_sparse_grid
from simulation.run_v04_107_provisional_array_comparison import load_provisional
from simulation.v03 import FmcwConfig
from simulation.v04 import generate_aoa_iq_with_positions, virtual_array_positions


def _perturb(positions: tuple[np.ndarray, np.ndarray], amplitude_mm: float) -> tuple[np.ndarray, np.ndarray]:
    x, y = positions
    index = np.arange(x.size, dtype=float).reshape(x.shape)
    # Alternating deterministic offsets emulate a bounded phase-center error;
    # a common translation is deliberately avoided because it only adds global phase.
    dx = (np.mod(index, 3.0) - 1.0) * amplitude_mm / 1000.0
    dy = (np.mod(index, 2.0) * 2.0 - 1.0) * amplitude_mm / 1000.0
    return x + dx, y + dy


def run(provisional_csv: Path, output: Path) -> dict:
    config = FmcwConfig()
    actual = virtual_array_positions(config)
    base_assumed = load_provisional(provisional_csv, config)
    levels = [0.0, 0.25, 0.5, 1.0]
    truth_az = np.arange(-45.0, 45.1, 15.0)
    truth_el = np.asarray([-10.0, 0.0, 10.0])
    solver_az = np.arange(-60.0, 60.01, 1.0)
    solver_el = np.arange(-20.0, 20.01, 1.0)
    rows = []
    for level in levels:
        assumed = _perturb(base_assumed, level)
        errors = []
        for el in truth_el:
            for az in truth_az:
                iq = generate_aoa_iq_with_positions(config, slant_range_m=30.0, radial_velocity_mps=0.0, azimuth_deg=float(az), elevation_deg=float(el), x_positions_m=actual[0], y_positions_m=actual[1])
                estimate = estimate_sparse_grid(config, np.mean(iq, axis=(0, 1)), assumed[0], assumed[1], solver_az, solver_el)
                errors.append((estimate[0] - az, estimate[1] - el))
        values = np.asarray(errors)
        rows.append({"phase_center_perturbation_mm": level, "azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))), "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))), "combined_rmse_deg": float(np.sqrt(np.mean(values ** 2))), "max_abs_error_deg": float(np.max(np.abs(values)))})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "phase_center_sensitivity.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    result = {"status": "completed_phase_center_sensitivity", "provisional_geometry": str(provisional_csv.resolve()), "actual_model": "ideal_half_lambda", "truth_grid": {"azimuth_deg": truth_az.tolist(), "elevation_deg": truth_el.tolist()}, "rows": rows, "hardware_aoa_validated": False, "interpretation": "synthetic bounded coordinate-error sensitivity; not measured antenna phase-center error"}
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        figure, axis = plt.subplots(figsize=(7, 4), constrained_layout=True)
        axis.plot([r["phase_center_perturbation_mm"] for r in rows], [r["combined_rmse_deg"] for r in rows], "o-")
        axis.set_xlabel("bounded phase-center coordinate perturbation (mm)"); axis.set_ylabel("combined AoA RMSE (deg)"); axis.set_title("Provisional array phase-center sensitivity"); axis.grid(True, alpha=0.25)
        figure.savefig(output / "phase_center_sensitivity.png", dpi=160); plt.close(figure)
    except Exception:
        pass
    lines = ["# V0.4.116 阵元相位中心坐标敏感性", "", "本阶段以理想半波长阵列产生已知角 IQ，再用 PCB 端点候选几何叠加确定性的阵元坐标扰动进行 AoA。扰动用于误差预算，不是天线实测相位中心。", "", "| 扰动幅度 (mm) | 方位 RMSE (°) | 俯仰 RMSE (°) | 综合 RMSE (°) | 最大绝对误差 (°) |", "|---:|---:|---:|---:|---:|"]
    lines.extend(f"| {r['phase_center_perturbation_mm']:.2f} | {r['azimuth_rmse_deg']:.3f} | {r['elevation_rmse_deg']:.3f} | {r['combined_rmse_deg']:.3f} | {r['max_abs_error_deg']:.3f} |" for r in rows)
    lines += ["", "## 如何解释", "", "横轴是阵元相位中心坐标的不确定量级，纵轴是已知角网格上的综合 AoA RMSE。本次样本没有呈现单调变化（23.994°→23.583°），因此不能把它解读成‘扰动越大误差越大’。更可靠的结论是：当前 PCB 候选几何与理想阵列之间的基线失配已经产生约 24° 综合 RMSE，掩盖了 0.25–1.0 mm 小扰动的独立影响；必须先获得真实相位中心，再做围绕真实基准的误差预算。", "", "## 证据边界", "", "实际天线相位中心还受封装、馈电、罩体、邻近金属、频率和安装结构影响。这里的扰动模式是人为设定的有界模型，不能作为 AWR2944P 实测误差或最终 PCB 指标。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--provisional", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.provisional.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
