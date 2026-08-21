"""Compare ideal and provisional PCB endpoint virtual-array geometries.

The provisional model is a sensitivity model only.  It is not a measured
antenna phase-center model and cannot validate TI hardware AoA.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_39_sparse_array_solver import estimate_sparse_grid
from simulation.v03 import FmcwConfig
from simulation.v04 import generate_aoa_iq_with_positions


def load_provisional(path: Path, config: FmcwConfig) -> tuple[np.ndarray, np.ndarray]:
    tx: dict[str, tuple[float, float]] = {}
    rx: dict[str, tuple[float, float]] = {}
    with path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            point = (float(row["x_relative_mm"]) / 1000.0, float(row["y_relative_mm"]) / 1000.0)
            if row["kind"] == "TX":
                tx[row["channel"]] = point
            else:
                rx[row["channel"]] = point
    expected = {f"TX{i}" for i in range(1, 5)} | {f"RX{i}" for i in range(1, 5)}
    if set(tx) | set(rx) != expected:
        raise ValueError(f"expected TX1..TX4 and RX1..RX4, got {sorted(set(tx) | set(rx))}")
    x = np.zeros((4, 4), dtype=float)
    y = np.zeros((4, 4), dtype=float)
    for rx_index in range(4):
        for tx_index in range(4):
            x[rx_index, tx_index] = tx[f"TX{tx_index + 1}"][0] + rx[f"RX{rx_index + 1}"][0]
            y[rx_index, tx_index] = tx[f"TX{tx_index + 1}"][1] + rx[f"RX{rx_index + 1}"][1]
    return x, y


def model_positions(provisional: Path, config: FmcwConfig) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    d = config.wavelength_m / 2.0
    ideal = (np.repeat((np.arange(4) * d)[None, :], 4, axis=0), np.repeat((np.arange(4) * d)[:, None], 4, axis=1))
    return {"ideal_half_lambda": ideal, "provisional_pcb_endpoint": load_provisional(provisional, config)}


def steering(config: FmcwConfig, x: np.ndarray, y: np.ndarray, azimuth_deg: np.ndarray, elevation_deg: np.ndarray) -> np.ndarray:
    az, el = np.meshgrid(np.deg2rad(azimuth_deg), np.deg2rad(elevation_deg), indexing="ij")
    phase = (2.0 * np.pi / config.wavelength_m) * (x.ravel()[None, None, :] * np.sin(az)[..., None] * np.cos(el)[..., None] + y.ravel()[None, None, :] * np.sin(el)[..., None])
    return np.exp(1j * phase).reshape(-1, 16) / np.sqrt(16.0)


def compare(provisional: Path, output: Path) -> dict:
    config = FmcwConfig()
    models = model_positions(provisional, config)
    truth_az = np.arange(-60.0, 60.1, 20.0)
    truth_el = np.arange(-20.0, 20.1, 10.0)
    solver_az = np.arange(-60.0, 60.01, 1.0)
    solver_el = np.arange(-20.0, 20.01, 1.0)
    metrics, details = [], []
    for actual_name, actual in models.items():
        for assumed_name, assumed in models.items():
            errors = []
            for elevation in truth_el:
                for azimuth in truth_az:
                    iq = generate_aoa_iq_with_positions(config, slant_range_m=30.0, radial_velocity_mps=0.0, azimuth_deg=float(azimuth), elevation_deg=float(elevation), x_positions_m=actual[0], y_positions_m=actual[1])
                    estimate = estimate_sparse_grid(config, np.mean(iq, axis=(0, 1)), assumed[0], assumed[1], solver_az, solver_el)
                    row = {"actual_model": actual_name, "assumed_model": assumed_name, "truth_azimuth_deg": float(azimuth), "truth_elevation_deg": float(elevation), "estimated_azimuth_deg": estimate[0], "estimated_elevation_deg": estimate[1], "score": estimate[2], "azimuth_error_deg": estimate[0] - azimuth, "elevation_error_deg": estimate[1] - elevation}
                    details.append(row)
                    errors.append((row["azimuth_error_deg"], row["elevation_error_deg"]))
            values = np.asarray(errors)
            metrics.append({"actual_model": actual_name, "assumed_model": assumed_name, "azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))), "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))), "combined_rmse_deg": float(np.sqrt(np.mean(values ** 2))), "max_abs_error_deg": float(np.max(np.abs(values)))})

    beam_az = np.arange(-90.0, 90.01, 0.5)
    beam_rows = []
    for name, (x, y) in models.items():
        response = np.abs(steering(config, x, y, beam_az, np.asarray([0.0])) @ steering(config, x, y, np.asarray([0.0]), np.asarray([0.0])).conj().T[:, 0]) ** 2
        response_db = 10.0 * np.log10(np.maximum(response / np.max(response), 1e-12))
        beam_rows.extend({"model": name, "azimuth_deg": float(angle), "response_db": float(value)} for angle, value in zip(beam_az, response_db))
    output.mkdir(parents=True, exist_ok=True)
    for filename, rows in (("pairwise_metrics.csv", metrics), ("pairwise_errors.csv", details), ("beam_response.csv", beam_rows)):
        with (output / filename).open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    coords = []
    for name, (x, y) in models.items():
        for rx in range(4):
            for tx in range(4):
                coords.append({"model": name, "rx_index": rx, "tx_index": tx, "x_m": x[rx, tx], "y_m": y[rx, tx], "x_lambda": x[rx, tx] / config.wavelength_m, "y_lambda": y[rx, tx] / config.wavelength_m})
    with (output / "model_coordinates.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(coords[0])); writer.writeheader(); writer.writerows(coords)
    try:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 5), dpi=160)
        for name in models:
            subset = [r for r in beam_rows if r["model"] == name]
            ax.plot([r["azimuth_deg"] for r in subset], [r["response_db"] for r in subset], label=name)
        ax.set(xlabel="Azimuth (deg)", ylabel="Normalized response (dB)", title="Ideal vs provisional PCB endpoint array response")
        ax.set_ylim(-35, 1); ax.grid(True, alpha=0.25); ax.legend(); fig.tight_layout(); fig.savefig(output / "beam_comparison.png"); plt.close(fig)
    except Exception:
        pass
    best = {actual: min((row for row in metrics if row["actual_model"] == actual), key=lambda row: row["combined_rmse_deg"]) for actual in models}
    summary = {"status": "completed_provisional_array_comparison", "models": list(models), "truth_grid": {"azimuth_deg": truth_az.tolist(), "elevation_deg": truth_el.tolist()}, "solver_grid": {"azimuth_deg": solver_az.tolist(), "elevation_deg": solver_el.tolist()}, "best_assumed_by_actual": best, "provisional_geometry_status": "geometric_endpoint_candidate_not_phase_center", "hardware_aoa_validated": False}
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.107 理想阵列与 PCB 几何候选比较", "", "本阶段将 V0.4.106 临时 TX/RX 端点候选接入稀疏阵列导向矢量求解器。扫描输入是无噪声合成 IQ；结果用于几何敏感性分析，不是硬件 AoA 验证。", "", "| 实际模型 | 估计模型 | 方位 RMSE(°) | 俯仰 RMSE(°) | 综合 RMSE(°) | 最大绝对误差(°) |", "|---|---|---:|---:|---:|---:"]
    lines += [f"| {r['actual_model']} | {r['assumed_model']} | {r['azimuth_rmse_deg']:.3f} | {r['elevation_rmse_deg']:.3f} | {r['combined_rmse_deg']:.3f} | {r['max_abs_error_deg']:.1f} |" for r in metrics]
    lines += ["", "## 如何解释", "", "同模型实际/估计时的误差代表当前网格搜索和角度量化误差；交叉模型误差表示若阵列几何假设不一致，AoA 可能产生偏差或空间混叠。PCB 候选含多波长间距，旁瓣和角度歧义是预期现象。", "", "## 证据边界", "", "PCB 模型来自 RF 走线端点的几何候选，状态为 `geometric_endpoint_candidate_not_phase_center`。未使用实测 DCA1000 IQ、方向图、互耦或校准矩阵，不能推导真实 AWR2944P AoA 精度。"]
    (output / "output_analysis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--provisional", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(compare(args.provisional, args.output), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
