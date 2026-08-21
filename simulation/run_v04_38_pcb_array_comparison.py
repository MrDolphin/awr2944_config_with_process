"""Compare ideal, CFG and PCB-copper-centroid virtual-array candidates.

The PCB model is explicitly a geometry candidate.  Copper centroids are not
electrical phase centres and this script must not be used as measured hardware
AoA validation.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_positions, generate_aoa_iq_with_positions


def _empty_models() -> dict[str, tuple[np.ndarray, np.ndarray]]:
    return {name: (np.zeros((4, 4), dtype=float), np.zeros((4, 4), dtype=float)) for name in ("ideal_half_lambda", "cfg_candidate", "pcb_centroid_candidate")}


def load_cfg(mapping_path: Path, config: FmcwConfig) -> tuple[np.ndarray, np.ndarray]:
    x = np.zeros((4, 4), dtype=float)
    y = np.zeros((4, 4), dtype=float)
    with mapping_path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            rx, tx = int(row["rx_index"]), int(row["tx_index"])
            x[rx, tx] = float(row["column"]) * float(row["azimuth_spacing_lambda"]) * config.wavelength_m
            y[rx, tx] = float(row["row"]) * float(row["elevation_spacing_lambda"]) * config.wavelength_m
    return x, y


def load_pcb_virtual(pcb_path: Path, config: FmcwConfig) -> tuple[np.ndarray, np.ndarray]:
    """Load TX/RX copper-centroid sum coordinates relative to TX1/RX1."""
    with pcb_path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    tx: dict[str, tuple[float, float]] = {}
    rx: dict[str, tuple[float, float]] = {}
    for row in rows:
        name = row.get("antenna", "").upper()
        point = (float(row["center_x_mil"]) * 0.0254, float(row["center_y_mil"]) * 0.0254)
        if name.startswith("TX"):
            tx[name] = point
        elif name.startswith("RX"):
            rx[name] = point
    expected = {f"TX{i}" for i in range(1, 5)} | {f"RX{i}" for i in range(1, 5)}
    if set(tx) | set(rx) != expected:
        raise ValueError(f"PCB candidate must contain TX1..TX4 and RX1..RX4, got {sorted(set(tx)|set(rx))}")
    tx0, rx0 = tx["TX1"], rx["RX1"]
    x = np.zeros((4, 4), dtype=float)
    y = np.zeros((4, 4), dtype=float)
    # Preserve the existing board-axis convention: x drives azimuth and y elevation.
    for tx_index in range(4):
        for rx_index in range(4):
            tx_point = tx[f"TX{tx_index + 1}"]
            rx_point = rx[f"RX{rx_index + 1}"]
            x[rx_index, tx_index] = (tx_point[0] - tx0[0]) + (rx_point[0] - rx0[0])
            y[rx_index, tx_index] = (tx_point[1] - tx0[1]) + (rx_point[1] - rx0[1])
    return x, y


def model_positions(mapping_path: Path, pcb_path: Path, config: FmcwConfig) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    models = _empty_models()
    d = config.wavelength_m / 2.0
    models["ideal_half_lambda"] = (
        np.repeat((np.arange(4) * d)[None, :], 4, axis=0),
        np.repeat((np.arange(4) * d)[:, None], 4, axis=1),
    )
    models["cfg_candidate"] = load_cfg(mapping_path, config)
    models["pcb_centroid_candidate"] = load_pcb_virtual(pcb_path, config)
    return models


def scan_pair(config: FmcwConfig, actual: tuple[np.ndarray, np.ndarray], assumed: tuple[np.ndarray, np.ndarray], azimuths: np.ndarray, elevations: np.ndarray) -> tuple[dict, list[dict]]:
    errors: list[dict] = []
    for elevation in elevations:
        for azimuth in azimuths:
            iq = generate_aoa_iq_with_positions(config, slant_range_m=30.0, radial_velocity_mps=0.0, azimuth_deg=float(azimuth), elevation_deg=float(elevation), x_positions_m=actual[0], y_positions_m=actual[1])
            estimated = estimate_aoa_from_positions(np.mean(iq, axis=(0, 1)), config, assumed[0], assumed[1])
            errors.append({"truth_azimuth_deg": float(azimuth), "truth_elevation_deg": float(elevation), "estimated_azimuth_deg": estimated[0], "estimated_elevation_deg": estimated[1], "azimuth_error_deg": estimated[0] - azimuth, "elevation_error_deg": estimated[1] - elevation})
    values = np.asarray([[row["azimuth_error_deg"], row["elevation_error_deg"]] for row in errors])
    summary = {"azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))), "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))), "azimuth_max_abs_error_deg": float(np.max(np.abs(values[:, 0]))), "elevation_max_abs_error_deg": float(np.max(np.abs(values[:, 1]))), "combined_rmse_deg": float(np.sqrt(np.mean(values ** 2)))}
    return summary, errors


def run(mapping_path: Path, pcb_path: Path, output: Path) -> dict:
    config = FmcwConfig()
    models = model_positions(mapping_path, pcb_path, config)
    azimuths = np.arange(-60.0, 60.1, 20.0)
    elevations = np.arange(-20.0, 20.1, 10.0)
    rows: list[dict] = []
    detail_rows: list[dict] = []
    for actual_name, actual in models.items():
        for assumed_name, assumed in models.items():
            metrics, details = scan_pair(config, actual, assumed, azimuths, elevations)
            rows.append({"actual_model": actual_name, "assumed_model": assumed_name, **metrics})
            detail_rows.extend({"actual_model": actual_name, "assumed_model": assumed_name, **detail} for detail in details)
    output.mkdir(parents=True, exist_ok=True)
    for name, data in (("pairwise_metrics.csv", rows), ("pairwise_errors.csv", detail_rows)):
        with (output / name).open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(data[0])); writer.writeheader(); writer.writerows(data)
    geometry_rows = []
    for model_name, (x, y) in models.items():
        for rx in range(4):
            for tx in range(4):
                geometry_rows.append({"model": model_name, "rx_index": rx, "tx_index": tx, "x_m": x[rx, tx], "y_m": y[rx, tx], "x_lambda": x[rx, tx] / config.wavelength_m, "y_lambda": y[rx, tx] / config.wavelength_m})
    with (output / "model_coordinates.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(geometry_rows[0])); writer.writeheader(); writer.writerows(geometry_rows)
    best_by_actual = {actual: min((row for row in rows if row["actual_model"] == actual), key=lambda row: row["combined_rmse_deg"]) for actual in models}
    summary = {"status": "completed_pcb_candidate_array_comparison", "models": list(models), "azimuth_grid_deg": azimuths.tolist(), "elevation_grid_deg": elevations.tolist(), "best_assumed_by_actual": best_by_actual, "pcb_model_status": "cad_copper_centroid_candidate_not_phase_center", "estimator_status": "phase_plane_unwrap_contract_not_sparse_array_solver", "hardware_aoa_validated": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.38 PCB 候选阵列 AoA 对比", "", "本阶段把 PCB TX/RX 铜区几何质心求和模型接入 AoA 敏感性扫描，并与理想半波长阵列和 CFG 几何候选比较。", "", f"扫描方位：{azimuths.min():.0f}°～{azimuths.max():.0f}°；俯仰：{elevations.min():.0f}°～{elevations.max():.0f}°。", "", "| 实际几何 | 估计几何 | 方位 RMSE(°) | 俯仰 RMSE(°) | 综合 RMSE(°) |", "|---|---|---:|---:|---:"]
    lines.extend(f"| {row['actual_model']} | {row['assumed_model']} | {row['azimuth_rmse_deg']:.4f} | {row['elevation_rmse_deg']:.4f} | {row['combined_rmse_deg']:.4f} |" for row in rows)
    lines += ["", "## 如何读结果", "", "对角度网格逐点比较真值和估计值；RMSE 越小表示在当前无噪声、无校准误差的几何模型下越一致。大误差只说明两套坐标定义不一致或出现空间混叠，不能单独证明 PCB 候选就是真实阵列。", "", "## 结果解释和限制", "", "PCB 候选的虚拟间距明显大于半波长，广角扫描会出现相位折叠/空间混叠。当前 V0.4 phase-plane estimator 依赖固定轴向 unwrap，因此即使 actual/assumed 使用同一 PCB 候选，广角自匹配也可能出现非零 RMSE；这说明需要下一阶段的稀疏阵列/整数相位歧义求解器，而不是说明 PCB 文件本身错误。", "", "## 证据边界", "", "PCB 模型来自 RF 铜区几何质心求和，状态为 `cad_copper_centroid_candidate_not_phase_center`。它用于布局敏感性分析，不是 AWR2944P 的实测相位中心、方向图或 TI SDK AoA 验证。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--pcb", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.mapping.resolve(), args.pcb.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
