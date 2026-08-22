"""Compare ideal, copper-centroid, and RF-endpoint candidate virtual arrays."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_38_pcb_array_comparison import load_cfg, load_pcb_virtual, scan_pair
from simulation.v03 import FmcwConfig


def load_endpoint_virtual(path: Path, config: FmcwConfig) -> tuple[np.ndarray, np.ndarray]:
    with path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    points = {row["net"]: (float(row["x_mil"]) * 0.0254, float(row["y_mil"]) * 0.0254) for row in rows}
    tx0, rx0 = points["TX1"], points["RX1"]
    x = np.zeros((4, 4)); y = np.zeros((4, 4))
    for tx in range(4):
        for rx in range(4):
            txp, rxp = points[f"TX{tx + 1}"], points[f"RX{rx + 1}"]
            x[rx, tx] = txp[0] - tx0[0] + rxp[0] - rx0[0]
            y[rx, tx] = txp[1] - tx0[1] + rxp[1] - rx0[1]
    return x, y


def models(mapping: Path, copper: Path, endpoint: Path, config: FmcwConfig) -> dict:
    d = config.wavelength_m / 2.0
    ideal = (np.repeat((np.arange(4) * d)[None, :], 4, axis=0), np.repeat((np.arange(4) * d)[:, None], 4, axis=1))
    return {"ideal_half_lambda": ideal, "pcb_copper_centroid": load_pcb_virtual(copper, config), "rf_endpoint_candidate": load_endpoint_virtual(endpoint, config), "cfg_candidate": load_cfg(mapping, config)}


def run(mapping: Path, copper: Path, endpoint: Path, output: Path) -> dict:
    config = FmcwConfig(); model_map = models(mapping, copper, endpoint, config)
    azimuths = np.arange(-60.0, 60.1, 20.0); elevations = np.arange(-20.0, 20.1, 10.0)
    metrics = []; details = []
    for actual_name, actual in model_map.items():
        for assumed_name, assumed in model_map.items():
            summary, errors = scan_pair(config, actual, assumed, azimuths, elevations)
            metrics.append({"actual_model": actual_name, "assumed_model": assumed_name, **summary})
            details.extend({"actual_model": actual_name, "assumed_model": assumed_name, **row} for row in errors)
    output.mkdir(parents=True, exist_ok=True)
    for filename, rows in (("pairwise_metrics.csv", metrics), ("pairwise_errors.csv", details)):
        with (output / filename).open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    coordinates = []
    for name, (x, y) in model_map.items():
        for rx in range(4):
            for tx in range(4):
                coordinates.append({"model": name, "rx_index": rx, "tx_index": tx, "x_m": x[rx, tx], "y_m": y[rx, tx], "x_lambda": x[rx, tx] / config.wavelength_m, "y_lambda": y[rx, tx] / config.wavelength_m})
    with (output / "model_coordinates.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(coordinates[0])); writer.writeheader(); writer.writerows(coordinates)
    result = {"status": "completed_four_array_model_comparison", "models": list(model_map), "azimuth_grid_deg": azimuths.tolist(), "elevation_grid_deg": elevations.tolist(), "hardware_aoa_validated": False, "phase_center_ready": False, "metrics_rows": len(metrics)}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.128 三种阵列几何模型比较", "", "本阶段比较理想半波长阵列、PCB 铜区几何中心阵列、RF 走线终点候选阵列和 CFG 候选阵列。扫描方位 -60°～60°、俯仰 -20°～20°。", "", "## 如何读表", "", "`pairwise_metrics.csv` 的 actual_model 表示产生 IQ 的真实几何，assumed_model 表示 AoA 解算器采用的几何。RMSE 越小，表示两套几何在当前无噪声仿真下越一致；大误差反映几何不一致、稀疏阵列相位折叠或当前简化解算器的局限。", "", "## 当前判断", "", "RF 终点候选与铜区中心不是同一个几何定义，不能直接互换。若在同一模型自匹配时仍出现较大误差，优先说明当前 phase-plane unwrap 解算器不适合这种非规则/大间距候选阵列，而不是证明 PCB 或终点提取错误。", "", "## 证据边界", "", "三种 PCB 相关模型均是几何候选，不是实测相位中心；没有加入天线方向图、互耦、极化、校准矩阵、噪声和船体结构。结果只用于阵列几何敏感性分析，不能作为真实 AoA 精度或探测距离结论。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--mapping", type=Path, required=True); parser.add_argument("--copper", type=Path, required=True); parser.add_argument("--endpoint", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.mapping.resolve(), args.copper.resolve(), args.endpoint.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
