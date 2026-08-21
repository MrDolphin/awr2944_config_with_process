"""Compare PCB-derived virtual coordinates with CFG antGeometryCfg coordinates."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def run(cad_path: Path, geometry_path: Path, output: Path, frequency_ghz: float = 77.0) -> dict:
    cad = read_csv(cad_path)
    geometry = read_csv(geometry_path)
    by_key = {(row["tx"], row["rx"]): row for row in cad}
    wavelength_mm = 299792458.0 / (frequency_ghz * 1e9) * 1000.0
    rows = []
    for item in geometry:
        key = (item["tx_name"], item["rx_name"])
        actual = by_key[key]
        expected_x = float(item["column"]) * float(item["azimuth_spacing_lambda"]) * wavelength_mm
        expected_y = float(item["row"]) * float(item["elevation_spacing_lambda"]) * wavelength_mm
        actual_x = float(actual["x_mm"])
        actual_y = float(actual["y_mm"])
        rows.append({
            "virtual_input_index": item["virtual_input_index"],
            "tx": key[0], "rx": key[1], "row": item["row"], "column": item["column"],
            "pcb_x_mm": actual_x, "pcb_y_mm": actual_y,
            "cfg_x_mm": expected_x, "cfg_y_mm": expected_y,
            "delta_x_mm": actual_x - expected_x, "delta_y_mm": actual_y - expected_y,
            "coordinate_status": "pcb_centroid_vs_cfg_ideal_raw_comparison",
        })
    rms_x = math.sqrt(sum(row["delta_x_mm"] ** 2 for row in rows) / len(rows))
    rms_y = math.sqrt(sum(row["delta_y_mm"] ** 2 for row in rows) / len(rows))
    output.mkdir(parents=True, exist_ok=True)
    with (output / "coordinate_mapping.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "frequency_ghz": frequency_ghz, "wavelength_mm": wavelength_mm,
        "count": len(rows), "raw_rms_delta_x_mm": rms_x, "raw_rms_delta_y_mm": rms_y,
        "interpretation_status": "pcb_centroid_is_not_electrical_phase_center",
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    lines = [
        "# V0.4.12 PCB 坐标与 CFG 阵列映射分析", "",
        f"频率：{frequency_ghz:g} GHz；波长：{wavelength_mm:.6f} mm；比较通道数：{len(rows)}。", "",
        f"当前 PCB 原点平移后的几何中心与 CFG 理想网格的 X 方向 RMS 差：{rms_x:.6f} mm。",
        f"当前 PCB 原点平移后的几何中心与 CFG 理想网格的 Y 方向 RMS 差：{rms_y:.6f} mm。", "",
        "这里没有做旋转、镜像、缩放或最佳刚体配准；数值仅用于发现坐标系不一致，不能作为 AoA 误差。",
        "如果要把 PCB 坐标用于 AoA，必须先确认板面法向、X/Y 正方向、坐标原点和电气相位中心。",
        "详细逐通道数据见 `coordinate_mapping.csv`。",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cad", type=Path, required=True)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.cad.resolve(), args.geometry.resolve(), args.output.resolve()), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
