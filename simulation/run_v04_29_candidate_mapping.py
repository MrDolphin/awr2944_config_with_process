"""Cross-check extracted PCB RF regions against antGeometryCfg virtual channels."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def as_float(row: dict[str, str], key: str) -> float:
    return float(row[key])


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def run(mapping_path: Path, region_path: Path, output: Path) -> dict:
    mapping = read_csv(mapping_path)
    regions = {row["antenna"].upper(): row for row in read_csv(region_path)}
    rows = []
    for item in mapping:
        tx = item["tx_name"].upper()
        rx = item["rx_name"].upper()
        tx_region = regions.get(tx)
        rx_region = regions.get(rx)
        complete = tx_region is not None and rx_region is not None
        tx_x = as_float(tx_region, "center_x_mm") if tx_region else None
        tx_y = as_float(tx_region, "center_y_mm") if tx_region else None
        rx_x = as_float(rx_region, "center_x_mm") if rx_region else None
        rx_y = as_float(rx_region, "center_y_mm") if rx_region else None
        rows.append({
            "virtual_input_index": item["virtual_input_index"],
            "tx_name": tx,
            "rx_name": rx,
            "cfg_row": item["row"],
            "cfg_column": item["column"],
            "cfg_azimuth_spacing_lambda": item["azimuth_spacing_lambda"],
            "cfg_elevation_spacing_lambda": item["elevation_spacing_lambda"],
            "tx_center_x_mm": "" if tx_x is None else f"{tx_x:.6f}",
            "tx_center_y_mm": "" if tx_y is None else f"{tx_y:.6f}",
            "rx_center_x_mm": "" if rx_x is None else f"{rx_x:.6f}",
            "rx_center_y_mm": "" if rx_y is None else f"{rx_y:.6f}",
            "mapping_status": "candidate_network_and_region_geometry" if complete else "missing_region_candidate",
            "phase_center_confirmed": "false",
            "evidence": "PCB RF net name + extracted copper-region centroid + antGeometryCfg row/column; centroid is not electrical phase centre",
        })
    output.mkdir(parents=True, exist_ok=True)
    mapping_fields = list(rows[0]) if rows else ["virtual_input_index"]
    write_csv(output / "channel_mapping_candidates.csv", rows, mapping_fields)

    # Compare relative shapes after removing arbitrary PCB/CFG origins.
    cfg = np.array([[float(item["column"]) * float(item["azimuth_spacing_lambda"]), float(item["row"]) * float(item["elevation_spacing_lambda"])] for item in mapping])
    pcb = np.array([[(float(regions[item["tx_name"].upper()]["center_x_mm"]) + float(regions[item["rx_name"].upper()]["center_x_mm"])) / 2.0, (float(regions[item["tx_name"].upper()]["center_y_mm"]) + float(regions[item["rx_name"].upper()]["center_y_mm"])) / 2.0] for item in mapping])
    cfg -= cfg.mean(axis=0)
    pcb -= pcb.mean(axis=0)
    transforms = {"identity": pcb, "mirror_x": pcb * np.array([-1.0, 1.0]), "mirror_y": pcb * np.array([1.0, -1.0]), "rotate_180": -pcb}
    transform_rows = []
    for name, transformed in transforms.items():
        error = transformed - cfg
        transform_rows.append({"transform": name, "rms_error_normalized_mm": f"{float(np.sqrt(np.mean(error ** 2))):.6f}", "x_rms_normalized_mm": f"{float(np.sqrt(np.mean(error[:, 0] ** 2))):.6f}", "y_rms_normalized_mm": f"{float(np.sqrt(np.mean(error[:, 1] ** 2))):.6f}", "status": "shape_screening_only"})
    write_csv(output / "coordinate_transform_candidates.csv", transform_rows, list(transform_rows[0]))
    best = min(transform_rows, key=lambda row: float(row["rms_error_normalized_mm"])) if transform_rows else None
    summary = {"status": "completed_candidate_channel_mapping", "channel_count": len(rows), "region_count": len(regions), "all_channels_have_named_regions": all(row["mapping_status"] == "candidate_network_and_region_geometry" for row in rows), "best_shape_transform": best["transform"] if best else None, "phase_center_confirmed": False, "coordinate_status": "candidate_only"}
    (output / "mapping_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.29 候选通道映射", "", f"- 虚拟通道数：{summary['channel_count']}", f"- 已提取 RF 区域数：{summary['region_count']}", f"- 最小形状筛查误差变换：{summary['best_shape_transform'] or '-'}", "", "## 结论", "", "8 个 PCB RF 网络可以与当前 4TX×4RX `antGeometryCfg` 名称组合，形成 16 个候选虚拟通道。这里使用的是 PCB 铜区质心和 CFG 理想行列，不是天线电气相位中心。", "", "坐标变换扫描只比较去除平移后的相对形状；它不能证明阵面朝向、绝对坐标、通道极性或真实 AoA 精度。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--regions", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.mapping.resolve(), args.regions.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
