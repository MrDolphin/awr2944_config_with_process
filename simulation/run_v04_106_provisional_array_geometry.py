"""Build a provisional TX/RX geometry package from V0.4.105 endpoint clues."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


EXPECTED = ["TX1", "TX2", "TX3", "TX4", "RX1", "RX2", "RX3", "RX4"]


def build(endpoint_summary: Path, output: Path, chip_x_mil: float, chip_y_mil: float) -> dict:
    source = json.loads(endpoint_summary.read_text(encoding="utf-8"))
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for channel in EXPECTED:
        candidates = source.get("candidates_by_net", {}).get(channel, [])
        if not candidates:
            raise ValueError(f"missing endpoint candidate for {channel}")
        p = candidates[0]
        rows.append({
            "channel": channel,
            "kind": "TX" if channel.startswith("TX") else "RX",
            "x_board_mil": p["x_mil"],
            "y_board_mil": p["y_mil"],
            "x_relative_mm": (p["x_mil"] - chip_x_mil) * 0.0254,
            "y_relative_mm": (p["y_mil"] - chip_y_mil) * 0.0254,
            "source_rank": 1,
            "confidence": "provisional_geometric_candidate",
            "phase_center_validated": False,
        })
    with (output / "virtual_array_coordinates.csv").open("w", newline="", encoding="utf-8") as handle:
        fields = list(rows[0].keys())
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    yaml_lines = [
        "schema: awr2944p_provisional_array_geometry_v0",
        "status: provisional",
        "confidence: geometric_candidate_not_phase_center",
        "source: v04_105_rf_endpoint_candidates",
        f"reference_chip_center_board_mil: [{chip_x_mil}, {chip_y_mil}]",
        "coordinate_convention: board_x_right_board_y_up; relative coordinates in mm",
        "elements:",
    ]
    for row in rows:
        yaml_lines.extend([
            f"  - channel: {row['channel']}",
            f"    kind: {row['kind']}",
            f"    x_relative_mm: {row['x_relative_mm']:.6f}",
            f"    y_relative_mm: {row['y_relative_mm']:.6f}",
            "    phase_center_validated: false",
        ])
    (output / "provisional_array_geometry.yaml").write_text("\n".join(yaml_lines) + "\n", encoding="utf-8")
    result = {
        "status": "completed_provisional_array_geometry",
        "source_endpoint_summary": str(endpoint_summary),
        "element_count": len(rows),
        "channels": EXPECTED,
        "phase_center_validated": False,
        "usable_for": ["candidate-vs-ideal geometry comparison", "plotting and experiment bookkeeping"],
        "not_usable_for": ["claiming TI antenna phase centers", "final AoA accuracy", "calibration replacement"],
        "reference_chip_center_board_mil": [chip_x_mil, chip_y_mil],
    }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.106 临时阵列几何候选\n\n"
        "本目录将 V0.4.105 每个网络最远几何端点转换为相对 U29 芯片中心的毫米坐标。\n\n"
        "这些坐标仅用于理想阵列与 PCB 几何候选的对比、绘图和实验记录；它们尚未经过天线 EM 仿真、暗室方向图或已知角度实测校准，不能称为 TI 官方相位中心。\n",
        encoding="utf-8",
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint-summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--chip-x-mil", type=float, default=5586.5365)
    parser.add_argument("--chip-y-mil", type=float, default=4399.3306)
    args = parser.parse_args()
    result = build(args.endpoint_summary, args.output, args.chip_x_mil, args.chip_y_mil)
    print(json.dumps({k: result[k] for k in ("status", "element_count", "phase_center_validated")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
