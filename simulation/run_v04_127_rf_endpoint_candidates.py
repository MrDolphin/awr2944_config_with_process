"""Extract heuristic RF trace endpoint candidates from the audited PCB graph."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

TARGETS = {"TX1", "TX2", "TX3", "TX4", "RX1", "RX2", "RX3", "RX4"}


def _point(x: str, y: str) -> tuple[float, float]:
    return float(x), float(y)


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def extract(primitives_csv: Path, pads_csv: Path, regions_csv: Path, output: Path) -> dict:
    primitives = _rows(primitives_csv)
    pads = {row["net_name"]: _point(row["x_mil"], row["y_mil"]) for row in _rows(pads_csv)}
    regions = {row["antenna"]: row for row in _rows(regions_csv)}
    candidates = []
    for net in sorted(TARGETS):
        points: list[tuple[float, float, str]] = []
        for row in primitives:
            if row["net"] != net:
                continue
            if row["kind"] == "Track":
                points.extend([(_point(row["x1_mil"], row["y1_mil"])[0], _point(row["x1_mil"], row["y1_mil"])[1], "track_start"), (_point(row["x2_mil"], row["y2_mil"])[0], _point(row["x2_mil"], row["y2_mil"])[1], "track_end")])
            elif row["kind"] == "Arc":
                cx, cy = _point(row["cx_mil"], row["cy_mil"]); radius = float(row["radius_mil"])
                for angle, label in ((float(row["start_deg"]), "arc_start"), (float(row["end_deg"]), "arc_end")):
                    radians = math.radians(angle); points.append((cx + radius * math.cos(radians), cy + radius * math.sin(radians), label))
        # The endpoint farthest from the package pad is a reproducible feed/antenna
        # candidate heuristic, not a proof of the radiating phase center.
        pad = pads[net]
        farthest = max(points, key=lambda item: _dist((item[0], item[1]), pad))
        region = regions.get(net, {})
        region_center = _point(region.get("center_x_mil", "0"), region.get("center_y_mil", "0")) if region else (float("nan"), float("nan"))
        candidates.append({"net": net, "candidate_kind": "farthest_rf_graph_endpoint", "x_mil": farthest[0], "y_mil": farthest[1], "x_mm": farthest[0] * 0.0254, "y_mm": farthest[1] * 0.0254, "package_pad_x_mil": pad[0], "package_pad_y_mil": pad[1], "distance_from_package_pad_mil": _dist((farthest[0], farthest[1]), pad), "distance_to_copper_centroid_mil": _dist((farthest[0], farthest[1]), region_center), "coordinate_status": "rf_graph_endpoint_candidate_not_phase_center"})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "rf_endpoint_candidates.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(candidates[0])); writer.writeheader(); writer.writerows(candidates)
    result = {"status": "completed_rf_endpoint_candidate_extraction", "row_count": len(candidates), "candidate_method": "farthest endpoint in each TX/RX graph from package pad", "phase_center_ready": False, "candidates": candidates}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.127 RF 走线终点候选", "", "本阶段从 V0.4.103 的 TX/RX Track/Arc 图元中提取每个网络距离 AWR2944 封装焊盘最远的图端点，作为后续馈电点/辐射区域追踪的候选。", "", "## 结果解释", "", "该启发式只利用 PCB 图形拓扑：同一 RF 网络中，距离芯片封装焊盘最远的端点通常更接近板级天线或连接终端。它不是天线相位中心证明，也没有处理过孔、电磁边界、端点渐变和封装内部走线。", "", "## 使用方式", "", "先将候选点与 `pcb_antenna_regions.csv` 的铜区几何中心比较，再建立理想阵列、铜区中心阵列和 RF 端点候选阵列的 AoA/波束差异。若候选点靠近铜区边缘或跨层，必须回到 PCB/STEP/EM 资料人工确认。", "", "## 结论", "", "当前 8 个 TX/RX 网络均得到一个可复现候选点，但 `phase_center_ready=false`。在接入真实 DCA1000 IQ、TI 校准和已知角度目标前，不得把这些候选点用于宣称真实 AoA 精度。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--primitives", type=Path, required=True); parser.add_argument("--pads", type=Path, required=True); parser.add_argument("--regions", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(extract(args.primitives.resolve(), args.pads.resolve(), args.regions.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
