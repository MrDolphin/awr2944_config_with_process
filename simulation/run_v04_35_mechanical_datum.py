"""Extract a mechanical PCB datum from an Altium ASCII PcbDoc."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path


MIL_TO_MM = 0.0254
FIELD_RE = re.compile(r"\|([^=|]+)=([^|]*)")


def parse(line: str) -> tuple[str, dict[str, str]] | None:
    match = re.search(r"\|RECORD=([^|]+)", line)
    if not match:
        return None
    return match.group(1), {key: value for key, value in FIELD_RE.findall(line)}


def as_mil(value: str | None) -> float | None:
    if not value:
        return None
    match = re.match(r"\s*([-+]?\d+(?:\.\d+)?)mil", value)
    return float(match.group(1)) if match else None


def run(pcbdoc: Path, regions_csv: Path, output: Path) -> dict:
    board_records = []
    components = []
    with pcbdoc.open(encoding="utf-8", errors="replace") as handle:
        for line in handle:
            item = parse(line)
            if not item:
                continue
            record, values = item
            if record == "Board":
                board_records.append(values)
            elif record == "Component":
                components.append(values)
    header = board_records[0] if board_records else {}
    origin_x = as_mil(header.get("ORIGINX")); origin_y = as_mil(header.get("ORIGINY"))
    vertices = []
    for values in board_records:
        for key, value in values.items():
            match = re.fullmatch(r"VX(\d+)", key)
            if match and f"VY{match.group(1)}" in values:
                x, y = as_mil(value), as_mil(values[f"VY{match.group(1)}"])
                if x is not None and y is not None:
                    vertices.append((x, y))
    unique_vertices = sorted(set(vertices))
    min_x = min((point[0] for point in unique_vertices), default=None); max_x = max((point[0] for point in unique_vertices), default=None)
    min_y = min((point[1] for point in unique_vertices), default=None); max_y = max((point[1] for point in unique_vertices), default=None)
    rows = []
    for index, (x, y) in enumerate(unique_vertices):
        rows.append({"vertex_index": index, "x_mil": x, "y_mil": y, "x_mm": x * MIL_TO_MM, "y_mm": y * MIL_TO_MM, "x_relative_to_origin_mm": (x - origin_x) * MIL_TO_MM if origin_x is not None else "", "y_relative_to_origin_mm": (y - origin_y) * MIL_TO_MM if origin_y is not None else ""})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "board_outline.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = list(rows[0]) if rows else ["vertex_index"]
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    rf_rows = []
    with regions_csv.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            x, y = float(row["center_x_mil"]), float(row["center_y_mil"])
            rf_rows.append({"antenna": row["antenna"], "center_x_mil": x, "center_y_mil": y, "center_x_relative_to_origin_mm": (x - origin_x) * MIL_TO_MM if origin_x is not None else "", "center_y_relative_to_origin_mm": (y - origin_y) * MIL_TO_MM if origin_y is not None else "", "status": "mechanical_board_relative_not_phase_center"})
    with (output / "rf_regions_board_relative.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = list(rf_rows[0]) if rf_rows else ["antenna"]
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(rf_rows)
    mechanical_candidates = []
    for values in components:
        designator = values.get("SOURCEDESIGNATOR", "")
        description = values.get("SOURCEDESCRIPTION", "")
        if designator.upper().startswith(("FID", "H", "MH")) or re.search(r"mount|hole|fiducial", description, re.IGNORECASE):
            mechanical_candidates.append({"designator": designator, "pattern": values.get("PATTERN", ""), "x": values.get("X", ""), "y": values.get("Y", ""), "description": description, "status": "mechanical_reference_candidate"})
    with (output / "mechanical_reference_candidates.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = list(mechanical_candidates[0]) if mechanical_candidates else ["designator", "status"]
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(mechanical_candidates)
    summary = {"status": "completed_mechanical_datum_extraction", "pcbdoc": str(pcbdoc.resolve()), "origin_x_mil": origin_x, "origin_y_mil": origin_y, "board_outline_vertex_count": len(unique_vertices), "board_width_mm": (max_x - min_x) * MIL_TO_MM if min_x is not None else None, "board_height_mm": (max_y - min_y) * MIL_TO_MM if min_y is not None else None, "rf_region_count": len(rf_rows), "mechanical_reference_candidate_count": len(mechanical_candidates), "phase_center_confirmed": False, "installation_pose_confirmed": False}
    (output / "mechanical_datum.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.35 PCB 机械坐标基准", "", f"- PCB 原点：({origin_x} mil, {origin_y} mil)", f"- 板框顶点数：{len(unique_vertices)}", f"- 板框尺寸：{summary['board_width_mm']:.3f} mm × {summary['board_height_mm']:.3f} mm" if summary["board_width_mm"] is not None else "- 板框尺寸：未知", f"- RF 区域：{len(rf_rows)}", f"- 机械参考候选：{len(mechanical_candidates)}", "", "## 边界", "", "板框原点和 RF 区域的相对坐标可以用于机械配准和坐标变换筛查，但不等于天线电气相位中心，也不能确认船体安装姿态、阵面法向或垂直地面角度。装配图当前未提供可直接提取的相位中心尺寸。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcbdoc", type=Path, required=True)
    parser.add_argument("--regions", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.pcbdoc.resolve(), args.regions.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
