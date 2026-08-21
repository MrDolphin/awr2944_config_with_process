"""Extract TX/RX antenna copper-region geometry from an Altium ASCII PcbDoc."""

from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path


NET_NAMES = {107: "TX4", 108: "TX3", 109: "TX2", 110: "TX1",
             117: "RX4", 118: "RX3", 119: "RX2", 120: "RX1"}
MIL_TO_MM = 0.0254


def extract(path: Path, output: Path) -> None:
    rows = []
    for line in path.read_text(errors="ignore").splitlines():
        match = re.search(r"RECORD=Region\|NET=(\d+)", line)
        if not match or int(match.group(1)) not in NET_NAMES:
            continue
        points = [(float(x.replace("mil", "")), float(y.replace("mil", "")))
                  for _, x, y in re.findall(r"VX(\d+)=([^|]+)\|VY\1=([^|]+)", line)]
        if not points:
            continue
        xs = [x for x, _ in points]
        ys = [y for _, y in points]
        name = NET_NAMES[int(match.group(1))]
        rows.append({
            "antenna": name,
            "net_id": int(match.group(1)),
            "region_vertex_count": len(points),
            "center_x_mil": sum(xs) / len(xs),
            "center_y_mil": sum(ys) / len(ys),
            "min_x_mil": min(xs), "max_x_mil": max(xs),
            "min_y_mil": min(ys), "max_y_mil": max(ys),
            "center_x_mm": sum(xs) / len(xs) * MIL_TO_MM,
            "center_y_mm": sum(ys) / len(ys) * MIL_TO_MM,
            "coordinate_status": "cad_copper_region_centroid_not_electrical_phase_center",
            "source": str(path.resolve()),
        })
    if len(rows) != 8:
        raise ValueError(f"expected 8 TX/RX antenna regions, found {len(rows)}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(sorted(rows, key=lambda row: row["antenna"]))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcbdoc", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    extract(args.pcbdoc.resolve(), args.output.resolve())
    print(f"Extracted 8 AWR2944P antenna regions into {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
