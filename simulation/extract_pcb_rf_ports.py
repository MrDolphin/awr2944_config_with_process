"""Extract RF-net pads and routed geometry from an Altium ASCII PcbDoc."""

from __future__ import annotations

import argparse
import csv
import re
from collections import Counter
from pathlib import Path


NET_NAMES = {107: "TX4", 108: "TX3", 109: "TX2", 110: "TX1",
             117: "RX4", 118: "RX3", 119: "RX2", 120: "RX1"}
MIL_TO_MM = 0.0254


def value(line: str, key: str) -> str | None:
    match = re.search(rf"(?:^|\|){re.escape(key)}=([^|]+)", line)
    return match.group(1).strip() if match else None


def extract(pcbdoc: Path, output: Path) -> dict:
    counts: Counter[str] = Counter()
    pads: list[dict[str, object]] = []
    endpoints: dict[int, list[tuple[float, float]]] = {net: [] for net in NET_NAMES}
    for line in pcbdoc.read_text(errors="ignore").splitlines():
        record = value(line, "RECORD")
        if not record:
            continue
        net_text = value(line, "NET")
        net = int(net_text) if net_text and net_text.isdigit() else None
        if net not in NET_NAMES:
            continue
        counts[record] += 1
        if record == "Pad":
            x, y = value(line, "X"), value(line, "Y")
            if x and y:
                pads.append({"antenna": NET_NAMES[net], "net_id": net,
                             "component": value(line, "COMPONENT") or "",
                             "pad_name": value(line, "NAME") or "",
                             "layer": value(line, "LAYER") or "",
                             "x_mil": float(x.replace("mil", "")),
                             "y_mil": float(y.replace("mil", "")),
                             "source_status": "rf_net_pad_not_antenna_phase_center"})
        elif record == "Track":
            x1, y1 = value(line, "X1"), value(line, "Y1")
            x2, y2 = value(line, "X2"), value(line, "Y2")
            if x1 and y1 and x2 and y2:
                endpoints[net].extend([(float(x1.replace("mil", "")), float(y1.replace("mil", ""))),
                                       (float(x2.replace("mil", "")), float(y2.replace("mil", "")))])
    output.mkdir(parents=True, exist_ok=True)
    with (output / "rf_net_pads.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = list(pads[0]) if pads else ["antenna", "net_id", "component", "pad_name", "layer", "x_mil", "y_mil", "source_status"]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(pads)
    summary_rows = []
    for net, name in NET_NAMES.items():
        points = endpoints[net]
        summary_rows.append({
            "antenna": name, "net_id": net, "pad_count": sum(row["net_id"] == net for row in pads),
            "track_endpoint_count": len(points), "track_min_x_mil": min((p[0] for p in points), default=""),
            "track_max_x_mil": max((p[0] for p in points), default=""),
            "track_min_y_mil": min((p[1] for p in points), default=""),
            "track_max_y_mil": max((p[1] for p in points), default=""),
        })
    with (output / "rf_net_geometry_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary_rows[0]))
        writer.writeheader()
        writer.writerows(summary_rows)
    (output / "output_analysis.md").write_text(
        "# V0.4.14 RF 网络馈电候选审计\n\n"
        f"ASCII PCB：`{pcbdoc.resolve()}`。RF 网络共识别 {len(NET_NAMES)} 个；Pad 对象 {len(pads)} 个。\n\n"
        "`rf_net_pads.csv` 是芯片/器件端的 RF 网络 Pad 候选，不是天线相位中心；"
        "`rf_net_geometry_summary.csv` 汇总各网络的 Track 端点范围。需要结合装配图和 RF 走线拓扑，"
        "才能判断哪一端是芯片端、哪一端是天线馈电端。\n",
        encoding="utf-8")
    return {"net_object_counts": dict(counts), "rf_pad_count": len(pads), "output": str(output.resolve())}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcbdoc", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(extract(args.pcbdoc.resolve(), args.output.resolve()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
