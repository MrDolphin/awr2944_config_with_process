"""Trace AWR2944 TX/RX PCB network primitives from an Altium ASCII PcbDoc."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path


FIELD_RE = re.compile(r"([^|=]+)=([^|]*)")
TARGETS = {"TX1", "TX2", "TX3", "TX4", "RX1", "RX2", "RX3", "RX4"}


def _records(text: str, kind: str):
    for chunk in text.split(f"|RECORD={kind}")[1:]:
        chunk = chunk.split("|RECORD=", 1)[0]
        yield dict(FIELD_RE.findall(chunk[:6000]))


def trace(pcbdoc: Path, output: Path) -> dict:
    text = pcbdoc.read_text(errors="ignore")
    net_names = {}
    for row in _records(text, "Net"):
        if row.get("NAME") in TARGETS:
            net_names[row.get("ID", "")] = row["NAME"]

    primitives = []
    for kind in ("Pad", "Track", "Arc"):
        for row in _records(text, kind):
            name = net_names.get(row.get("NET", ""))
            if not name:
                continue
            item = {"kind": kind, "net": name, "net_id": row.get("NET", ""), "layer": row.get("LAYER", "")}
            if kind == "Pad":
                item.update({"component": row.get("COMPONENT", ""), "pad": row.get("NAME", ""), "x1_mil": float(row["X"].removesuffix("mil")), "y1_mil": float(row["Y"].removesuffix("mil"))})
            elif kind == "Track":
                item.update({"x1_mil": float(row["X1"].removesuffix("mil")), "y1_mil": float(row["Y1"].removesuffix("mil")), "x2_mil": float(row["X2"].removesuffix("mil")), "y2_mil": float(row["Y2"].removesuffix("mil")), "width_mil": float(row["WIDTH"].removesuffix("mil"))})
            else:
                item.update({"cx_mil": float(row["LOCATION.X"].removesuffix("mil")), "cy_mil": float(row["LOCATION.Y"].removesuffix("mil")), "radius_mil": float(row["RADIUS"].removesuffix("mil")), "start_deg": float(row["STARTANGLE"]), "end_deg": float(row["ENDANGLE"]), "width_mil": float(row["WIDTH"].removesuffix("mil"))})
            primitives.append(item)

    output.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for item in primitives for key in item})
    with (output / "rf_network_primitives.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(primitives)

    counts = {name: {kind: sum(1 for item in primitives if item["net"] == name and item["kind"] == kind) for kind in ("Pad", "Track", "Arc", "Region", "Polygon")} for name in sorted(TARGETS)}
    result = {
        "status": "completed_rf_network_trace",
        "source": str(pcbdoc),
        "target_networks": sorted(TARGETS),
        "primitive_count": len(primitives),
        "primitive_counts_by_net": counts,
        "primitive_types_found": sorted({item["kind"] for item in primitives}),
        "phase_center_ready": False,
        "interpretation": "TX/RX package pads and Top-layer RF track/arc geometry were found; no target-net Region/Polygon/independent antenna pad was present in this ASCII export.",
        "remaining_evidence": [
            "identify RF trace endpoints against the antenna drawing or EM model",
            "confirm whether the trace endpoint is a feed point or a radiating element center",
            "obtain measured/EM phase centers and radiation pattern",
        ],
    }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    report = [
        "# V0.4.103 TX/RX PCB 网络追踪",
        "",
        f"输入：`{pcbdoc}`",
        "",
        f"共找到 {len(primitives)} 个目标网络图元：" + ", ".join(f"{k}={v}" for k, v in sorted({kind: sum(1 for x in primitives if x['kind'] == kind) for kind in ('Pad', 'Track', 'Arc')}.items())) + "。",
        "",
        "## 结论",
        "",
        "TX/RX 网络已经可以从 AWR2944 芯片封装焊盘追踪到 Top 层 RF 走线和圆弧。当前导出中没有出现带目标网络名的 Region、Polygon 或独立天线 Pad，因此不能把走线终点直接当作天线相位中心。",
        "",
        "本结果适合作为阵面建模的几何线索和资料审计证据，不足以单独生成真实方向图或校准 AoA。",
        "",
        "完整图元表见 `rf_network_primitives.csv`；机器可读结论见 `summary.json`。",
    ]
    (output / "output_analysis.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcbdoc", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = trace(args.pcbdoc, args.output)
    print(json.dumps({k: result[k] for k in ("status", "primitive_count", "primitive_types_found", "phase_center_ready")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
