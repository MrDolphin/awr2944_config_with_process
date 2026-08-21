"""Audit the supplied AWR2944 EVM Altium ASCII PCB asset.

This extracts board metadata, the XA2944BGALT component, TX/RX package pads,
and the corresponding net names.  Package-pad coordinates are not antenna
phase-center coordinates; the report explicitly keeps that boundary.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path


FIELD_RE = re.compile(r"([^|=]+)=([^|]*)")


def records(text: str, kind: str):
    for chunk in text.split(f"|RECORD={kind}")[1:]:
        # A PcbDoc can contain nested ParamItem records; stop at the next
        # record marker so duplicate keys from a later record do not overwrite
        # the owning record's fields.
        chunk = chunk.split("|RECORD=", 1)[0]
        yield dict(FIELD_RE.findall(chunk[:6000]))


def audit(pcbdoc: Path, output: Path) -> dict:
    text = pcbdoc.read_text(errors="ignore")
    boards = list(records(text, "Board"))
    components = list(records(text, "Component"))
    pads = list(records(text, "Pad"))
    nets = {row.get("ID"): row.get("NAME", "") for row in records(text, "Net")}

    board = boards[0] if boards else {}
    chip = next((c for c in components if c.get("SOURCEDESCRIPTION") == "XA2944BGALT"), None)
    if chip is None:
        raise ValueError("XA2944BGALT component not found")

    txrx_names = {"TX1", "TX2", "TX3", "TX4", "RX1", "RX2", "RX3", "RX4"}
    package_pads = []
    for pad in pads:
        if pad.get("COMPONENT") != chip.get("ID"):
            continue
        net_name = nets.get(pad.get("NET"), "")
        if net_name in txrx_names:
            package_pads.append({
                "component": chip.get("SOURCEDESIGNATOR"),
                "pad": pad.get("NAME", ""),
                "net_id": pad.get("NET", ""),
                "net_name": net_name,
                "x_mil": float(pad["X"].removesuffix("mil")),
                "y_mil": float(pad["Y"].removesuffix("mil")),
                "x_mm": float(pad["X"].removesuffix("mil")) * 0.0254,
                "y_mm": float(pad["Y"].removesuffix("mil")) * 0.0254,
                "layer": pad.get("LAYER", ""),
            })

    output.mkdir(parents=True, exist_ok=True)
    with (output / "tx_rx_package_pads.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=package_pads[0].keys() if package_pads else ["net_name"])
        writer.writeheader()
        writer.writerows(package_pads)

    result = {
        "status": "completed_pcb_asset_audit",
        "source": str(pcbdoc),
        "source_format": "Altium ASCII PcbDoc",
        "board": {k: board.get(k, "") for k in ("ORIGINX", "ORIGINY", "VX0", "VY0", "VX1", "VY1", "VX2", "VY2", "VX3", "VY3")},
        "chip": {k: chip.get(k, "") for k in ("SOURCEDESIGNATOR", "SOURCEDESCRIPTION", "X", "Y", "ROTATION", "PATTERN")},
        "tx_rx_package_pad_count": len(package_pads),
        "tx_rx_package_pads": package_pads,
        "extractable_for_array_model": [
            "board coordinate origin and outline",
            "XA2944BGALT package center and rotation",
            "chip TX/RX package pad coordinates and net names",
            "PCB stackup dielectric constants and heights from V9_STACK_LAYER records",
        ],
        "not_yet_proven": [
            "TX/RX antenna phase-center coordinates",
            "element-to-element coupling and feed transition geometry",
            "measured azimuth/elevation radiation pattern",
            "TI calibration matrix and board-to-radar coordinate transform",
        ],
    }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = [
        "# V0.4.102 EVM PCB 资产审计",
        "",
        f"输入：`{pcbdoc}`",
        "",
        "## 已提取",
        f"- 格式：Altium ASCII PcbDoc；芯片：{chip.get('SOURCEDESIGNATOR')} / {chip.get('SOURCEDESCRIPTION')}。",
        f"- 芯片封装 TX/RX 网络焊盘：{len(package_pads)} 个；坐标单位同时保存为 mil 和 mm。",
        f"- 板级原点：{board.get('ORIGINX')}、{board.get('ORIGINY')}；板框顶点字段已写入 `summary.json`。",
        "",
        "## 不能直接等同的内容",
        "",
        "这些坐标是 AWR2944 芯片封装引脚坐标，不是 PCB 天线单元的相位中心。若直接把它们当作阵列坐标，会把封装内部连接误当成辐射单元，导致 AoA 和波束图结论失真。",
        "",
        "下一步需要从 TX/RX 网络继续追踪 PCB 铜箔/过孔/天线单元，或取得 TI 的天线版图/EM 模型；再与已知角度采集和校准矩阵闭环。",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcbdoc", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.pcbdoc, args.output)
    print(json.dumps({k: result[k] for k in ("status", "chip", "tx_rx_package_pad_count")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
