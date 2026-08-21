"""Extract AWR2944EVM RF array candidates from an Altium ASCII PCB document."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

MIL_TO_MM = 0.0254
RF_NETS = {
    107: ("TX4", "TX"),
    108: ("TX3", "TX"),
    109: ("TX2", "TX"),
    110: ("TX1", "TX"),
    117: ("RX4", "RX"),
    118: ("RX3", "RX"),
    119: ("RX2", "RX"),
    120: ("RX1", "RX"),
}


def _fields(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for item in line.strip("|").split("|"):
        if "=" in item:
            key, value = item.split("=", 1)
            result[key] = value.strip()
    return result


def _mil(value: str | None) -> float | None:
    if value is None:
        return None
    return float(value.removesuffix("mil"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def extract(pcb_doc: Path, output: Path) -> dict:
    board_origin = None
    board_vertices: list[tuple[float, float]] = []
    pads: dict[int, dict] = {}
    regions: dict[int, dict] = {}

    with pcb_doc.open(encoding="utf-8", errors="ignore") as handle:
        lines = handle
        for line in lines:
            fields = _fields(line)
            record = fields.get("RECORD")
            if record == "Board" and board_origin is None and fields.get("ORIGINX"):
                board_origin = (_mil(fields["ORIGINX"]), _mil(fields["ORIGINY"]))
                for index in range(32):
                    x = _mil(fields.get(f"VX{index}"))
                    y = _mil(fields.get(f"VY{index}"))
                    if x is not None and y is not None:
                        board_vertices.append((x, y))
            if record not in {"Pad", "Region"} or "NET" not in fields:
                continue
            net = int(fields["NET"])
            if net not in RF_NETS:
                continue
            if record == "Pad" and net not in pads:
                pads[net] = {
                    "component": fields.get("COMPONENT", ""),
                    "pad": fields.get("NAME", ""),
                    "x_mil": _mil(fields.get("X")),
                    "y_mil": _mil(fields.get("Y")),
                    "layer": fields.get("LAYER", ""),
                }
            if record == "Region" and net not in regions:
                vertices = []
                for index in range(int(fields.get("MAINCONTOURVERTEXCOUNT", "0"))):
                    x = _mil(fields.get(f"VX{index}"))
                    y = _mil(fields.get(f"VY{index}"))
                    if x is not None and y is not None:
                        vertices.append((x, y))
                if vertices:
                    regions[net] = {"vertices": vertices, "layer": fields.get("LAYER", "")}

    if board_origin is None:
        raise ValueError("ASCII PCB document has no board origin")
    missing = [name for net, (name, _) in RF_NETS.items() if net not in pads or net not in regions]
    if missing:
        raise ValueError(f"missing RF pad/region records: {', '.join(missing)}")

    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for net, (name, role) in RF_NETS.items():
        vertices = regions[net]["vertices"]
        xs = [item[0] for item in vertices]
        ys = [item[1] for item in vertices]
        center_x = sum(xs) / len(xs)
        center_y = sum(ys) / len(ys)
        pad = pads[net]
        rows.append({
            "net_id": net, "channel": name, "role": role,
            "chip_component": pad["component"], "chip_pad": pad["pad"],
            "chip_pad_x_mil": pad["x_mil"], "chip_pad_y_mil": pad["y_mil"],
            "region_center_x_mil": center_x, "region_center_y_mil": center_y,
            "region_min_x_mil": min(xs), "region_max_x_mil": max(xs),
            "region_min_y_mil": min(ys), "region_max_y_mil": max(ys),
            "local_x_mm": (center_x - board_origin[0]) * MIL_TO_MM,
            "local_y_mm": (center_y - board_origin[1]) * MIL_TO_MM,
            "region_width_mm": (max(xs) - min(xs)) * MIL_TO_MM,
            "region_height_mm": (max(ys) - min(ys)) * MIL_TO_MM,
            "layer": regions[net]["layer"],
            "evidence": "ASCII_PcbDoc RF net + Pad + Region",
        })
    rows.sort(key=lambda row: (row["role"], row["local_x_mm"]))
    fields = list(rows[0])
    with (output / "rf_array_candidates.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    tx = {row["channel"]: row for row in rows if row["role"] == "TX"}
    rx = {row["channel"]: row for row in rows if row["role"] == "RX"}
    reference_x = tx["TX1"]["local_x_mm"] + rx["RX1"]["local_x_mm"]
    reference_y = tx["TX1"]["local_y_mm"] + rx["RX1"]["local_y_mm"]
    virtual_rows = []
    index = 0
    for tx_name in ("TX1", "TX2", "TX3", "TX4"):
        for rx_name in ("RX1", "RX2", "RX3", "RX4"):
            virtual_rows.append({
                "virtual_channel": index, "tx": tx_name, "rx": rx_name,
                "x_mm": tx[tx_name]["local_x_mm"] + rx[rx_name]["local_x_mm"] - reference_x,
                "y_mm": tx[tx_name]["local_y_mm"] + rx[rx_name]["local_y_mm"] - reference_y,
                "coordinate_status": "cad_region_centroid_sum_candidate_not_phase_center",
            })
            index += 1
    with (output / "virtual_array_candidates.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(virtual_rows[0]))
        writer.writeheader()
        writer.writerows(virtual_rows)

    board = {
        "source_file": str(pcb_doc),
        "source_sha256": _sha256(pcb_doc),
        "board_origin_mil": {"x": board_origin[0], "y": board_origin[1]},
        "board_outline_bbox_mil": {
            "min_x": min((item[0] for item in board_vertices), default=None),
            "max_x": max((item[0] for item in board_vertices), default=None),
            "min_y": min((item[1] for item in board_vertices), default=None),
            "max_y": max((item[1] for item in board_vertices), default=None),
        },
        "coordinate_definition": "local_x_mm=(board_x-origin_x)*0.0254; local_y_mm=(board_y-origin_y)*0.0254; z=0; phase_center_not_claimed",
        "frequency_ghz": 77.0,
        "wavelength_mm": 299.792458 / 77.0,
        "status": "candidate_geometry_extracted_from_ascii_pcb",
        "hardware_validated": False,
    }
    (output / "antenna_geometry.json").write_text(json.dumps(board, indent=2, ensure_ascii=False), encoding="utf-8")
    yaml = [
        "source: PROC113D_ASCII.PcbDoc",
        f"source_sha256: {board['source_sha256']}",
        "coordinate_system:",
        "  x: board X minus ORIGINX, millimetres",
        "  y: board Y minus ORIGINY, millimetres",
        "  z: 0 (PCB plane; phase-centre height unknown)",
        "  boresight: not established from CAD alone",
        "array_status: candidate_geometry_extracted_from_ascii_pcb",
        "hardware_validated: false",
        "channels:",
    ]
    for row in rows:
        yaml.extend([
            f"  - channel: {row['channel']}",
            f"    role: {row['role']}",
            f"    net_id: {row['net_id']}",
            f"    chip_pad: {row['chip_component']}.{row['chip_pad']}",
            f"    x_mm: {row['local_x_mm']:.6f}",
            f"    y_mm: {row['local_y_mm']:.6f}",
            "    z_mm: 0.0",
        ])
    (output / "antenna_geometry.yaml").write_text("\n".join(yaml) + "\n", encoding="utf-8")

    rx = [row for row in rows if row["role"] == "RX"]
    tx_az = [row for row in rows if row["channel"] in {"TX1", "TX3", "TX4"}]
    tx_elev = next(row for row in rows if row["channel"] == "TX2")
    report = f"""# V0.4.85 PCB/CAD 阵列候选提取报告

## 输入资料

- Altium ASCII PCB：`{pcb_doc}`
- SHA-256：`{board['source_sha256']}`
- 解析对象：`RF net -> chip Pad -> antenna Region`

## 已确认的几何证据

ASCII PCB 中存在 8 条 RF 网络：TX1–TX4、RX1–RX4。每条网络都能关联到 AWR 芯片焊盘和一个 TOP 层 Region，因此本阶段不是根据图片估计阵元位置，而是从 CAD 导出的坐标字段直接提取。

- RX 阵元局部 x 坐标约为 {min(row['local_x_mm'] for row in rx):.3f}、{sorted(row['local_x_mm'] for row in rx)[1]:.3f}、{sorted(row['local_x_mm'] for row in rx)[2]:.3f}、{max(row['local_x_mm'] for row in rx):.3f} mm。
- RX 相邻间距约为 {(sorted(row['local_x_mm'] for row in rx)[1] - sorted(row['local_x_mm'] for row in rx)[0]):.3f} mm，即约 0.49 λ（77 GHz）。
- TX1/TX3/TX4 位于近似同一 y 行，横向相邻间距约为 {(sorted(row['local_x_mm'] for row in tx_az)[1] - sorted(row['local_x_mm'] for row in tx_az)[0]):.3f} mm，即约 2.01 λ；这与 3 个方位 TX 的候选布局一致。
- TX2 的 y 坐标相对该行偏移约 {abs(tx_elev['local_y_mm'] - tx_az[0]['local_y_mm']):.3f} mm，且 CFG 的 3Azim/1Elev 配置将 TX2 作为独立通道，因此 TX2 是俯仰 TX 候选。最终俯仰相位中心仍需实测确认。

## 如何接入仿真

`rf_array_candidates.csv` 可作为 TX/RX 几何候选输入，`virtual_array_candidates.csv` 给出 4×4 虚拟通道的质心和坐标。当前只替换阵元位置，不替换 TI SDK 的通道校准、相位中心、天线方向图或生产测试数据。建议先比较理想半波长阵列与本候选几何的波束图、AoA 偏差和海杂波点云扩散，再用角反射器实测冻结坐标变换。

## 限制

本结果仍不是“已校准实测阵列”：PCB Region 的几何中心不一定等于电磁相位中心；板面坐标到雷达前视坐标的旋转、板面法向、天线极化、封装影响和 TX/RX 通道幅相校准尚未由该文件证明。`hardware_validated` 保持为 `false`。
"""
    (output / "output_analysis.md").write_text(report, encoding="utf-8")
    result = {
        "status": "completed_pcb_rf_array_candidate_extraction",
        "source_sha256": board["source_sha256"],
        "channel_count": len(rows), "tx_count": sum(row["role"] == "TX" for row in rows),
        "rx_count": sum(row["role"] == "RX" for row in rows),
        "hardware_validated": False,
        "virtual_channel_count": len(virtual_rows),
    }
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcb-doc", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(extract(args.pcb_doc.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
