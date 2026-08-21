"""Build an evidence-grade table for PCB RF channels and virtual channels."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)


def pdf_support(document_rows: list[dict[str, str]]) -> tuple[bool, str]:
    for row in document_rows:
        if row.get("format") == "pdf" and "GCPW" in row.get("snippets", "") and "antenna" in row.get("snippets", "").lower():
            return True, f"{row['path']} page {row.get('page', '')}: {row.get('snippets', '')}"
    return False, ""


def run(mapping_path: Path, regions_path: Path, documents_path: Path, output: Path) -> dict:
    mapping = read_csv(mapping_path)
    regions = {row["antenna"].upper(): row for row in read_csv(regions_path)}
    documents = read_csv(documents_path)
    schematic_support, schematic_source = pdf_support(documents)
    names = sorted(regions)
    channel_rows = []
    for row in mapping:
        tx, rx = row["tx_name"].upper(), row["rx_name"].upper()
        pcb_ok = tx in regions and rx in regions
        status = "document_supported_candidate" if pcb_ok and schematic_support else "pcb_cfg_candidate"
        channel_rows.append({
            "virtual_input_index": row["virtual_input_index"], "tx_name": tx, "rx_name": rx,
            "cfg_row": row["cfg_row"], "cfg_column": row["cfg_column"],
            "pcb_region_evidence": "confirmed_region_extraction" if pcb_ok else "missing_region",
            "schematic_evidence": "gcpw_to_antenna_context" if schematic_support else "missing_context",
            "phase_center_evidence": "not_available", "channel_mapping_evidence": status,
            "calibration_evidence": "not_available", "overall_status": "candidate_only",
            "source_note": schematic_source if schematic_support else "",
        })
    output.mkdir(parents=True, exist_ok=True)
    channel_fields = list(channel_rows[0]) if channel_rows else ["virtual_input_index"]
    write_csv(output / "evidence_grade.csv", channel_rows, channel_fields)
    gaps = [
        {"gap_id": "phase_center", "item": "每个 TX/RX 的电气相位中心坐标", "status": "missing", "required_evidence": "封装/天线版图或近场测量", "blocks_real_array_claim": "yes"},
        {"gap_id": "pin_mapping", "item": "芯片引脚到 TX/RX 天线馈点的连通关系", "status": "candidate", "required_evidence": "Altium ASCII SchDoc、网表或原理图引脚级解析", "blocks_real_array_claim": "yes"},
        {"gap_id": "coordinate_frame", "item": "PCB 坐标到雷达坐标的原点、法向和镜像", "status": "candidate", "required_evidence": "装配基准和机械测量", "blocks_real_array_claim": "yes"},
        {"gap_id": "channel_polarity", "item": "TX/RX 顺序、I/Q 符号和 LVDS lane 语义", "status": "missing", "required_evidence": "DCA1000 原始 bin + 已知角目标", "blocks_real_array_claim": "yes"},
        {"gap_id": "complex_calibration", "item": "每通道复数幅相校准矩阵", "status": "missing", "required_evidence": "TI 校准流程或实测角反射器", "blocks_real_array_claim": "yes"},
        {"gap_id": "pattern", "item": "真实 TX/RX 天线方向图和互耦", "status": "missing", "required_evidence": "TI 方向图数据或暗室/近场测试", "blocks_real_array_claim": "no"},
    ]
    write_csv(output / "gap_register.csv", gaps, list(gaps[0]))
    contract = {"version": "v04_31", "required_for_real_aoa": ["phase_center", "pin_mapping", "coordinate_frame", "channel_polarity", "complex_calibration"], "accepted_inputs": ["altium_ascii_schdoc_or_netlist", "known_angle_corner_reflector_capture", "tx_rx_calibration_measurement", "mechanical_coordinate_datum"], "current_array_input_status": "candidate_only"}
    (output / "calibration_input_contract.json").write_text(json.dumps(contract, indent=2, ensure_ascii=False), encoding="utf-8")
    summary = {"status": "completed_evidence_grade", "virtual_channel_count": len(channel_rows), "rf_region_count": len(names), "schematic_gcpw_support": schematic_support, "all_channels_candidate_only": all(row["overall_status"] == "candidate_only" for row in channel_rows), "real_array_confirmed": False, "missing_gap_count": sum(row["status"] == "missing" for row in gaps)}
    (output / "evidence_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.31 硬件阵列证据等级", "", f"- 虚拟通道：{summary['virtual_channel_count']}", f"- PCB RF 区域：{summary['rf_region_count']}", f"- 原理图 GCPW 到天线文字支持：{'是' if schematic_support else '否'}", f"- 当前真实阵列确认：否", f"- 缺失关键证据项：{summary['missing_gap_count']}", "", "## 当前结论", "", "每个 TX/RX 虚拟通道都有 PCB 区域、CFG 行列和原理图文字的候选支持，但相位中心、引脚级连通、坐标基准、通道极性和复数校准仍未完成。", "当前数据可用于候选几何敏感性仿真，不可宣称为 AWR2944P 实测阵列坐标或实测 AoA 精度。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mapping", type=Path, required=True)
    parser.add_argument("--regions", type=Path, required=True)
    parser.add_argument("--documents", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.mapping.resolve(), args.regions.resolve(), args.documents.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
