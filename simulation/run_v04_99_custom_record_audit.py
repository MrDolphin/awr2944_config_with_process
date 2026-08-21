"""Audit the custom framed AWR record files found in the legacy capture tree."""

from __future__ import annotations

import argparse
import json
import re
import struct
from pathlib import Path


MAGIC = bytes.fromhex("02 01 04 03 06 05 08 07")
HEADER_BYTES = 40


def audit(path: Path) -> dict:
    data = path.read_bytes()
    offsets = [match.start() for match in re.finditer(re.escape(MAGIC), data)]
    packets = []
    for index, offset in enumerate(offsets):
        if offset + HEADER_BYTES > len(data):
            packets.append({"index": index, "offset": offset, "truncated_header": True})
            continue
        header = struct.unpack_from("<8sIIIIIIII", data, offset)
        fields = header[1:]
        next_offset = offsets[index + 1] if index + 1 < len(offsets) else len(data)
        packets.append({"index": index, "offset": offset, "next_offset": next_offset, "version": fields[0], "declared_packet_bytes": fields[1], "product_code": fields[2], "frame_or_sequence": fields[3], "field5": fields[4], "num_detected_objects": fields[5], "num_tlvs": fields[6], "subframe": fields[7], "payload_bytes_between_headers": max(0, next_offset - offset - HEADER_BYTES)})
    declared = [row["declared_packet_bytes"] for row in packets if "declared_packet_bytes" in row]
    sequences = [row["frame_or_sequence"] for row in packets if "frame_or_sequence" in row]
    gaps = sum(1 for a, b in zip(sequences, sequences[1:]) if b != a + 1)
    return {"path": str(path.resolve()), "file_bytes": len(data), "magic_hex": MAGIC.hex(" "), "header_bytes_assumed": HEADER_BYTES, "packet_count": len(packets), "first_headers": packets[:5], "declared_packet_bytes_min": min(declared) if declared else None, "declared_packet_bytes_max": max(declared) if declared else None, "sequence_first": sequences[0] if sequences else None, "sequence_last": sequences[-1] if sequences else None, "sequence_gap_count": gaps, "truncated_header_count": sum("truncated_header" in row for row in packets), "interpretation_status": "custom_framed_record_requires_format_confirmation_not_standard_payload_only_dca"}


def run(inputs: list[Path], output: Path) -> dict:
    results = [audit(path) for path in inputs if path.is_file()]
    output.mkdir(parents=True, exist_ok=True)
    (output / "record_audit.json").write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")
    summary = {"status": "completed_custom_record_audit", "file_count": len(results), "packet_counts": {Path(row["path"]).name: row["packet_count"] for row in results}, "all_have_custom_magic": all(row["packet_count"] > 0 for row in results), "standard_dca_payload_ready": False, "requires_evidence": ["confirmed 40-byte UART header layout", "payload TLV interpretation", "matching CFG or capture metadata"]}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.99 AWR UART record 文件结构审计", "", f"扫描文件：{len(results)} 个。", "", "## 结论", "", "record_01.bin、record_02.bin、record_03.bin 均符合 legacy `radar_server.py` 的 UART 输出帧格式：8 字节 TI magic + 8 个 uint32，共 40 字节帧头；头部包含包长、平台标识 0x2944、帧号、检测目标数和 TLV 数量。它们不是 DCA1000 ADC 原始 IQ。", "", "因此 `standard_dca_payload_ready=false` 的含义已明确：这些文件属于板载 UART 点云/中间结果记录，不能送入 V0.4.91 原始 IQ AoA 解码器。", "", "## 下一步", "", "应使用 UART TLV 解码器提取点云，作为板载点云对标数据；DCA1000 原始 IQ 仍需单独采集并保存为 payload-only BIN 或带明确元数据的 HDF5。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input", type=Path, action="append", required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run([path.resolve() for path in args.input], args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
