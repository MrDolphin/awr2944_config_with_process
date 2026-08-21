"""Audit the custom framed AWR record files found in the legacy capture tree."""

from __future__ import annotations

import argparse
import json
import re
import struct
from pathlib import Path


MAGIC = bytes.fromhex("02 01 04 03 06 05 08 07")
HEADER_BYTES = 48


def audit(path: Path) -> dict:
    data = path.read_bytes()
    offsets = [match.start() for match in re.finditer(re.escape(MAGIC), data)]
    packets = []
    for index, offset in enumerate(offsets):
        if offset + HEADER_BYTES > len(data):
            packets.append({"index": index, "offset": offset, "truncated_header": True})
            continue
        fields = struct.unpack_from("<12I", data, offset)
        next_offset = offsets[index + 1] if index + 1 < len(offsets) else len(data)
        packets.append({"index": index, "offset": offset, "next_offset": next_offset, "frame_or_sequence": fields[5], "declared_packet_bytes": fields[3], "product_code": fields[4], "field6": fields[6], "field7": fields[7], "field8": fields[8], "field9": fields[9], "field10": fields[10], "field11": fields[11], "payload_bytes_between_headers": max(0, next_offset - offset - HEADER_BYTES)})
    declared = [row["declared_packet_bytes"] for row in packets if "declared_packet_bytes" in row]
    sequences = [row["frame_or_sequence"] for row in packets if "frame_or_sequence" in row]
    gaps = sum(1 for a, b in zip(sequences, sequences[1:]) if b != a + 1)
    return {"path": str(path.resolve()), "file_bytes": len(data), "magic_hex": MAGIC.hex(" "), "header_bytes_assumed": HEADER_BYTES, "packet_count": len(packets), "first_headers": packets[:5], "declared_packet_bytes_min": min(declared) if declared else None, "declared_packet_bytes_max": max(declared) if declared else None, "sequence_first": sequences[0] if sequences else None, "sequence_last": sequences[-1] if sequences else None, "sequence_gap_count": gaps, "truncated_header_count": sum("truncated_header" in row for row in packets), "interpretation_status": "custom_framed_record_requires_format_confirmation_not_standard_payload_only_dca"}


def run(inputs: list[Path], output: Path) -> dict:
    results = [audit(path) for path in inputs if path.is_file()]
    output.mkdir(parents=True, exist_ok=True)
    (output / "record_audit.json").write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")
    summary = {"status": "completed_custom_record_audit", "file_count": len(results), "packet_counts": {Path(row["path"]).name: row["packet_count"] for row in results}, "all_have_custom_magic": all(row["packet_count"] > 0 for row in results), "standard_dca_payload_ready": False, "requires_evidence": ["custom 48-byte header layout", "payload sample interpretation", "chirps/samples/RX/TX from matching CFG or capture metadata"]}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.99 自定义 AWR record 文件结构审计", "", f"扫描文件：{len(results)} 个。", "", "## 结论", "", "record_01.bin、record_02.bin、record_03.bin 均包含重复的 8 字节 AWR magic 和 48 字节候选帧头，不符合当前 V0.4.91 直接读取 payload-only DCA1000 BIN 的输入假设。", "", "当前只确认了文件结构和序号连续性，尚未确认 48 字节字段含义、payload 是否为 ADC IQ、RX/TX 排列或 chirp/sample 边界。因此 `standard_dca_payload_ready=false`。", "", "## 下一步", "", "需要将这些 record 文件与采集脚本的元数据、实际 CFG 和 UDP 保存模式对应起来；确认后再写专用解包器，不直接删头或猜测 IQ 维度。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input", type=Path, action="append", required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run([path.resolve() for path in args.input], args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
