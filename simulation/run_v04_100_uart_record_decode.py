"""Decode legacy AWR UART TLV record files into point-cloud evidence."""

from __future__ import annotations

import argparse
import csv
import json
import struct
from pathlib import Path

from simulation.run_v04_99_custom_record_audit import MAGIC


HEADER_FORMAT = "<8sIIIIIIII"
HEADER_BYTES = struct.calcsize(HEADER_FORMAT)


def decode(path: Path) -> tuple[list[dict], list[dict]]:
    data = path.read_bytes()
    frames, points = [], []
    offset = 0
    while True:
        start = data.find(MAGIC, offset)
        if start < 0 or start + HEADER_BYTES > len(data):
            break
        header = struct.unpack_from(HEADER_FORMAT, data, start)
        packet_length = header[2]
        if packet_length < HEADER_BYTES or start + packet_length > len(data):
            break
        frame_number, detected_count, tlv_count = header[4], header[6], header[7]
        payload = data[start + HEADER_BYTES:start + packet_length]
        cursor = 0
        frame_points = []
        tlv_types = []
        for _ in range(tlv_count):
            if cursor + 8 > len(payload):
                break
            tlv_type, tlv_length = struct.unpack_from("<II", payload, cursor)
            cursor += 8
            body = payload[cursor:cursor + tlv_length]
            cursor += tlv_length
            tlv_types.append(tlv_type)
            if tlv_type == 1:
                for point_index in range(0, len(body) - 15, 16):
                    x, y, z, velocity = struct.unpack_from("<ffff", body, point_index)
                    point = {"source_file": str(path.resolve()), "frame_num": frame_number, "point_index": point_index // 16, "x_m": x, "y_m": y, "z_m": z, "velocity_mps": velocity}
                    points.append(point); frame_points.append(point)
        frames.append({"source_file": str(path.resolve()), "frame_num": frame_number, "packet_length": packet_length, "declared_detected_count": detected_count, "tlv_count": tlv_count, "tlv_types": tlv_types, "decoded_point_count": len(frame_points)})
        offset = start + packet_length
    return frames, points


def run(inputs: list[Path], output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    all_frames, all_points = [], []
    for path in inputs:
        frames, points = decode(path); all_frames.extend(frames); all_points.extend(points)
    with (output / "uart_frames.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = ["source_file", "frame_num", "packet_length", "declared_detected_count", "tlv_count", "tlv_types", "decoded_point_count"]; writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(all_frames)
    with (output / "uart_points.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = ["source_file", "frame_num", "point_index", "x_m", "y_m", "z_m", "velocity_mps"]; writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(all_points)
    summary = {"status": "completed_uart_tlv_record_decode", "input_files": [str(path.resolve()) for path in inputs], "frame_count": len(all_frames), "point_count": len(all_points), "source_kind": "board_uart_tlv_point_cloud_not_adc_iq", "dca1000_raw_iq_ready": False, "hardware_aoa_validated": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text("# V0.4.100 UART TLV record 解码\n\n本阶段按 legacy `radar_server.py` 的 40 字节 UART frame header 和 TLV type 1 点云格式解码 record 文件。\n\n" + f"解码帧数：{len(all_frames)}；点数：{len(all_points)}。这些点是板载 UART 输出的 detected points，不是 DCA1000 ADC IQ，不能直接用于原始 AoA 重建。\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input", type=Path, action="append", required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run([path.resolve() for path in args.input], args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
