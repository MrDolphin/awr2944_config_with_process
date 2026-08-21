"""Compare decoded UART point counts/coordinates with a legacy JSONL point log."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def run(uart_frames_csv: Path, uart_points_csv: Path, reference_jsonl: Path, output: Path) -> dict:
    with uart_frames_csv.open(encoding="utf-8", newline="") as handle:
        frames = list(csv.DictReader(handle))
    with uart_points_csv.open(encoding="utf-8", newline="") as handle:
        points = list(csv.DictReader(handle))
    reference = [json.loads(line) for line in reference_jsonl.read_text(encoding="utf-8").splitlines() if line.strip()]
    # Use record_01 for the comparison because test_01.jsonl covers frame 4..389,
    # while all three record files reuse frame numbers independently.
    selected = [row for row in frames if Path(row["source_file"]).name == "record_01.bin"]
    decoded_counts = {int(row["frame_num"]): int(row["decoded_point_count"]) for row in selected}
    decoded_points = {}
    for row in points:
        if Path(row["source_file"]).name == "record_01.bin":
            decoded_points.setdefault(int(row["frame_num"]), []).append(tuple(float(row[key]) for key in ("x_m", "y_m", "z_m", "velocity_mps")))
    rows = []
    for item in reference:
        frame = int(item["frame_num"]); expected = [(float(point.get("x", 0.0)), float(point.get("y", 0.0)), float(point.get("z", 0.0)), float(point.get("v", 0.0))) for point in item.get("points", [])]; actual = decoded_points.get(frame, [])
        count_match = len(expected) == len(actual)
        coordinate_rmse = None
        if count_match and expected:
            # Compare sorted coordinates so point order does not matter.
            expected_sorted = sorted(expected); actual_sorted = sorted(actual)
            errors = [sum((a - b) ** 2 for a, b in zip(left, right)) for left, right in zip(expected_sorted, actual_sorted)]
            coordinate_rmse = math.sqrt(sum(errors) / len(errors))
        rows.append({"frame_num": frame, "reference_point_count": len(expected), "decoded_point_count": decoded_counts.get(frame), "count_match": count_match if frame in decoded_counts else False, "coordinate_rmse": coordinate_rmse})
    overlap = [row for row in rows if row["decoded_point_count"] is not None]
    summary = {"status": "completed_uart_reference_comparison", "reference_frames": len(reference), "decoded_record01_frames": len(selected), "overlap_frames": len(overlap), "point_count_match_frames": sum(row["count_match"] for row in overlap), "coordinate_comparable_frames": sum(row["coordinate_rmse"] is not None for row in overlap), "source_type": "uart_point_cloud_cross_check_not_adc_iq", "hardware_aoa_validated": False}
    output.mkdir(parents=True, exist_ok=True)
    with (output / "uart_reference_comparison.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["frame_num", "reference_point_count", "decoded_point_count", "count_match", "coordinate_rmse"]); writer.writeheader(); writer.writerows(rows)
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text("# V0.4.101 UART 点云参考交叉验证\n\n将 `record_01.bin` 解码结果与同目录 `test_01.jsonl` 逐帧比较。两者都属于板载 UART 点云记录，不涉及 DCA1000 ADC IQ。\n\n" + f"参考帧：{len(reference)}；record_01 帧：{len(selected)}；帧号重叠：{len(overlap)}；点数一致帧：{summary['point_count_match_frames']}；可比较坐标帧：{summary['coordinate_comparable_frames']}。\n\n点数或坐标不一致表示两份记录可能不是同一次采集/同一配置，不能据此判定 UART 解码错误；需要采集时间、CFG 和 frame source 对齐后再做严格逐帧验证。\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--uart-frames", type=Path, required=True); parser.add_argument("--uart-points", type=Path, required=True); parser.add_argument("--reference-jsonl", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.uart_frames.resolve(), args.uart_points.resolve(), args.reference_jsonl.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
