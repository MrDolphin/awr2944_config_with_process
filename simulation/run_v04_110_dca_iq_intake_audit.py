"""Inventory capture files before admitting them to the DCA1000 AoA path.

The classifier is intentionally conservative: a binary file is only a
``dca1000_raw_iq_candidate`` when it is not a known UART point-cloud record
and its size is compatible with the configured ADC word geometry.  This is a
readiness gate, not a decoder and not proof that a file contains ADC IQ.
"""

from __future__ import annotations

import argparse
import csv
import json
import struct
from pathlib import Path


# Legacy radar_server uses the standard mmWave SDK sync pattern in its
# 40-byte UART packet header (stored as little-endian bytes).
UART_MAGIC = bytes.fromhex("02 01 04 03 06 05 08 07")


def _is_uart_record(path: Path) -> bool:
    try:
        with path.open("rb") as handle:
            head = handle.read(8)
        return head.startswith(UART_MAGIC)
    except OSError:
        return False


def classify(path: Path, *, adc_samples: int, rx_count: int, chirps_per_frame: int) -> dict:
    size = path.stat().st_size
    row = {"path": str(path.resolve()), "name": path.name, "suffix": path.suffix.lower(), "size_bytes": size,
           "classification": "unknown_requires_manifest", "size_multiple_of_chirp_bytes": False,
           "size_multiple_of_frame_bytes": False, "notes": ""}
    if path.suffix.lower() != ".bin":
        row["notes"] = "not a binary capture file"
        return row
    if _is_uart_record(path):
        row.update(classification="uart_point_cloud_record", notes="legacy radar_server framed TLV; not DCA1000 ADC IQ")
        return row
    # Complex int16: I and Q are two int16 words per RX sample.
    chirp_bytes = adc_samples * rx_count * 2 * 2
    frame_bytes = chirp_bytes * chirps_per_frame
    row["size_multiple_of_chirp_bytes"] = size % chirp_bytes == 0
    row["size_multiple_of_frame_bytes"] = size % frame_bytes == 0
    if row["size_multiple_of_chirp_bytes"]:
        row.update(classification="dca1000_raw_iq_candidate", notes="size compatible with complex int16 ADC words; manifest and wire-order verification still required")
    else:
        row["notes"] = "size is not compatible with configured complex int16 ADC words"
    return row


def run(roots: list[Path], output: Path, *, adc_samples: int = 656, rx_count: int = 4, chirps_per_frame: int = 240) -> dict:
    paths = sorted({path for root in roots if root.exists() for path in root.rglob("*") if path.is_file() and path.suffix.lower() in {".bin", ".h5", ".hdf5"}})
    rows = [classify(path, adc_samples=adc_samples, rx_count=rx_count, chirps_per_frame=chirps_per_frame) for path in paths]
    output.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0]) if rows else ["path", "name", "suffix", "size_bytes", "classification", "size_multiple_of_chirp_bytes", "size_multiple_of_frame_bytes", "notes"]
    with (output / "capture_inventory.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    counts = {}
    for row in rows:
        counts[row["classification"]] = counts.get(row["classification"], 0) + 1
    summary = {"status": "completed_dca_iq_intake_audit", "roots": [str(root.resolve()) for root in roots], "file_count": len(rows), "classification_counts": counts, "geometry": {"adc_samples": adc_samples, "rx_count": rx_count, "chirps_per_frame": chirps_per_frame, "complex_int16_chirp_bytes": adc_samples * rx_count * 4, "complex_int16_frame_bytes": adc_samples * rx_count * 4 * chirps_per_frame}, "dca1000_adc_iq_validated": False, "channel_order_verified": False, "ti_calibration_verified": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    uart = counts.get("uart_point_cloud_record", 0); candidates = counts.get("dca1000_raw_iq_candidate", 0)
    lines = ["# V0.4.110 DCA1000 ADC IQ 接入审计", "", "本阶段只读扫描采集目录，按文件格式和配置尺寸进行保守分类，不修改原始数据。‘候选’不等于已解码，也不等于已完成 AoA 验证。", "", f"扫描文件：{len(rows)}；UART 点云记录：{uart}；DCA1000 原始 IQ 候选：{candidates}。", "", "## 分类含义", "", "- `uart_point_cloud_record`：识别到旧版 radar_server 的帧/TLV 点云记录，只能用于点云回放或协议对照，不能当作 ADC IQ。", "- `dca1000_raw_iq_candidate`：不是已知 UART 记录且文件大小符合 complex-int16 ADC 字节几何，只能进入下一步 manifest、端序、LVDS wire order 和帧边界检查。", "- `unknown_requires_manifest`：缺少足够证据，不能进入 AoA。", "", "## 本阶段验收门", "", "1. 保存采集 CFG 快照和 DCA1000 配置；2. 记录 ADC 位宽、I/Q 端序、RX 数量、样本数、chirp/frame；3. 通过已知角度目标确认通道顺序；4. 记录 TI 校准命令、校准结果和幅相矩阵；5. 绑定时间戳、IMU 姿态和安装角度。", "", "## 证据边界", "", "文件尺寸只能说明‘可能符合’，无法证明文件中确实是 ADC IQ。没有 `channel_order_verified` 和 `ti_calibration_verified` 时，不得把后续角度结果写成 AWR2944P 硬件精度。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--adc-samples", type=int, default=656)
    parser.add_argument("--rx-count", type=int, default=4)
    parser.add_argument("--chirps-per-frame", type=int, default=240)
    args = parser.parse_args()
    result = run([path.resolve() for path in args.root], args.output.resolve(), adc_samples=args.adc_samples, rx_count=args.rx_count, chirps_per_frame=args.chirps_per_frame)
    print(json.dumps(result, ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
