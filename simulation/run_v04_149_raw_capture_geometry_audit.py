"""Audit raw capture byte geometry before IQ decoding."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from simulation.run_v04_111_cfg_linked_manifest import parse_cfg


UART_SYNC = bytes.fromhex("02 01 04 03 06 05 08 07")


def run(case_dir: Path, output: Path) -> dict:
    case_dir = case_dir.resolve(); output.mkdir(parents=True, exist_ok=True)
    manifest_path = case_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else {}
    cfg = case_dir / str(manifest.get("capture", {}).get("cfg_file", "profile.cfg"))
    capture = case_dir / str(manifest.get("capture_file", "capture.bin"))
    issues: list[str] = []
    cfg_info = parse_cfg(cfg) if cfg.is_file() else {}
    # Complex int16 ADC payload: I and Q are two int16 values = 4 bytes.
    expected_frame_bytes = int(cfg_info.get("adc_samples", 0)) * int(cfg_info.get("rx_count", 0)) * 4 * int(cfg_info.get("chirps_per_frame", 0))
    size = capture.stat().st_size if capture.is_file() else 0
    if not manifest_path.is_file(): issues.append("manifest_missing")
    if not cfg.is_file(): issues.append("cfg_missing")
    if not capture.is_file(): issues.append("capture_missing")
    elif size == 0: issues.append("capture_empty")
    elif capture.read_bytes()[:8] == UART_SYNC: issues.append("input_is_uart_point_cloud")
    if expected_frame_bytes <= 0: issues.append("invalid_expected_frame_geometry")
    complete_frames = size // expected_frame_bytes if expected_frame_bytes else 0
    remainder = size % expected_frame_bytes if expected_frame_bytes else size
    if capture.is_file() and size > 0 and remainder: issues.append("payload_not_multiple_of_frame_bytes")
    result = {
        "status": "raw_capture_geometry_ready_for_decode" if not issues else "raw_capture_geometry_incomplete",
        "case_dir": str(case_dir), "manifest": str(manifest_path), "cfg": str(cfg), "capture": str(capture),
        "capture_bytes": size, "expected_frame_bytes": expected_frame_bytes, "complete_frames": complete_frames,
        "remainder_bytes": remainder, "cfg_geometry": cfg_info, "issues": issues,
        "ready_for_decode": not issues, "hardware_aoa_validated": False,
    }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.149 原始 capture.bin 几何审计\n\n"
        f"状态：`{result['status']}`\n\n"
        f"文件大小：`{size}` bytes；期望每帧：`{expected_frame_bytes}` bytes；完整帧：`{complete_frames}`；余数：`{remainder}`。\n\n"
        f"问题：`{'、'.join(issues) if issues else '无'}`。\n\n"
        "本阶段只做字节几何和 UART 头识别，不证明文件一定是有效 ADC IQ，不验证通道顺序、TI 校准或硬件 AoA。\n",
        encoding="utf-8",
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--case", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.case, args.output), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
