#!/usr/bin/env python3
"""First-pass, format-safe analysis for a DCA1000 ADC payload ``.bin`` file.

This tool intentionally does not claim a final LVDS lane/IQ ordering.  It
checks capture integrity before Range/Doppler/AoA processing and writes a
machine-readable JSON report plus ``output_analysis.md`` next to the capture.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path
from typing import Any

try:
    from tools.dca1000_capture import parse_radar_cfg
except ModuleNotFoundError:  # Direct ``python tools/analyze_adc_capture.py``.
    from dca1000_capture import parse_radar_cfg


INT16_BYTES = 2
SATURATION_ABS = 32760


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bin", required=True, dest="bin_path", help="Header-stripped DCA1000 ADC .bin file.")
    parser.add_argument("--cfg", required=True, help="CFG used for this acquisition.")
    parser.add_argument("--metadata", help="Companion capture .json; defaults to .bin sibling with .json suffix.")
    parser.add_argument("--output-dir", help="Defaults to the capture directory.")
    return parser.parse_args(argv)


def load_json(path: Path | None) -> dict[str, Any]:
    if path is None or not path.is_file():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, dict) else {}


def _word_statistics(path: Path) -> dict[str, Any]:
    count = 0
    total = 0
    total_sq = 0
    minimum: int | None = None
    maximum: int | None = None
    saturation_count = 0
    even_total = even_total_sq = even_count = 0
    odd_total = odd_total_sq = odd_count = 0

    with path.open("rb") as handle:
        while chunk := handle.read(1024 * 1024):
            usable = len(chunk) - (len(chunk) % INT16_BYTES)
            if not usable:
                continue
            values = struct.unpack(f"<{usable // INT16_BYTES}h", chunk[:usable])
            # The word index is not part of any calculation.  Iterating values
            # directly also avoids an unnecessary tuple-unpack dependency in
            # post-capture analysis runs.
            for value in values:
                count += 1
                total += value
                total_sq += value * value
                minimum = value if minimum is None else min(minimum, value)
                maximum = value if maximum is None else max(maximum, value)
                saturation_count += int(abs(value) >= SATURATION_ABS)
                if (count - 1) % 2 == 0:
                    even_count += 1
                    even_total += value
                    even_total_sq += value * value
                else:
                    odd_count += 1
                    odd_total += value
                    odd_total_sq += value * value

    def moments(n: int, value_total: int, value_sq_total: int) -> dict[str, float | None]:
        if not n:
            return {"mean": None, "rms": None, "std": None}
        mean = value_total / n
        rms = math.sqrt(value_sq_total / n)
        std = math.sqrt(max(0.0, value_sq_total / n - mean * mean))
        return {"mean": mean, "rms": rms, "std": std}

    return {
        "count": count,
        "minimum": minimum,
        "maximum": maximum,
        "saturation_count": saturation_count,
        "saturation_fraction": saturation_count / count if count else None,
        **moments(count, total, total_sq),
        "even_word": moments(even_count, even_total, even_total_sq),
        "odd_word": moments(odd_count, odd_total, odd_total_sq),
    }


def _frame_candidates(file_bytes: int, radar_cfg: dict[str, Any], duration_s: float | None) -> list[dict[str, Any]]:
    samples = int(radar_cfg.get("num_adc_samples", 0) or 0)
    rx = int(radar_cfg.get("num_rx", 0) or 0)
    chirps = int(radar_cfg.get("num_chirps_per_frame", 0) or 0)
    if not samples or not rx or not chirps:
        return []
    scalar_words_per_frame = samples * rx * chirps
    result = []
    for label, bytes_per_scalar in (("one_int16_word_per_sample", 2), ("int16_i_plus_int16_q_per_sample", 4)):
        bytes_per_frame = scalar_words_per_frame * bytes_per_scalar
        full_frames, remainder = divmod(file_bytes, bytes_per_frame)
        item: dict[str, Any] = {
            "interpretation": label,
            "bytes_per_frame": bytes_per_frame,
            "full_frames": full_frames,
            "trailing_bytes": remainder,
        }
        if duration_s and duration_s > 0:
            item["observed_frame_rate_hz"] = full_frames / duration_s
        result.append(item)
    return result


def analyze(bin_path: Path, cfg_path: Path, metadata_path: Path | None = None) -> dict[str, Any]:
    if not bin_path.is_file():
        raise FileNotFoundError(f"capture file not found: {bin_path}")
    if not cfg_path.is_file():
        raise FileNotFoundError(f"cfg file not found: {cfg_path}")
    metadata = load_json(metadata_path)
    radar_cfg = parse_radar_cfg(str(cfg_path))
    duration = metadata.get("duration_s")
    duration_s = float(duration) if isinstance(duration, (int, float)) and duration > 0 else None
    size = bin_path.stat().st_size
    return {
        "analysis_scope": "first_pass_container_and_int16_word_statistics_only",
        "format_assessment": {
            "device_profile": "AWR2944P real-only LVDS ADC stream",
            "validated_storage_model": "one signed int16 word per RX/chirp/ADC sample",
            "validated_bytes_per_frame": int(radar_cfg.get("estimated_payload_bytes_per_frame", 0) or 0),
            "evidence": "Observed full-frame rate is compared against frameCfg; final lane/RX/TX ordering remains pending.",
        },
        "limitations": [
            "This report does not assert final LVDS lane order or RX/TX word ordering.",
            "Range/Doppler/AoA requires validated AWR2944P LVDS lane formatting and channel reordering.",
        ],
        "file": {"path": str(bin_path), "bytes": size, "trailing_odd_byte": bool(size % INT16_BYTES)},
        "metadata": metadata,
        "radar_cfg": radar_cfg,
        "int16_words": _word_statistics(bin_path),
        "frame_candidates": _frame_candidates(size, radar_cfg, duration_s),
    }


def markdown(report: dict[str, Any]) -> str:
    file_info = report["file"]
    words = report["int16_words"]
    radar = report["radar_cfg"]
    lines = [
        "# 初步原始 ADC 采集分析",
        "",
        "## 结论边界",
        "",
        "当前 AWR2944P 配置的 LVDS ADC 流按 real-only、每个 RX/chirp/ADC sample 一个 int16 字（2 bytes）解释。尚未确认最终 LVDS lane 顺序与 RX/TX 重排，因此不能直接将本报告当作 AoA 或点云结论。",
        "",
        "## 文件与采集概况",
        "",
        f"- 文件：`{file_info['path']}`",
        f"- 文件大小：`{file_info['bytes']:,}` bytes",
        f"- 16-bit 数据字数量：`{words['count']:,}`",
        f"- 文件末尾是否有孤立字节：`{file_info['trailing_odd_byte']}`",
        f"- CFG：RX=`{radar.get('num_rx')}`，ADC samples=`{radar.get('num_adc_samples')}`，chirps/frame=`{radar.get('num_chirps_per_frame')}`，frame period=`{radar.get('frame_period_ms')}` ms。",
        f"- 已验证的 real-only 帧长度：`{report['format_assessment']['validated_bytes_per_frame']:,}` bytes/frame。",
        "",
        "## 原始 16-bit 数据字质量",
        "",
        f"- 最小值 / 最大值：`{words['minimum']}` / `{words['maximum']}`",
        f"- 均值：`{words['mean']:.3f}`，RMS：`{words['rms']:.3f}`，标准差：`{words['std']:.3f}`",
        f"- 接近满量程字数：`{words['saturation_count']:,}`，比例：`{words['saturation_fraction']:.6%}`",
        f"- 偶数序号字均值 / RMS：`{words['even_word']['mean']:.3f}` / `{words['even_word']['rms']:.3f}`",
        f"- 奇数序号字均值 / RMS：`{words['odd_word']['mean']:.3f}` / `{words['odd_word']['rms']:.3f}`",
        "",
        "## 候选帧结构",
        "",
        "下表保留两种字节数计算对照。AWR2944P 当前 real-only 配置应使用第一行；其帧率应与 `frameCfg` 的帧周期接近。",
        "",
        "| 候选解释 | 每帧字节数 | 完整帧数 | 尾部字节 | 观测帧率 |",
        "|---|---:|---:|---:|---:|",
    ]
    for candidate in report["frame_candidates"]:
        rate = candidate.get("observed_frame_rate_hz")
        rate_text = f"{rate:.3f} Hz" if rate is not None else "未提供采集时长"
        lines.append(
            f"| {candidate['interpretation']} | {candidate['bytes_per_frame']:,} | {candidate['full_frames']:,} | {candidate['trailing_bytes']:,} | {rate_text} |"
        )
    lines.extend([
        "",
        "## 下一步",
        "",
        "1. 用采集元数据的完整帧计数与 `frameCfg` 对照，确认输出帧率。",
        "2. 根据 TI AWR2944P LVDS 格式确认 lane、RX 与 TX 的最终排序。",
        "3. 之后才能做每个 RX/TX 的 Range FFT、Range-Doppler 图、AoA 和海杂波统计。",
        "",
    ])
    return "\n".join(lines)


def write_outputs(report: dict[str, Any], output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "adc_first_pass_analysis.json"
    markdown_path = output_dir / "output_analysis.md"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    markdown_path.write_text(markdown(report), encoding="utf-8")
    return json_path, markdown_path


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    bin_path = Path(args.bin_path)
    cfg_path = Path(args.cfg)
    meta_path = Path(args.metadata) if args.metadata else bin_path.with_suffix(".json")
    report = analyze(bin_path, cfg_path, meta_path)
    json_path, markdown_path = write_outputs(report, Path(args.output_dir) if args.output_dir else bin_path.parent)
    print(f"[DONE] JSON analysis: {json_path}")
    print(f"[DONE] Markdown analysis: {markdown_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
