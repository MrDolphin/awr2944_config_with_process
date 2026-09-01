#!/usr/bin/env python3
"""Summarize one DCA1000 capture without overstating calibration status.

The tool combines capture metadata with the already-generated range-domain
report.  It deliberately records calibration, Range-Doppler and AoA gates as
not run or blocked until their required physical evidence is supplied.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

try:
    from tools.analyze_adc_capture import analyze as analyze_first_pass
except ModuleNotFoundError:  # Direct ``python tools/post_capture_analysis.py``.
    from analyze_adc_capture import analyze as analyze_first_pass


PASS = "PASS"
FAIL = "FAIL"
NOT_RUN = "NOT_RUN"
BLOCKED = "BLOCKED"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bin", required=True, dest="bin_path")
    parser.add_argument("--cfg", required=True)
    parser.add_argument("--metadata", help="Companion DCA1000 capture JSON; defaults to .bin sibling.")
    parser.add_argument("--range-analysis-dir", help="Defaults to capture-directory/range_analysis.")
    parser.add_argument("--output-dir", help="Defaults to the capture directory.")
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, dict) else {}


def gate(status: str, summary: str, **evidence: Any) -> dict[str, Any]:
    return {"status": status, "summary": summary, "evidence": evidence}


def capture_integrity_gate(metadata: dict[str, Any], bin_size: int) -> dict[str, Any]:
    packet_count = metadata.get("packet_count")
    dropped = metadata.get("dropped_packets_estimate")
    if bin_size <= 0:
        return gate(FAIL, "采集 BIN 文件为空。", saved_bytes=bin_size, packet_count=packet_count, dropped_packets_estimate=dropped)
    if not isinstance(packet_count, int) or packet_count <= 0:
        return gate(FAIL, "采集元数据没有有效 UDP 包计数。", saved_bytes=bin_size, packet_count=packet_count, dropped_packets_estimate=dropped)
    if not isinstance(dropped, int):
        return gate(NOT_RUN, "未提供 dropped_packets_estimate，不能判定 UDP 完整性。", saved_bytes=bin_size, packet_count=packet_count)
    if dropped:
        return gate(FAIL, "采集记录到 UDP 丢包估计，不能作为无丢包标定数据。", saved_bytes=bin_size, packet_count=packet_count, dropped_packets_estimate=dropped)
    return gate(PASS, "非空 BIN、有效 UDP 包计数且 dropped_packets_estimate=0。", saved_bytes=bin_size, packet_count=packet_count, dropped_packets_estimate=dropped)


def range_gate(range_report: dict[str, Any]) -> dict[str, Any]:
    if not range_report:
        return gate(NOT_RUN, "没有找到 range_fft_analysis.json；未运行距离域分析。")
    frames = range_report.get("full_frames")
    strongest = range_report.get("strongest_mean_range_peak")
    if not isinstance(frames, int) or frames < 1 or not isinstance(strongest, dict):
        return gate(FAIL, "距离域分析文件缺少完整帧数或最强峰。", full_frames=frames, strongest_mean_range_peak=strongest)
    return gate(
        PASS,
        "已生成 real-only 距离 FFT；峰值仅是相对距离域证据，不代表绝对测距标定。",
        full_frames=frames,
        trailing_bytes_ignored=range_report.get("trailing_bytes_ignored"),
        strongest_mean_range_peak=strongest,
        candidate_static_peaks=range_report.get("candidate_static_peaks", []),
    )


def analyze(bin_path: Path, cfg_path: Path, metadata_path: Path, range_analysis_dir: Path) -> dict[str, Any]:
    first_pass = analyze_first_pass(bin_path, cfg_path, metadata_path)
    metadata = first_pass["metadata"]
    range_report = load_json(range_analysis_dir / "range_fft_analysis.json")
    bin_size = int(first_pass["file"]["bytes"])
    range_domain = range_gate(range_report)
    return {
        "analysis_scope": "post_capture_quality_and_calibration_gate_summary",
        "limitations": [
            "PASS only confirms the evidence named in each gate; it does not prove any unlisted hardware property.",
            "Absolute range, RX/TX phase, Range-Doppler and AoA require distinct physical validation evidence.",
        ],
        "file": first_pass["file"],
        "metadata": metadata,
        "radar_cfg": first_pass["radar_cfg"],
        "range_analysis_dir": str(range_analysis_dir),
        "gates": {
            "capture_integrity": capture_integrity_gate(metadata, bin_size),
            "frame_structure": gate(
                PASS if range_domain["status"] == PASS else NOT_RUN,
                "距离分析已按当前 real-only CFG 重组成完整帧；仍应结合实际采集场景复核。"
                if range_domain["status"] == PASS
                else "未取得可用距离分析结果，不能确认帧结构。",
                validated_bytes_per_frame=first_pass["format_assessment"]["validated_bytes_per_frame"],
                range_full_frames=range_report.get("full_frames"),
                trailing_bytes_ignored=range_report.get("trailing_bytes_ignored"),
            ),
            "range_domain": range_domain,
            "absolute_range_calibration": gate(
                NOT_RUN,
                "未提供已知距离目标、实测距离或 range bias 结果，不能给出绝对距离标定结论。",
                required_evidence=["固定强反射器", "已测量目标距离", "重复采集的距离峰偏差"],
            ),
            "rx_tx_phase_calibration": gate(
                NOT_RUN,
                "未提供 TI measureRangeBiasAndRxChanPhase/compRangeBiasAndRxChanPhase 的实测结果。",
                required_evidence=["校准夹具", "TI 相位测量输出", "回填并复测结果"],
            ),
            "range_doppler": gate(
                BLOCKED,
                "尚未验证 TDM 慢时间排序和实测数据的多 TX 重排，故不生成可解释的 Range-Doppler 结论。",
                prerequisites=["已验证帧内 chirp/TX 顺序", "真实采集 Range-Doppler 对照目标"],
            ),
            "aoa_point_cloud_accuracy": gate(
                BLOCKED,
                "RX/TX 相位校准与虚拟阵列几何尚未验收，不能输出可信 AoA/点云精度。",
                prerequisites=["RX/TX 相位校准", "虚拟阵列映射", "已知角度目标验证"],
            ),
        },
    }


def markdown(report: dict[str, Any]) -> str:
    labels = {
        "capture_integrity": "采集完整性门",
        "frame_structure": "帧结构门",
        "range_domain": "距离域门",
        "absolute_range_calibration": "绝对距离标定门",
        "rx_tx_phase_calibration": "RX/TX 通道相位校准门",
        "range_doppler": "Range-Doppler 门",
        "aoa_point_cloud_accuracy": "AoA / 点云精度门",
    }
    status_names = {PASS: "通过", FAIL: "失败", NOT_RUN: "未执行", BLOCKED: "被阻塞"}
    lines = [
        "# AWR2944P 采集后综合分析",
        "",
        "## 结果边界",
        "",
        "本报告将已经获得的采集与距离域证据，同尚未完成的物理标定严格分开。`通过`不等于整套雷达已完成绝对距离、相位、速度或点云精度标定。",
        "",
        "## 文件",
        "",
        f"- ADC BIN：`{report['file']['path']}`",
        f"- 文件大小：`{report['file']['bytes']:,}` bytes",
        f"- 距离域分析目录：`{report['range_analysis_dir']}`",
        "",
        "## 门状态",
        "",
        "| 门 | 状态 | 结论 |",
        "|---|---|---|",
    ]
    for key, item in report["gates"].items():
        lines.append(f"| {labels[key]} | {status_names[item['status']]} (`{item['status']}`) | {item['summary']} |")
    lines.extend(["", "## 可追溯证据", ""])
    for key, item in report["gates"].items():
        lines.extend([f"### {labels[key]}", "", f"- 状态：`{item['status']}`", f"- 结论：{item['summary']}"])
        for evidence_key, value in item["evidence"].items():
            lines.append(f"- `{evidence_key}`：`{json.dumps(value, ensure_ascii=False)}`")
        lines.append("")
    return "\n".join(lines)


def write_outputs(report: dict[str, Any], output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "post_capture_analysis.json"
    markdown_path = output_dir / "output_analysis.md"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    markdown_path.write_text(markdown(report), encoding="utf-8")
    return json_path, markdown_path


def print_dynamic_summary(report: dict[str, Any], markdown_path: Path) -> None:
    for name, item in report["gates"].items():
        print(f"[POST-ANALYSIS] {name}={item['status']} {item['summary']}")
    print(f"[POST-ANALYSIS] report={markdown_path}")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    bin_path = Path(args.bin_path)
    cfg_path = Path(args.cfg)
    metadata_path = Path(args.metadata) if args.metadata else bin_path.with_suffix(".json")
    range_dir = Path(args.range_analysis_dir) if args.range_analysis_dir else bin_path.parent / "range_analysis"
    output_dir = Path(args.output_dir) if args.output_dir else bin_path.parent
    report = analyze(bin_path, cfg_path, metadata_path, range_dir)
    _, markdown_path = write_outputs(report, output_dir)
    print_dynamic_summary(report, markdown_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
