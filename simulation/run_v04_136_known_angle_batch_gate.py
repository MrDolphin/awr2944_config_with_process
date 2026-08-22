"""Run the known-angle batch processor with an explicit real-capture gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from simulation.run_v04_92_known_angle_batch import run as batch_run


def run(input_root: Path, candidate_csv: Path, output: Path) -> dict:
    summary = batch_run(input_root, candidate_csv, output)
    summary["stage"] = "v04.136_known_angle_batch_gate"
    summary["real_capture_required"] = True
    summary["hardware_aoa_validated"] = False
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    report = ["# V0.4.136 五工况已知角批量准入", "", f"发现 manifest：{summary['manifest_count']} 个；成功处理：{summary['processed_capture_count']} 个；失败：{summary['failure_count']} 个。", "", "## 当前结果", "", "本次使用 V0.4.135 的五个 manifest 模板执行批处理。当前 5 个工况均因缺少 `capture.bin` 未进入解码，属于预期的真实采集前 dry-run。", "", "## 通过条件", "", "每个工况必须先补齐真实 capture.bin、manifest 字段、CFG/DCA1000 配置、通道顺序证据和 TI 校准，再重新运行本阶段。只有成功处理数量达到目标覆盖，才可计算坐标变换 RMSE。", "", "## 证据边界", "", "本阶段没有生成硬件 AoA 结论；`hardware_aoa_validated=false`。", ""]
    (output / "output_analysis.md").write_text("\n".join(report), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--candidate-csv", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.candidate_csv.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
