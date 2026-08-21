"""Run the V0.4.24-V0.4.26 sea-state reporting loop in one command."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from simulation.run_v04_point_cloud_stats import run as run_stats
from simulation.run_v04_sea_state_interpretation import run as run_interpretation
from simulation.run_v04_sea_state_visual_report import run as run_visual_report


def run(inputs: list[tuple[str, Path]], output: Path, threshold: float = 0.2) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    stats_dir = output / "stats"
    visual_dir = output / "visual_report"
    interpretation_dir = output / "interpretation"
    summaries = run_stats(inputs, stats_dir)
    visual_report = run_visual_report(inputs, visual_dir)
    interpretation_report = run_interpretation(stats_dir / "point_cloud_stats.csv", interpretation_dir, threshold)
    bundle = {
        "status": "completed_descriptive_sea_state_bundle",
        "run_count": len(inputs),
        "labels": [label for label, _ in inputs],
        "stats_csv": str((stats_dir / "point_cloud_stats.csv").resolve()),
        "visual_report": str(visual_report.resolve()),
        "interpretation_report": str(interpretation_report.resolve()),
        "descriptive_only": True,
        "summaries": summaries,
    }
    (output / "bundle_summary.json").write_text(json.dumps(bundle, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "README.md").write_text(
        "# V0.4.27 海况报告包\n\n"
        "本目录由一个命令生成统计、图像和描述性判读。它是仿真分析报告包，不是海况等级分类器。\n\n"
        f"- [统计 CSV](stats/point_cloud_stats.csv)\n- [图文报告](visual_report/sea_state_comparison.md)\n"
        f"- [自动判读](interpretation/sea_state_interpretation.md)\n- [汇总 JSON](bundle_summary.json)\n",
        encoding="utf-8")
    return bundle


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", action="append", nargs=2, metavar=("LABEL", "H5"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--relative-threshold", type=float, default=0.2)
    args = parser.parse_args()
    bundle = run([(label, Path(path).resolve()) for label, path in args.input], args.output.resolve(), args.relative_threshold)
    print(json.dumps(bundle, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
