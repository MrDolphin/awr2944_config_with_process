"""Produce descriptive, non-classifying interpretation of point-cloud stats."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


NUMERIC = ("point_count", "velocity_std_mps", "azimuth_std_deg", "elevation_std_deg")


def read_stats(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        for key in NUMERIC:
            row[key] = float(row[key])
    return rows


def interpret(rows: list[dict], threshold: float = 0.2) -> list[dict]:
    if not rows:
        return []
    baseline = rows[0]
    output = []
    for row in rows:
        item = dict(row); item["baseline_label"] = baseline["label"]
        for key in NUMERIC:
            base = baseline[key]; value = row[key]
            item[f"{key}_delta"] = value - base
            item[f"{key}_relative_change"] = None if base == 0 else (value - base) / base
        flags = []
        if row["point_count"] > baseline["point_count"] * (1 + threshold): flags.append("more_cfar_detections_than_baseline")
        if row["velocity_std_mps"] > baseline["velocity_std_mps"] * (1 + threshold) and baseline["velocity_std_mps"] > 0: flags.append("wider_velocity_spread_than_baseline")
        if row["azimuth_std_deg"] > baseline["azimuth_std_deg"] * (1 + threshold) and baseline["azimuth_std_deg"] > 0: flags.append("wider_azimuth_spread_than_baseline")
        if row["elevation_std_deg"] > baseline["elevation_std_deg"] * (1 + threshold) and baseline["elevation_std_deg"] > 0: flags.append("wider_elevation_spread_than_baseline")
        item["descriptive_flags"] = flags
        output.append(item)
    return output


def run(stats_csv: Path, output: Path, threshold: float = 0.2) -> Path:
    interpreted = interpret(read_stats(stats_csv), threshold)
    output.mkdir(parents=True, exist_ok=True)
    (output / "interpretation.json").write_text(json.dumps({"runs": interpreted, "status": "descriptive_not_sea_state_classifier"}, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.26 海况点云自动判读", "", f"基准：{interpreted[0]['label'] if interpreted else '-'}；相对变化阈值：{threshold:.0%}。", "", "| 标签 | 点数变化 | 速度展宽变化 | 方位展宽变化 | 俯仰展宽变化 | 描述性标记 |", "|---|---:|---:|---:|---:|---|"]
    for row in interpreted:
        flags = ", ".join(row["descriptive_flags"]) or "-"
        lines.append(f"| {row['label']} | {row['point_count_delta']:+.2f} | {row['velocity_std_mps_delta']:+.4f} | {row['azimuth_std_deg_delta']:+.4f} | {row['elevation_std_deg_delta']:+.4f} | {flags} |")
    lines += ["", "## 解释边界", "", "这些标记只描述当前 CFAR/AoA 仿真输出相对基准的变化，不是海况等级分类器，也不是实测检测概率或海况测量。", "需要结合风速、有效波高、波向、船速和姿态实测才能形成海况结论。", ""]
    report = output / "sea_state_interpretation.md"; report.write_text("\n".join(lines), encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stats-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--relative-threshold", type=float, default=0.2)
    args = parser.parse_args()
    print(run(args.stats_csv.resolve(), args.output.resolve(), args.relative_threshold))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
