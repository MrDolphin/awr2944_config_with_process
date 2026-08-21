"""Batch known-angle captures and aggregate coordinate-transform evidence."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_91_known_angle_capture import process


def run(input_root: Path, candidate_csv: Path, output: Path) -> dict:
    manifests = sorted(input_root.rglob("manifest.json")) if input_root.is_dir() else []
    output.mkdir(parents=True, exist_ok=True)
    capture_rows: list[dict] = []
    failures: list[dict] = []
    for manifest in manifests:
        capture_id = manifest.parent.name
        capture_output = output / capture_id
        try:
            summary = process(manifest, candidate_csv, capture_output)
            for row in summary["results"]:
                capture_rows.append({"capture_id": capture_id, "manifest": str(manifest.resolve()), **row, "manifest_readiness": summary["manifest_readiness"]["status"]})
        except Exception as exc:
            failures.append({"capture_id": capture_id, "manifest": str(manifest.resolve()), "error": f"{type(exc).__name__}: {exc}"})
    aggregate_rows = []
    for transform in ("identity", "mirror_x", "mirror_y", "rotate_180"):
        subset = [row for row in capture_rows if row["transform"] == transform]
        errors = np.asarray([row["combined_error_deg"] for row in subset], dtype=float)
        aggregate_rows.append({"transform": transform, "capture_count": len(subset), "combined_rmse_deg": float(np.sqrt(np.mean(errors ** 2))) if len(errors) else None, "mean_combined_error_deg": float(np.mean(errors)) if len(errors) else None, "max_combined_error_deg": float(np.max(errors)) if len(errors) else None})
    if capture_rows:
        with (output / "capture_transform_results.csv").open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(capture_rows[0])); writer.writeheader(); writer.writerows(capture_rows)
    with (output / "transform_aggregate.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(aggregate_rows[0])); writer.writeheader(); writer.writerows(aggregate_rows)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axis = plt.subplots(figsize=(8, 4), constrained_layout=True)
    labels = [row["transform"] for row in aggregate_rows]
    values = [row["combined_rmse_deg"] for row in aggregate_rows]
    if any(value is not None for value in values):
        axis.bar(labels, [value if value is not None else 0.0 for value in values], color="#4472c4")
        axis.set_ylabel("combined RMSE (deg)")
        axis.set_title("Known-angle coordinate-transform batch")
    else:
        axis.text(0.5, 0.5, "No real capture manifests", ha="center", va="center", transform=axis.transAxes)
        axis.set_title("Known-angle coordinate-transform batch")
        axis.set_xticks([]); axis.set_yticks([])
    figure.savefig(output / "transform_aggregate.png", dpi=160)
    plt.close(figure)
    best = min((row for row in aggregate_rows if row["combined_rmse_deg"] is not None), key=lambda row: row["combined_rmse_deg"], default=None)
    summary = {"status": "completed_known_angle_batch" if capture_rows else "awaiting_known_angle_manifests", "input_root": str(input_root.resolve()), "candidate_csv": str(candidate_csv.resolve()), "manifest_count": len(manifests), "processed_capture_count": len({row["capture_id"] for row in capture_rows}), "failure_count": len(failures), "failures": failures, "aggregate": aggregate_rows, "best_transform_by_batch_rmse": best["transform"] if best else None, "hardware_aoa_validated": False, "evidence_status": "real_capture_candidate_only_until_channel_order_calibration_and_phase_center_are_verified"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.92 多角度已知角采集批处理", "", f"发现 manifest：{len(manifests)} 个；成功处理采集：{summary['processed_capture_count']} 个；失败：{len(failures)} 个。", "", "## 坐标候选汇总", "", "| 候选 | 采集数 | 综合 RMSE(°) | 平均误差(°) | 最大误差(°) |", "|---|---:|---:|---:|---:|"]
    lines.extend(f"| {row['transform']} | {row['capture_count']} | {row['combined_rmse_deg'] if row['combined_rmse_deg'] is not None else '—'} | {row['mean_combined_error_deg'] if row['mean_combined_error_deg'] is not None else '—'} | {row['max_combined_error_deg'] if row['max_combined_error_deg'] is not None else '—'} |" for row in aggregate_rows)
    lines += ["", f"批处理 RMSE 最小候选：`{summary['best_transform_by_batch_rmse'] or '暂无'}`。", "", "## 解释和边界", "", "批处理只汇总每个已知角采集的候选误差。只有目标真值覆盖多个方位/俯仰、距离和姿态，并且 DCA1000 wire order、通道顺序、TI 校准和机械基准均有证据时，才可用它冻结坐标变换。当前 `hardware_aoa_validated` 保持为 false。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--candidate-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.input_root.resolve(), args.candidate_csv.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
