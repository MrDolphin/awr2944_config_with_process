"""Evaluate high-energy cells only inside each CFAR scenario's valid region."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_45_cfar_sweep import SCENARIOS
from simulation.run_v04_69_cfar_cell_evidence import _cell_evidence


def _read_power(path: Path):
    with h5py.File(path, "r") as handle:
        return handle["/range_doppler/power_linear"][...], handle["/axes/range_m"][...], handle["/axes/velocity_mps"][...]


def run(input_root: Path, output: Path, top_n: int = 20) -> dict:
    rows: list[dict] = []
    for path in sorted(input_root.glob("*_range_doppler.h5")):
        case_id = path.stem.replace("_range_doppler", "")
        power, ranges, velocities = _read_power(path)
        for scenario in SCENARIOS:
            tr_d, tr_r = scenario["training"]; gd, gr = scenario["guard"]
            d_min, d_max = tr_d + gd, power.shape[1] - tr_d - gd
            r_min, r_max = tr_r + gr, power.shape[2] - tr_r - gr
            candidate_indices = []
            for frame in range(power.shape[0]):
                valid_plane = power[frame, d_min:d_max, r_min:r_max]
                count = min(top_n, valid_plane.size)
                flat = np.argpartition(valid_plane.reshape(-1), -count)[-count:]
                flat = flat[np.argsort(valid_plane.reshape(-1)[flat])[::-1]]
                for rank, index in enumerate(flat, start=1):
                    local_d, local_r = np.unravel_index(int(index), valid_plane.shape)
                    candidate_indices.append((float(valid_plane[local_d, local_r]), frame, local_d + d_min, local_r + r_min, rank))
            candidate_indices.sort(reverse=True)
            selected = candidate_indices[:top_n]
            for rank, (cell_power, frame, d, r, _) in enumerate(selected, start=1):
                evidence = _cell_evidence(power[frame], d, r, scenario)
                rows.append({"case_id": case_id, "scenario_id": scenario["scenario_id"], "rank_valid": rank, "frame": frame, "doppler_index": d, "range_index": r, "range_m": float(ranges[r]), "velocity_mps": float(velocities[d]), "cell_power_linear": cell_power, **evidence})
    if not rows:
        raise ValueError("no valid CFAR candidates found")
    summary_rows: list[dict] = []
    for case_id in sorted({row["case_id"] for row in rows}):
        for scenario_id in sorted({row["scenario_id"] for row in rows if row["case_id"] == case_id}):
            subset = [row for row in rows if row["case_id"] == case_id and row["scenario_id"] == scenario_id]
            summary_rows.append({"case_id": case_id, "scenario_id": scenario_id, "candidate_count": len(subset), "above_threshold_count": sum(bool(row["above_threshold"]) for row in subset), "local_maximum_count": sum(bool(row["local_maximum"]) for row in subset), "would_detect_count": sum(bool(row["would_detect"]) for row in subset), "max_power_to_threshold_db": max(float(row["power_to_threshold_db"]) for row in subset if row["power_to_threshold_db"] is not None)})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "valid_region_cfar_evidence.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    with (output / "valid_region_cfar_matrix.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary_rows[0])); writer.writeheader(); writer.writerows(summary_rows)
    summary = {"status": "completed_valid_region_cfar_evidence", "case_count": len({row["case_id"] for row in rows}), "scenario_count": len(SCENARIOS), "top_n_valid_per_case_scenario": top_n, "matrix_rows": len(summary_rows), "hardware_validated": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.70 CFAR 有效区域证据", "", f"每个海况/CFAR 场景在可评估区域内取最高 {top_n} 个功率单元。", "", "## 统计字段", "", "`above_threshold_count` 只反映功率门限条件；`local_maximum_count` 只反映邻域峰值条件；`would_detect_count` 同时满足两者。若有效区域内仍然 `would_detect_count=0`，才有理由继续检查阈值或峰值规则，而不是把边界效应当作漏检。", "", "## 证据边界", "", "输入仍是合成距离-多普勒谱；本阶段区分了 CFAR 边界和有效区域，但不代表 TI SDK 或实板 CFAR 实现。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--top-n", type=int, default=20); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve(), args.top_n), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
