"""Locate high-energy cells and explain their CA-CFAR decision evidence."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_45_cfar_sweep import SCENARIOS


def _read_power(path: Path):
    with h5py.File(path, "r") as handle:
        return (handle["/range_doppler/power_linear"][...], handle["/axes/range_m"][...], handle["/axes/velocity_mps"][...])


def _cell_evidence(power: np.ndarray, d: int, r: int, scenario: dict) -> dict:
    tr_d, tr_r = scenario["training"]; gd, gr = scenario["guard"]
    rows, cols = power.shape
    valid = tr_d + gd <= d < rows - tr_d - gd and tr_r + gr <= r < cols - tr_r - gr
    if not valid:
        return {"valid_for_cfar": False, "noise_linear": None, "threshold_linear": None, "power_to_threshold_db": None, "local_max": None, "above_threshold": False, "local_maximum": False, "would_detect": False}
    outer = power[d - tr_d - gd:d + tr_d + gd + 1, r - tr_r - gr:r + tr_r + gr + 1]
    mask = np.ones_like(outer, dtype=bool)
    mask[tr_d:tr_d + 2 * gd + 1, tr_r:tr_r + 2 * gr + 1] = False
    noise = float(np.mean(outer[mask]))
    count = int(mask.sum())
    alpha = count * (scenario["pfa"] ** (-1.0 / count) - 1.0)
    threshold = alpha * noise
    neighborhood = power[max(0, d - 1):d + 2, max(0, r - 1):r + 2]
    local_max = float(np.max(neighborhood))
    cell_power = float(power[d, r])
    above = cell_power > threshold
    local = cell_power >= local_max
    return {"valid_for_cfar": True, "noise_linear": noise, "threshold_linear": threshold, "power_to_threshold_db": float(10 * np.log10(max(cell_power, 1e-30) / max(threshold, 1e-30))), "local_max": local_max, "above_threshold": above, "local_maximum": local, "would_detect": bool(above and local)}


def run(input_root: Path, output: Path, top_n: int = 5) -> dict:
    rows: list[dict] = []
    case_summaries: list[dict] = []
    for path in sorted(input_root.glob("*_range_doppler.h5")):
        case_id = path.stem.replace("_range_doppler", "")
        power, ranges, velocities = _read_power(path)
        flat_indices = np.argpartition(power.reshape(-1), -top_n)[-top_n:]
        flat_indices = flat_indices[np.argsort(power.reshape(-1)[flat_indices])[::-1]]
        top_cells = [np.unravel_index(int(index), power.shape) for index in flat_indices]
        case_summaries.append({"case_id": case_id, "top_cell_count": len(top_cells), "top_power_linear": float(power[top_cells[0]])})
        for rank, (frame, d, r) in enumerate(((int(idx[0]), int(idx[1]), int(idx[2])) for idx in top_cells), start=1):
            for scenario in SCENARIOS:
                evidence = _cell_evidence(power[frame], d, r, scenario)
                rows.append({"case_id": case_id, "rank": rank, "frame": frame, "doppler_index": d, "range_index": r, "range_m": float(ranges[r]), "velocity_mps": float(velocities[d]), "cell_power_linear": float(power[frame, d, r]), "scenario_id": scenario["scenario_id"], "pfa": scenario["pfa"], "training_doppler": scenario["training"][0], "training_range": scenario["training"][1], "guard_doppler": scenario["guard"][0], "guard_range": scenario["guard"][1], **evidence})
    if not rows:
        raise ValueError("no range_doppler inputs found")
    output.mkdir(parents=True, exist_ok=True)
    with (output / "cfar_cell_evidence.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    with (output / "top_energy_cells.csv").open("w", encoding="utf-8", newline="") as handle:
        top_rows = [row for row in rows if row["scenario_id"] == SCENARIOS[0]["scenario_id"]]
        writer = csv.DictWriter(handle, fieldnames=["case_id", "rank", "frame", "doppler_index", "range_index", "range_m", "velocity_mps", "cell_power_linear"]); writer.writeheader(); writer.writerows([{key: row[key] for key in writer.fieldnames} for row in top_rows])
    detections = [row for row in rows if row["would_detect"]]
    summary = {"status": "completed_cfar_cell_evidence", "case_count": len(case_summaries), "top_n_per_case": top_n, "scenario_count": len(SCENARIOS), "top_cells_meeting_any_cfar": len({(row["case_id"], row["rank"]) for row in detections}), "all_case_summaries": case_summaries, "hardware_validated": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.69 高能量单元与 CFAR 决策证据", "", f"每个海况取未 CFAR 功率最高的 {top_n} 个单元，共 {len(case_summaries)} 个海况、{len(SCENARIOS)} 组 CFAR 参数。", "", "## 字段解释", "", "`noise_linear` 是训练窗噪声估计，`threshold_linear` 是 CA-CFAR 阈值，`power_to_threshold_db` 是单元功率相对阈值的 dB 比值，`above_threshold` 和 `local_maximum` 分别表示两个保留条件，`would_detect` 表示该单元在该场景下是否会被保留。", "", "## 证据边界", "", "高能量单元若位于 CFAR 不可评估边界，`valid_for_cfar=false`，不能据此判断算法漏检；输入仍是合成距离-多普勒谱，不是实测 IQ。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--top-n", type=int, default=5); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve(), args.top_n), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
