"""Compare CA-CFAR, exploratory OS-CFAR and fixed energy gates."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_45_cfar_sweep import SCENARIOS
from simulation.run_v04_69_cfar_cell_evidence import _cell_evidence


def _load(path: Path):
    with h5py.File(path, "r") as handle:
        return handle["/range_doppler/power_linear"][...]


def _valid_coords(plane: np.ndarray, scenario: dict):
    tr_d, tr_r = scenario["training"]; gd, gr = scenario["guard"]
    for d in range(tr_d + gd, plane.shape[0] - tr_d - gd):
        for r in range(tr_r + gr, plane.shape[1] - tr_r - gr):
            yield d, r


def _os_threshold(plane: np.ndarray, d: int, r: int, scenario: dict) -> float:
    tr_d, tr_r = scenario["training"]; gd, gr = scenario["guard"]
    outer = plane[d - tr_d - gd:d + tr_d + gd + 1, r - tr_r - gr:r + tr_r + gr + 1]
    mask = np.ones_like(outer, dtype=bool); mask[tr_d:tr_d + 2 * gd + 1, tr_r:tr_r + 2 * gr + 1] = False
    ordered = np.sort(outer[mask].reshape(-1))
    rank = min(len(ordered) - 1, int(0.75 * (len(ordered) - 1)))
    alpha = len(ordered) * (scenario["pfa"] ** (-1.0 / len(ordered)) - 1.0)
    return float(alpha * ordered[rank])


def _local_max(plane: np.ndarray, d: int, r: int) -> bool:
    cell = float(plane[d, r]); neighborhood = plane[max(0, d - 1):d + 2, max(0, r - 1):r + 2]
    return cell >= float(np.max(neighborhood))


def run(input_root: Path, output: Path, fixed_percentile: float = 99.0) -> dict:
    rows: list[dict] = []
    for path in sorted(input_root.glob("*_range_doppler.h5")):
        case_id = path.stem.replace("_range_doppler", "")
        power = _load(path)
        fixed_threshold = float(np.percentile(power.reshape(-1), fixed_percentile))
        for scenario in SCENARIOS:
            counts = {"ca_cfar_local_peak": 0, "ca_threshold_only": 0, "os_cfar_local_peak": 0, "fixed_energy_local_peak": 0, "valid_cells": 0}
            for frame in range(power.shape[0]):
                plane = power[frame]
                for d, r in _valid_coords(plane, scenario):
                    counts["valid_cells"] += 1
                    cell = float(plane[d, r]); local = _local_max(plane, d, r)
                    ca = _cell_evidence(plane, d, r, scenario)
                    counts["ca_cfar_local_peak"] += int(ca["would_detect"])
                    counts["ca_threshold_only"] += int(ca["above_threshold"])
                    os_threshold = _os_threshold(plane, d, r, scenario)
                    counts["os_cfar_local_peak"] += int(cell > os_threshold and local)
                    counts["fixed_energy_local_peak"] += int(cell > fixed_threshold and local)
            for detector, count in counts.items():
                if detector == "valid_cells":
                    continue
                rows.append({"case_id": case_id, "scenario_id": scenario["scenario_id"], "detector": detector, "pfa": scenario["pfa"], "training_doppler": scenario["training"][0], "training_range": scenario["training"][1], "guard_doppler": scenario["guard"][0], "guard_range": scenario["guard"][1], "valid_cells": counts["valid_cells"], "detection_count": count, "detections_per_frame": count / max(power.shape[0], 1), "detection_rate_valid_cells": count / max(counts["valid_cells"], 1), "fixed_percentile": fixed_percentile, "fixed_threshold_linear": fixed_threshold})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "detector_comparison.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summary = {"status": "completed_detector_comparison", "case_count": len({row["case_id"] for row in rows}), "scenario_count": len(SCENARIOS), "detectors": sorted({row["detector"] for row in rows}), "fixed_energy_percentile": fixed_percentile, "os_cfar_status": "exploratory_order_statistic_not_ti_sdk_equivalent", "hardware_validated": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.71 CFAR/能量门检测器对照", "", "本阶段在相同五海况功率谱上比较 CA-CFAR（含局部峰值）、CA 阈值门（关闭局部峰值）、探索性 OS-CFAR 和固定 P99 能量门+局部峰值。", "", "## 解释", "", "`detection_count` 是当前合成距离-多普勒网格上的保留单元数，不等于真实目标数或虚警率。`ca_threshold_only` 用来量化局部峰值规则的影响；`os_cfar_local_peak` 的排序统计量和缩放仅为工程对照，不声称等价于 TI SDK。", "", "## 证据边界", "", "输入不是 DCA1000 实测 IQ，参数也不是 TI 固件默认值。最终应以真实 CFG、固件输出和实测数据重新验证。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--fixed-percentile", type=float, default=99.0); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve(), args.fixed_percentile), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
