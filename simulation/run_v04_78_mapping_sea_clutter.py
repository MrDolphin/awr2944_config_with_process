"""Apply channel-order candidates to V0.4.72 sea-clutter point spectra."""

from __future__ import annotations

import argparse, csv, json
from collections import defaultdict
from pathlib import Path
import h5py
import numpy as np

from simulation.run_v04_75_coordinate_transform_validation import _load_cfg_geometry
from simulation.run_v04_77_channel_order_sensitivity import permutations
from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_positions


def run(points_csv: Path, input_root: Path, mapping_csv: Path, output: Path) -> dict:
    with points_csv.open(encoding="utf-8", newline="") as handle:
        points = list(csv.DictReader(handle))
    config = FmcwConfig(samples_per_chirp=64, chirps_per_frame=16)
    x, y = _load_cfg_geometry(mapping_csv, config)
    spectra = {}
    for case_id in sorted({row["case_id"] for row in points}):
        with h5py.File(input_root / f"{case_id}_range_doppler.h5", "r") as handle:
            spectra[case_id] = handle["/range_doppler/spectrum_complex"][...]
    values = defaultdict(list)
    for row in points:
        case, scenario, detector = row["case_id"], row["scenario_id"], row["detector"]
        channel = spectra[case][int(row["frame"]), int(row["doppler_index"]), int(row["range_index"])]
        for name, candidate in permutations(channel).items():
            with np.errstate(invalid="ignore", divide="ignore"):
                az, el = estimate_aoa_from_positions(candidate, config, x, y)
            values[(case, scenario, detector, name)].append((float(az), float(el)))
    rows = []
    for (case, scenario, detector, mapping), angles in sorted(values.items()):
        array = np.asarray(angles, dtype=float)
        finite = np.isfinite(array).all(axis=1)
        finite_angles = array[finite]
        rows.append({"case_id": case, "scenario_id": scenario, "detector": detector, "mapping": mapping,
                     "point_count": len(array), "finite_aoa_count": int(finite.sum()), "nonfinite_aoa_count": int((~finite).sum()),
                     "mean_azimuth_deg": float(np.mean(finite_angles[:,0])) if len(finite_angles) else float("nan"),
                     "std_azimuth_deg": float(np.std(finite_angles[:,0])) if len(finite_angles) else float("nan"),
                     "mean_elevation_deg": float(np.mean(finite_angles[:,1])) if len(finite_angles) else float("nan"),
                     "std_elevation_deg": float(np.std(finite_angles[:,1])) if len(finite_angles) else float("nan")})
    output.mkdir(parents=True, exist_ok=True)
    with (output/"mapping_sea_clutter_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer=csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summary={"status":"completed_mapping_sea_clutter_analysis", "input_point_count":len(points), "group_count":len(rows), "mapping_count":len(permutations(np.zeros((4,4),complex))), "hardware_validated":False, "source_status":"V0.4.72 synthetic HDF5 point spectra"}
    (output/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding="utf-8")
    (output/"output_analysis.md").write_text("# V0.4.78 通道映射对海杂波 AoA 点云的影响\n\n"
        "本阶段复用 V0.4.72 已筛选的海杂波复数谱点，对 8 种 TX/RX 通道排列候选重新估计 AoA。检测器和检测单元不变，因此这里比较的是通道映射对角度点云的影响。\n\n"
        "`nonfinite_aoa_count` 表示当前映射下 AoA 反演得到非有限角度的点数；均值和标准差只对有限角度统计。结果仍是合成数据和候选通道映射，不能替代 DCA1000/TI 固件验证。\n",encoding="utf-8")
    return summary


def main():
    p=argparse.ArgumentParser(); p.add_argument("--points-csv",type=Path,required=True); p.add_argument("--input-root",type=Path,required=True); p.add_argument("--mapping-csv",type=Path,required=True); p.add_argument("--output",type=Path,required=True); a=p.parse_args(); print(json.dumps(run(a.points_csv.resolve(),a.input_root.resolve(),a.mapping_csv.resolve(),a.output.resolve()),indent=2,ensure_ascii=False))


if __name__ == "__main__": main()
