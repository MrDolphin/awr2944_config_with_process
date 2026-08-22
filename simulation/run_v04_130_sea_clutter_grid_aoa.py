"""Apply grid-beam AoA to active V0.120 sea facets and reweight clutter."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_124_pattern_weighted_clutter import _pattern
from simulation.run_v04_128_array_model_comparison import models
from simulation.run_v04_129_grid_beam_aoa import estimate_grid
from simulation.v03 import FmcwConfig
from simulation.v04 import generate_aoa_iq_with_positions


def run(physics: Path, mapping: Path, copper: Path, endpoint: Path, az_pattern: Path, el_pattern: Path, output: Path) -> dict:
    config = FmcwConfig(); model_map = models(mapping, copper, endpoint, config)
    az_axis, az_gain = _pattern(az_pattern, "azimuth_deg"); el_axis, el_gain = _pattern(el_pattern, "elevation_deg")
    with physics.open(encoding="utf-8", newline="") as handle:
        source_rows = list(csv.DictReader(handle))
    az_grid = np.arange(-80.0, 80.1, 4.0); el_grid = np.arange(-40.0, 40.1, 4.0)
    active = [row for row in source_rows if float(row["masked_scatter_6db"]) > 0.0]
    cache: dict[tuple[str, float, float], tuple[float, float, float]] = {}
    rows = []
    for model_name, positions in model_map.items():
        for row in active:
            truth_az = float(row["azimuth_deg"]); truth_el = float(row["elevation_down_deg"]); key = (model_name, round(truth_az, 6), round(truth_el, 6))
            if key not in cache:
                iq = generate_aoa_iq_with_positions(config, slant_range_m=float(row["slant_range_m"]), radial_velocity_mps=float(row["radial_velocity_mps"]), azimuth_deg=truth_az, elevation_deg=truth_el, x_positions_m=positions[0], y_positions_m=positions[1])
                cache[key] = estimate_grid(np.mean(iq, axis=(0, 1)), config, positions, az_grid, el_grid)
            est_az, est_el, peak = cache[key]
            gain_truth = float(np.interp(abs(truth_az), az_axis, az_gain) + np.interp(abs(truth_el), el_axis, el_gain))
            gain_est = float(np.interp(abs(est_az), az_axis, az_gain) + np.interp(abs(est_el), el_axis, el_gain))
            one_way = 10.0 ** (gain_est / 10.0); two_way = one_way ** 2
            rows.append({"case_id": row["case_id"], "frame": row["frame"], "facet_index": row["facet_index"], "model": model_name, "truth_azimuth_deg": truth_az, "truth_elevation_down_deg": truth_el, "estimated_azimuth_deg": est_az, "estimated_elevation_down_deg": est_el, "azimuth_error_deg": est_az - truth_az, "elevation_error_deg": est_el - truth_el, "normalized_beam_peak": peak, "truth_pattern_gain_dBi": gain_truth, "estimated_pattern_gain_dBi": gain_est, "pattern_one_way_weight": one_way, "pattern_two_way_weight": two_way, "scatter_6db": float(row["masked_scatter_6db"]), "aoa_weighted_scatter_one_way": float(row["masked_scatter_6db"]) * one_way, "aoa_weighted_scatter_two_way": float(row["masked_scatter_6db"]) * two_way})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "sea_clutter_grid_aoa.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summaries = []
    for model_name in model_map:
        part = [row for row in rows if row["model"] == model_name]; azerr = np.asarray([row["azimuth_error_deg"] for row in part]); elerr = np.asarray([row["elevation_error_deg"] for row in part])
        summaries.append({"model": model_name, "active_rows": len(part), "unique_angle_cache_entries": sum(1 for key in cache if key[0] == model_name), "azimuth_rmse_deg": float(np.sqrt(np.mean(azerr ** 2))), "elevation_rmse_deg": float(np.sqrt(np.mean(elerr ** 2))), "one_way_scatter_sum": float(sum(row["aoa_weighted_scatter_one_way"] for row in part)), "two_way_scatter_sum": float(sum(row["aoa_weighted_scatter_two_way"] for row in part)), "mean_estimated_pattern_gain_dBi": float(np.mean([row["estimated_pattern_gain_dBi"] for row in part]))})
    with (output / "model_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    result = {"status": "completed_sea_clutter_grid_aoa", "source_rows": len(source_rows), "active_rows": len(active), "model_count": len(model_map), "cache_entries": len(cache), "grid_step_deg": 4.0, "hardware_aoa_validated": False, "phase_center_ready": False, "summaries": summaries}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.130 海杂波微面元网格 AoA", "", f"本阶段处理 V0.120 的 {len(source_rows)} 行海面微面元，其中 {len(active)} 行具有 6 dB 波束内散射代理；对四种阵列模型逐点生成合成 IQ 并进行 4° 网格波束扫描。", "", "## 输出含义", "", "`sea_clutter_grid_aoa.csv` 保存每个有效微面元在四种模型下的真值角度、估计角度、角度误差、方向图估计增益、单程/双程方向图权重和加权海杂波代理。没有有效散射的微面元未重复生成 AoA，因为其散射幅度为零。", "", "## 结论边界", "", "同一模型下角度误差用于检查网格 AoA 接口是否稳定；不同模型之间的误差反映阵列几何差异。方向图仍是 TI EVM 曲线的粗粒度数字化，海杂波是 V0.120 合成代理，不能解释为实测功率、dBsm、检测概率或探测距离。", "", "下一步应使用真实 DCA1000 ADC IQ 和已知角度目标校验通道顺序、校准矩阵与阵元坐标，再将实测角度替换当前合成角度。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--physics", type=Path, required=True); parser.add_argument("--mapping", type=Path, required=True); parser.add_argument("--copper", type=Path, required=True); parser.add_argument("--endpoint", type=Path, required=True); parser.add_argument("--az-pattern", type=Path, required=True); parser.add_argument("--el-pattern", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.physics.resolve(), args.mapping.resolve(), args.copper.resolve(), args.endpoint.resolve(), args.az_pattern.resolve(), args.el_pattern.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
