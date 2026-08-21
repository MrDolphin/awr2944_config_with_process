"""Quantify AoA error caused by candidate TX/RX channel permutations."""

from __future__ import annotations

import argparse, csv, json
from pathlib import Path
import numpy as np

from simulation.run_v04_75_coordinate_transform_validation import _load_cfg_geometry
from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_positions, generate_aoa_iq_with_positions


def permutations(channel: np.ndarray):
    return {
        "identity": channel,
        "tx_reverse": channel[:, ::-1],
        "rx_reverse": channel[::-1, :],
        "tx_rx_reverse": channel[::-1, ::-1],
        "transpose": channel.T,
        "transpose_tx_reverse": channel.T[:, ::-1],
        "transpose_rx_reverse": channel.T[::-1, :],
        "transpose_both_reverse": channel.T[::-1, ::-1],
    }


def run(mapping_csv: Path, output: Path) -> dict:
    config = FmcwConfig(samples_per_chirp=64, chirps_per_frame=16)
    x, y = _load_cfg_geometry(mapping_csv, config)
    grids = {"narrow": ((-5.0, 0.0, 5.0), (-2.0, 0.0, 2.0)),
             "wide": ((-40.0, -20.0, 0.0, 20.0, 40.0), (-15.0, 0.0, 15.0))}
    errors = {(grid, name): [] for grid in grids for name in permutations(np.zeros((4, 4), dtype=complex))}
    for grid, (azimuths, elevations) in grids.items():
      for elevation in elevations:
        for azimuth in azimuths:
            iq = generate_aoa_iq_with_positions(config, slant_range_m=20.0, radial_velocity_mps=0.0,
                azimuth_deg=azimuth, elevation_deg=elevation, x_positions_m=x, y_positions_m=y)
            channel = np.mean(iq, axis=(0, 1))
            for name, candidate in permutations(channel).items():
                est_az, est_el = estimate_aoa_from_positions(candidate, config, x, y)
                errors[(grid, name)].append((est_az-azimuth, est_el-elevation))
    rows = []
    for (grid, name), values in errors.items():
        values = np.asarray(values)
        rows.append({"grid": grid, "permutation": name, "azimuth_rmse_deg": float(np.sqrt(np.mean(values[:,0]**2))),
                     "elevation_rmse_deg": float(np.sqrt(np.mean(values[:,1]**2))),
                     "azimuth_max_abs_error_deg": float(np.max(np.abs(values[:,0]))),
                     "elevation_max_abs_error_deg": float(np.max(np.abs(values[:,1]))),
                     "sample_count": len(values), "status": "synthetic_known_angle_sensitivity"})
    output.mkdir(parents=True, exist_ok=True)
    with (output/"channel_order_sensitivity.csv").open("w",newline="",encoding="utf-8") as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    best=min(rows,key=lambda row:row["azimuth_rmse_deg"]+row["elevation_rmse_deg"])
    identity_narrow=next(r for r in rows if r["grid"]=="narrow" and r["permutation"]=="identity")
    identity_wide=next(r for r in rows if r["grid"]=="wide" and r["permutation"]=="identity")
    summary={"status":"completed_channel_order_sensitivity", "permutation_count":8, "grid_count":2, "best_candidate":best["permutation"], "best_grid":best["grid"], "identity_narrow_azimuth_rmse_deg":identity_narrow["azimuth_rmse_deg"], "identity_narrow_elevation_rmse_deg":identity_narrow["elevation_rmse_deg"], "identity_wide_azimuth_rmse_deg":identity_wide["azimuth_rmse_deg"], "identity_wide_elevation_rmse_deg":identity_wide["elevation_rmse_deg"], "mapping_validated":False, "hardware_validated":False}
    (output/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding="utf-8")
    (output/"output_analysis.md").write_text("# V0.4.77 TX/RX 通道排列敏感性\n\n"
        "本阶段使用同一 CFG 阵列生成已知角度复数 IQ，再对 4×4 通道矩阵施加 TX/RX 正反序、转置和镜像候选，最后使用原始 CFG 几何估计 AoA。\n\n"
        f"共测试 {len(rows)} 个‘角度范围×排列’组合，综合最小候选为 `{best['grid']}/{best['permutation']}`。窄角基线用于检查软件自洽性，宽角结果还会包含当前阵列间距造成的相位展开/空间混叠影响。除 identity 外的误差表示通道顺序错误可能造成的 AoA 偏差，不用于证明某一候选是真实硬件顺序。\n\n"
        "必须用 DCA1000 原始 IQ、已知角反射器或 TI 输出数据确认真实通道顺序。\n",encoding="utf-8")
    return summary


def main():
    p=argparse.ArgumentParser(); p.add_argument("--mapping",type=Path,required=True); p.add_argument("--output",type=Path,required=True); a=p.parse_args(); print(json.dumps(run(a.mapping.resolve(),a.output.resolve()),indent=2,ensure_ascii=False))


if __name__ == "__main__": main()
