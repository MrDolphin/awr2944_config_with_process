"""Run V0.4.1 AoA FOV scan with controlled channel errors and noise."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.artifacts import create_run_directory
from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_channels, generate_aoa_scatterer_iq


def run_scan(*, results_root: Path, run_id: str, seed: int = 101,
             noise_std: float = 0.02, channel_phase_std_deg: float = 1.0) -> Path:
    rng = np.random.default_rng(seed)
    config = FmcwConfig()
    azimuths = np.arange(-60.0, 60.1, 20.0)
    elevations = np.arange(-20.0, 20.1, 10.0)
    gains = (1.0 + 0.02 * rng.standard_normal((4, 4))) * np.exp(
        1j * np.deg2rad(channel_phase_std_deg) * rng.standard_normal((4, 4))
    )
    rows = []
    first_iq = None
    for elevation in elevations:
        for azimuth in azimuths:
            iq = generate_aoa_scatterer_iq(
                config, slant_range_m=30.0, radial_velocity_mps=0.0,
                azimuth_deg=float(azimuth), elevation_deg=float(elevation),
            )
            iq = iq * gains[None, None, :, :]
            iq = iq + noise_std * (
                rng.standard_normal(iq.shape) + 1j * rng.standard_normal(iq.shape)
            ) / np.sqrt(2.0)
            estimated_az, estimated_el = estimate_aoa_from_channels(iq, config)
            rows.append({
                "truth_azimuth_deg": float(azimuth), "truth_elevation_deg": float(elevation),
                "estimated_azimuth_deg": estimated_az, "estimated_elevation_deg": estimated_el,
                "azimuth_error_deg": estimated_az - azimuth,
                "elevation_error_deg": estimated_el - elevation,
            })
            if first_iq is None:
                first_iq = iq
    az_errors = np.asarray([row["azimuth_error_deg"] for row in rows])
    el_errors = np.asarray([row["elevation_error_deg"] for row in rows])
    output = create_run_directory(results_root, producer="python",
                                  stage_id="v04_aoa_cfar_point_cloud", run_id=run_id)
    with h5py.File(output / "data" / "aoa_scan.h5", "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-aoa-v0.4.1"
        handle.attrs["producer"] = "python"
        handle.attrs["fov_config"] = "-60..60 azimuth, -20..20 elevation"
        handle.create_dataset("/radar/iq_example", data=first_iq, compression="gzip")
        handle.create_dataset("/scan/results", data=np.asarray([
            [r["truth_azimuth_deg"], r["truth_elevation_deg"],
             r["estimated_azimuth_deg"], r["estimated_elevation_deg"]] for r in rows
        ]))
    summary = {
        "seed": seed, "point_count": len(rows), "noise_std": noise_std,
        "channel_phase_std_deg": channel_phase_std_deg,
        "azimuth_range_deg": [float(azimuths.min()), float(azimuths.max())],
        "elevation_range_deg": [float(elevations.min()), float(elevations.max())],
        "azimuth_rmse_deg": float(np.sqrt(np.mean(np.square(az_errors)))),
        "elevation_rmse_deg": float(np.sqrt(np.mean(np.square(el_errors)))),
        "azimuth_max_abs_error_deg": float(np.max(np.abs(az_errors))),
        "elevation_max_abs_error_deg": float(np.max(np.abs(el_errors))),
        "rows": rows,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.1 AoA FOV 扫描输出分析\n\n"
        f"- 扫描范围：方位 {azimuths.min():.0f}°～{azimuths.max():.0f}°，俯仰 {elevations.min():.0f}°～{elevations.max():.0f}°。\n"
        f"- 扫描点数：{len(rows)}；噪声标准差：{noise_std}；通道相位误差标准差：{channel_phase_std_deg}°。\n"
        f"- 方位 RMSE：{summary['azimuth_rmse_deg']:.6f}°；俯仰 RMSE：{summary['elevation_rmse_deg']:.6f}°。\n"
        f"- 最大绝对误差：方位 {summary['azimuth_max_abs_error_deg']:.6f}°，俯仰 {summary['elevation_max_abs_error_deg']:.6f}°。\n\n"
        "扫描点的输入角度是 truth，输出角度是相位斜坡估计值。RMSE 越小表示在当前"
        "噪声、通道误差和软件阵列布局下角度恢复越稳定；边界点误差若增大，通常意味着"
        "FOV 边缘的阵列相位灵敏度、噪声或角度包络问题。\n\n"
        "`aoaFovCfg -1 -60 60 -20 20` 在这里仅作为扫描范围参考，不等同于实测硬件可用范围。"
        "阵元坐标仍是合成布局，尚未加入 AWR2944P EVM 的真实标定矩阵、天线方向图、CFAR 和点云。\n",
        encoding="utf-8")
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--noise-std", type=float, default=0.02)
    parser.add_argument("--channel-phase-std-deg", type=float, default=1.0)
    args = parser.parse_args()
    print(run_scan(results_root=args.results_root.resolve(), run_id=args.run_id,
                   seed=args.seed, noise_std=args.noise_std,
                   channel_phase_std_deg=args.channel_phase_std_deg))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
