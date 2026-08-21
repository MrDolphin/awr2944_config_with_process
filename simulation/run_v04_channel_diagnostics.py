"""Diagnose AoA error signatures caused by channel-order faults."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.artifacts import create_run_directory
from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_channels, generate_aoa_scatterer_iq, virtual_array_positions


def _faults(iq: np.ndarray) -> dict[str, np.ndarray]:
    faults = {"identity": iq, "rx_reverse": iq[:, :, ::-1, :],
              "tx_reverse": iq[:, :, :, ::-1], "tx_swap_01": iq[:, :, :, [1, 0, 2, 3]],
              "iq_conjugated": np.conjugate(iq)}
    return faults


def run(*, results_root: Path, run_id: str, azimuth_deg: float = 20.0,
        elevation_deg: float = 10.0) -> Path:
    config = FmcwConfig()
    iq = generate_aoa_scatterer_iq(config, slant_range_m=30.0,
                                   radial_velocity_mps=0.0,
                                   azimuth_deg=azimuth_deg,
                                   elevation_deg=elevation_deg)
    results = {}
    for name, faulty in _faults(iq).items():
        estimated_az, estimated_el = estimate_aoa_from_channels(faulty, config)
        results[name] = {
            "estimated_azimuth_deg": estimated_az,
            "estimated_elevation_deg": estimated_el,
            "azimuth_error_deg": estimated_az - azimuth_deg,
            "elevation_error_deg": estimated_el - elevation_deg,
        }
    output = create_run_directory(results_root, producer="python",
                                  stage_id="v04_aoa_cfar_point_cloud", run_id=run_id)
    with h5py.File(output / "data" / "channel_diagnostics.h5", "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-aoa-v0.4.10"
        handle.attrs["truth_azimuth_deg"] = azimuth_deg
        handle.attrs["truth_elevation_deg"] = elevation_deg
        handle.create_dataset("/truth/iq", data=iq, compression="gzip")
        for name, faulty in _faults(iq).items():
            handle.create_dataset(f"/faults/{name}", data=faulty, compression="gzip")
    summary = {"truth_azimuth_deg": azimuth_deg, "truth_elevation_deg": elevation_deg,
               "diagnostics": results,
               "interpretation_status": "synthetic_error_fingerprints_only"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.10 通道排列诊断分析\n\n"
        f"真值方位 {azimuth_deg:.1f}°、俯仰 {elevation_deg:.1f}°。每个故障场景只改变通道排列或 I/Q 符号。\n\n"
        "| 场景 | 方位估计 | 俯仰估计 | 方位误差 | 俯仰误差 |\n|---|---:|---:|---:|---:|\n"
        + "\n".join(
            f"| {name} | {value['estimated_azimuth_deg']:.3f} | {value['estimated_elevation_deg']:.3f} | "
            f"{value['azimuth_error_deg']:.3f} | {value['elevation_error_deg']:.3f} |"
            for name, value in results.items()
        ) + "\n\n"
        "分析方法：真实抓包若呈现与某个故障场景相似的角度偏差，可优先检查对应的 RX/TX 顺序或 I/Q 符号。"
        "这只是软件误差指纹，不能代替真实 DCA1000 抓包、角反射器和通道校准。\n",
        encoding="utf-8")
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--azimuth-deg", type=float, default=20.0)
    parser.add_argument("--elevation-deg", type=float, default=10.0)
    args = parser.parse_args()
    print(run(results_root=args.results_root.resolve(), run_id=args.run_id,
              azimuth_deg=args.azimuth_deg, elevation_deg=args.elevation_deg))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
