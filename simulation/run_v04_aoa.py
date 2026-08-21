"""Generate one V0.4 virtual-array AoA contract run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py

from simulation.artifacts import create_run_directory
from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_channels, generate_aoa_scatterer_iq


def run(*, results_root: Path, run_id: str, azimuth_deg: float, elevation_deg: float) -> Path:
    config = FmcwConfig()
    iq = generate_aoa_scatterer_iq(config, slant_range_m=30.0, radial_velocity_mps=0.0,
                                   azimuth_deg=azimuth_deg, elevation_deg=elevation_deg)
    estimated_az, estimated_el = estimate_aoa_from_channels(iq, config)
    output = create_run_directory(results_root, producer="python",
                                  stage_id="v04_aoa_virtual_array", run_id=run_id)
    with h5py.File(output / "data" / "aoa_contract.h5", "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-aoa-v0.4"
        handle.attrs["producer"] = "python"
        handle.attrs["array_layout"] = "synthetic_tx_x_rx_y_not_evm_layout"
        handle.create_dataset("/radar/iq", data=iq, compression="gzip")
        handle.create_dataset("/truth/azimuth_deg", data=azimuth_deg)
        handle.create_dataset("/truth/elevation_deg", data=elevation_deg)
        handle.create_dataset("/estimate/azimuth_deg", data=estimated_az)
        handle.create_dataset("/estimate/elevation_deg", data=estimated_el)
    summary = {"truth_azimuth_deg": azimuth_deg, "truth_elevation_deg": elevation_deg,
               "estimated_azimuth_deg": estimated_az, "estimated_elevation_deg": estimated_el,
               "azimuth_error_deg": estimated_az - azimuth_deg,
               "elevation_error_deg": estimated_el - elevation_deg,
               "iq_shape": list(iq.shape), "array_layout": "synthetic_not_evm_layout"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4 AoA 输出分析\n\n"
        f"- 输入方位角：{azimuth_deg:.3f}°；输入俯仰角：{elevation_deg:.3f}°。\n"
        f"- 估计方位角：{estimated_az:.3f}°；估计俯仰角：{estimated_el:.3f}°。\n"
        f"- 角度误差：方位 {estimated_az-azimuth_deg:.6f}°，俯仰 {estimated_el-elevation_deg:.6f}°。\n\n"
        "HDF5 中 `/radar/iq` 维度为 `(chirp, sample, rx, tx)`；相位差来自软件定义的 4x4 虚拟阵列。"
        "本结果只验证已知角度到通道相位斜坡的数学合同，不代表 AWR2944P EVM 的真实阵元坐标、"
        "校准、AoA FOV、旁瓣或 TI SDK AoA 算法性能。\n",
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
