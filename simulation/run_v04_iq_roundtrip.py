"""Run V0.4.9 synthetic DCA1000/TDM/AoA roundtrip."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.artifacts import create_run_directory
from simulation.dca1000_iq import (
    decode_interleaved_iq, encode_interleaved_iq, flatten_tdm_virtual_channels,
    reshape_tdm_virtual_channels,
)
from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_positions, generate_aoa_iq_with_positions, virtual_array_positions


def run(*, results_root: Path, run_id: str, azimuth_deg: float = 20.0,
        elevation_deg: float = 10.0) -> Path:
    config = FmcwConfig(samples_per_chirp=64, chirps_per_frame=16)
    x, y = virtual_array_positions(config)
    virtual = generate_aoa_iq_with_positions(
        config, slant_range_m=30.0, radial_velocity_mps=0.0,
        azimuth_deg=azimuth_deg, elevation_deg=elevation_deg,
        x_positions_m=x, y_positions_m=y, amplitude=1000.0,
    )
    raw_chirps = flatten_tdm_virtual_channels(virtual)
    raw_bytes = encode_interleaved_iq(raw_chirps)
    decoded = decode_interleaved_iq(raw_bytes, chirps=64, samples_per_chirp=64, rx_count=4)
    recovered = reshape_tdm_virtual_channels(decoded)
    estimated_az, estimated_el = estimate_aoa_from_positions(
        np.mean(recovered[0], axis=0), config, x, y
    )
    output = create_run_directory(results_root, producer="python",
                                  stage_id="v04_aoa_cfar_point_cloud", run_id=run_id)
    with h5py.File(output / "data" / "iq_roundtrip.h5", "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-aoa-v0.4.9"
        handle.attrs["wire_order_assumption"] = "sample -> rx -> I,Q"
        handle.attrs["tx_sequence"] = "TX1,TX2,TX3,TX4"
        handle.create_dataset("/synthetic/virtual_iq", data=virtual, compression="gzip")
        handle.create_dataset("/decoded/iq", data=decoded, compression="gzip")
        handle.create_dataset("/recovered/virtual_iq", data=recovered, compression="gzip")
    summary = {
        "truth_azimuth_deg": azimuth_deg, "truth_elevation_deg": elevation_deg,
        "estimated_azimuth_deg": estimated_az, "estimated_elevation_deg": estimated_el,
        "azimuth_error_deg": estimated_az - azimuth_deg,
        "elevation_error_deg": estimated_el - elevation_deg,
        "raw_bytes": len(raw_bytes), "decoded_shape": list(decoded.shape),
        "recovered_shape": list(recovered.shape), "channel_order_verified": "synthetic_only",
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.9 DCA1000/TDM/AoA 合成闭环分析\n\n"
        f"- 输入角度：方位 {azimuth_deg:.3f}°，俯仰 {elevation_deg:.3f}°。\n"
        f"- 输出角度：方位 {estimated_az:.3f}°，俯仰 {estimated_el:.3f}°。\n"
        f"- 角度误差：方位 {estimated_az-azimuth_deg:.6f}°，俯仰 {estimated_el-elevation_deg:.6f}°。\n"
        f"- 原始字节数：{len(raw_bytes)}；解码形状：{decoded.shape}；重排形状：{recovered.shape}。\n\n"
        "本闭环验证的是软件数据格式、I/Q 交织、4TX TDM 顺序和 AoA 数学链路。它使用合成数据，"
        "不能证明真实 DCA1000 LVDS lane 顺序、RX 通道编号、I/Q 符号或实板校准已经正确。\n",
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
