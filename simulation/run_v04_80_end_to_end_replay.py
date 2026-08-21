"""End-to-end synthetic raw-IQ replay through TDM, RD and AoA stages."""

from __future__ import annotations

import argparse, json
from pathlib import Path
import h5py
import numpy as np

from simulation.dca1000_iq import decode_interleaved_iq, encode_interleaved_iq, flatten_tdm_virtual_channels, reshape_tdm_virtual_channels
from simulation.run_v04_75_coordinate_transform_validation import _load_cfg_geometry
from simulation.v03 import FmcwConfig, process_range_doppler
from simulation.v04 import estimate_aoa_from_positions, generate_aoa_iq_with_positions
from simulation.run_v04_79_raw_iq_contract import validate


def run(mapping_csv: Path, output: Path, azimuth_deg: float = 5.0, elevation_deg: float = 2.0) -> dict:
    config = FmcwConfig(samples_per_chirp=64, chirps_per_frame=4)
    x, y = _load_cfg_geometry(mapping_csv, config)
    virtual = generate_aoa_iq_with_positions(config, slant_range_m=20.0, radial_velocity_mps=0.0,
        azimuth_deg=azimuth_deg, elevation_deg=elevation_deg, x_positions_m=x, y_positions_m=y, amplitude=1000.0)
    raw_chirps = flatten_tdm_virtual_channels(virtual, tx_sequence=(0, 1, 2, 3))
    encoded = encode_interleaved_iq(raw_chirps)
    decoded = decode_interleaved_iq(encoded, chirps=raw_chirps.shape[0], samples_per_chirp=raw_chirps.shape[1], rx_count=4)
    recovered = reshape_tdm_virtual_channels(decoded, tx_sequence=(0, 1, 2, 3))
    scale_error = float(np.max(np.abs(recovered - virtual)))
    normalized = recovered / 1000.0
    channel = np.mean(normalized, axis=(0, 1))
    est_az, est_el = estimate_aoa_from_positions(channel, config, x, y)
    rd = process_range_doppler(recovered, config)
    output.mkdir(parents=True, exist_ok=True)
    h5_path = output / "end_to_end_replay.h5"
    with h5py.File(h5_path, "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-raw-iq-contract-v0.4.79"
        handle.attrs["raw_dtype"] = "little_endian_int16"
        handle.attrs["wire_order_assumption"] = "sample -> rx -> I,Q"
        handle.attrs["channel_order_verified"] = False
        handle.attrs["cfg_snapshot"] = str(mapping_csv.resolve())
        handle.attrs["tdm_tx_sequence"] = "0,1,2,3"
        handle.attrs["ti_calibration_status"] = "synthetic_unity_not_measured"
        handle.attrs["capture_timestamp"] = "synthetic"
        handle.attrs["imu_or_platform_pose_reference"] = "synthetic_level_pose"
        handle.create_dataset("/radar/raw_iq", data=decoded, compression="gzip")
        handle.create_dataset("/radar/virtual_iq", data=recovered, compression="gzip")
        handle.create_dataset("/range_doppler/power_linear", data=rd.power_linear)
        handle.create_dataset("/axes/range_m", data=rd.range_axis_m)
        handle.create_dataset("/axes/velocity_mps", data=rd.velocity_axis_mps)
    contract = validate(h5_path)
    summary = {"status":"completed_end_to_end_synthetic_replay", "raw_iq_contract_status": contract["status"], "input_mapping":str(mapping_csv.resolve()), "raw_shape":list(decoded.shape), "virtual_shape":list(recovered.shape), "encoded_bytes":len(encoded), "max_int16_roundtrip_error":scale_error, "truth_azimuth_deg":azimuth_deg, "truth_elevation_deg":elevation_deg, "estimated_azimuth_deg":float(est_az), "estimated_elevation_deg":float(est_el), "azimuth_error_deg":float(est_az-azimuth_deg), "elevation_error_deg":float(est_el-elevation_deg), "peak_range_m":rd.peak_range_m, "peak_velocity_mps":rd.peak_velocity_mps, "hardware_validated":False, "channel_order_verified":False}
    (output/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding="utf-8")
    (output/"output_analysis.md").write_text("# V0.4.80 原始 IQ→TDM→RD→AoA 端到端回放\n\n"
        "本阶段先生成合成 4 TX/4 RX 虚拟 IQ，按 TX 序列 0,1,2,3 展平为原始 chirp，使用 little-endian int16 I/Q 编码，再解码和 TDM 重组。重组后的虚拟 IQ 同时进入距离-多普勒和 AoA 处理。\n\n"
        f"原始 IQ 形状：`{decoded.shape}`；虚拟 IQ 形状：`{recovered.shape}`；int16 往返最大误差：`{scale_error:.3f}`。\n\n"
        f"已知角度：az={azimuth_deg:.3f}°、el={elevation_deg:.3f}°；估计角度：az={est_az:.3f}°、el={est_el:.3f}°。\n\n"
        "## 边界\n\n"
        "这是软件端到端回放闭环，不是 DCA1000 实测数据；HDF5 中明确记录 `channel_order_verified=false` 和合成校准状态。后续可替换 `/radar/raw_iq` 的来源文件，保留同一重组、RD、AoA 和报告接口。\n",encoding="utf-8")
    return summary


def main():
    p=argparse.ArgumentParser(); p.add_argument("--mapping",type=Path,required=True); p.add_argument("--output",type=Path,required=True); p.add_argument("--azimuth",type=float,default=5.0); p.add_argument("--elevation",type=float,default=2.0); a=p.parse_args(); print(json.dumps(run(a.mapping.resolve(),a.output.resolve(),a.azimuth,a.elevation),indent=2,ensure_ascii=False))


if __name__ == "__main__": main()
