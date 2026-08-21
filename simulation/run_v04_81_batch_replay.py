"""Batch replay BIN/HDF5/NPZ IQ files through TDM, RD and exploratory AoA."""

from __future__ import annotations

import argparse, csv, json, shutil
from pathlib import Path
import h5py
import numpy as np

from simulation.dca1000_iq import decode_interleaved_iq, reshape_tdm_virtual_channels
from simulation.run_v04_75_coordinate_transform_validation import _load_cfg_geometry
from simulation.run_v04_79_raw_iq_contract import validate
from simulation.v03 import FmcwConfig, process_range_doppler
from simulation.v04 import estimate_aoa_from_positions


def _load_input(path: Path, chirps: int | None, samples: int | None, rx_count: int, tx_sequence: tuple[int, ...]):
    metadata = {}
    if path.suffix.lower() == ".bin":
        if chirps is None or samples is None:
            raise ValueError("--chirps and --samples-per-chirp are required for .bin")
        raw = decode_interleaved_iq(path.read_bytes(), chirps=chirps, samples_per_chirp=samples, rx_count=rx_count)
        return raw, reshape_tdm_virtual_channels(raw, tx_sequence=tx_sequence), metadata
    if path.suffix.lower() == ".npz":
        archive = np.load(path, allow_pickle=False)
        data = np.asarray(archive["iq"] if "iq" in archive else archive["raw_iq"])
        return (data, reshape_tdm_virtual_channels(data, tx_sequence=tx_sequence), metadata) if data.ndim == 3 else (None, data, metadata)
    with h5py.File(path, "r") as handle:
        metadata = {str(k): (v.decode() if isinstance(v, bytes) else v.item() if hasattr(v, "item") else v) for k, v in handle.attrs.items()}
        raw_key = "/radar/raw_iq" if "/radar/raw_iq" in handle else ("/radar/iq" if "/radar/iq" in handle else None)
        virtual_key = "/radar/virtual_iq" if "/radar/virtual_iq" in handle else ("/recovered/virtual_iq" if "/recovered/virtual_iq" in handle else None)
        raw = handle[raw_key][...] if raw_key else None
        virtual = handle[virtual_key][...] if virtual_key else (reshape_tdm_virtual_channels(raw, tx_sequence=tx_sequence) if raw is not None else None)
        if virtual is None:
            raise ValueError("HDF5 has no raw or virtual IQ dataset")
        return raw, virtual, metadata


def _one(path: Path, out: Path, mapping_csv: Path, chirps: int | None, samples: int | None, rx_count: int, tx_sequence: tuple[int, ...]):
    raw, virtual, metadata = _load_input(path, chirps, samples, rx_count, tx_sequence)
    if virtual.ndim != 4 or virtual.shape[2:] != (4, 4):
        raise ValueError(f"virtual IQ must end in (4,4), got {virtual.shape}")
    config = FmcwConfig(samples_per_chirp=virtual.shape[1], chirps_per_frame=virtual.shape[0])
    x, y = _load_cfg_geometry(mapping_csv, config)
    rd = process_range_doppler(virtual, config)
    channel = np.mean(virtual, axis=(0, 1))
    az, el = estimate_aoa_from_positions(channel, config, x, y)
    out.mkdir(parents=True, exist_ok=True)
    h5_path = out / "replay.h5"
    with h5py.File(h5_path, "w") as handle:
        attrs = {"schema_version":"awr2944p-raw-iq-contract-v0.4.79", "raw_dtype":str(metadata.get("raw_dtype", "decoded_complex")), "wire_order_assumption":str(metadata.get("wire_order_assumption", "unknown")), "channel_order_verified":bool(metadata.get("channel_order_verified", False)), "cfg_snapshot":str(mapping_csv.resolve()), "tdm_tx_sequence":",".join(map(str,tx_sequence)), "ti_calibration_status":str(metadata.get("ti_calibration_status", "unknown")), "capture_timestamp":str(metadata.get("capture_timestamp", "unknown")), "imu_or_platform_pose_reference":str(metadata.get("imu_or_platform_pose_reference", "unknown"))}
        for key, value in attrs.items(): handle.attrs[key] = value
        if raw is not None: handle.create_dataset("/radar/raw_iq", data=raw, compression="gzip")
        handle.create_dataset("/radar/virtual_iq", data=virtual, compression="gzip")
        handle.create_dataset("/range_doppler/power_linear", data=rd.power_linear)
        handle.create_dataset("/axes/range_m", data=rd.range_axis_m); handle.create_dataset("/axes/velocity_mps", data=rd.velocity_axis_mps)
    contract = validate(h5_path)
    summary = {"file":str(path.resolve()), "status":"completed_batch_replay", "raw_shape":list(raw.shape) if raw is not None else None, "virtual_shape":list(virtual.shape), "contract_status":contract["status"], "estimated_azimuth_deg":float(az), "estimated_elevation_deg":float(el), "peak_range_m":rd.peak_range_m, "peak_velocity_mps":rd.peak_velocity_mps, "hardware_validated":False, "channel_order_verified":bool(metadata.get("channel_order_verified", False))}
    (out/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding="utf-8")
    (out/"output_analysis.md").write_text(f"# V0.4.81 批量回放：{path.name}\n\n输入类型：`{path.suffix.lower()}`；原始形状：`{summary['raw_shape']}`；虚拟形状：`{summary['virtual_shape']}`。\n\n契约状态：`{contract['status']}`。AoA 为平均虚拟通道的探索性统计，不替代基于 Range-Doppler 复数 bin 的实测 AoA。\n",encoding="utf-8")
    return summary


def run(input_dir: Path, output: Path, mapping_csv: Path, chirps: int | None = None, samples: int | None = None, rx_count: int = 4, tx_sequence: tuple[int, ...] = (0,1,2,3)) -> dict:
    files = sorted(p for p in input_dir.iterdir() if p.suffix.lower() in {".bin", ".h5", ".npz"})
    rows, errors = [], []
    for path in files:
        try: rows.append(_one(path, output/path.stem, mapping_csv, chirps, samples, rx_count, tx_sequence))
        except Exception as exc: errors.append({"file":str(path.resolve()), "error":str(exc)})
    output.mkdir(parents=True, exist_ok=True)
    with (output/"manifest.csv").open("w",newline="",encoding="utf-8") as handle:
        fields=["file","status","contract_status","raw_shape","virtual_shape","estimated_azimuth_deg","estimated_elevation_deg","peak_range_m","peak_velocity_mps"]; writer=csv.DictWriter(handle,fieldnames=fields); writer.writeheader(); writer.writerows([{k:row.get(k) for k in fields} for row in rows])
    summary={"status":"completed_batch_replay", "input_file_count":len(files), "success_count":len(rows), "error_count":len(errors), "errors":errors, "hardware_validated":False}
    (output/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding="utf-8")
    (output/"output_analysis.md").write_text("# V0.4.81 多文件原始 IQ 批量回放\n\n支持 `.bin`、`.h5`、`.npz`。每个输入文件生成独立子目录、HDF5 回放结果、契约验证状态和 output_analysis.md；`manifest.csv` 汇总全部文件。\n\n`.bin` 文件必须额外提供 chirp 数和每 chirp sample 数。真实采集数据仍需确认 LVDS wire order、TDM TX 序列、通道顺序和 TI 校准状态。\n",encoding="utf-8")
    return summary


def main():
    p=argparse.ArgumentParser(); p.add_argument("--input-dir",type=Path,required=True); p.add_argument("--output",type=Path,required=True); p.add_argument("--mapping",type=Path,required=True); p.add_argument("--chirps",type=int); p.add_argument("--samples-per-chirp",type=int); p.add_argument("--rx-count",type=int,default=4); p.add_argument("--tx-sequence",type=int,nargs=4,default=(0,1,2,3)); a=p.parse_args(); print(json.dumps(run(a.input_dir.resolve(),a.output.resolve(),a.mapping.resolve(),a.chirps,a.samples_per_chirp,a.rx_count,tuple(a.tx_sequence)),indent=2,ensure_ascii=False))


if __name__ == "__main__": main()
