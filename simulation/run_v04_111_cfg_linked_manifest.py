"""Generate a known-angle capture manifest linked to an actual radar CFG."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_cfg(path: Path) -> dict:
    profile = None; frame = None; chirps = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        parts = raw.split()
        if not parts or parts[0].startswith("%"):
            continue
        if parts[0] == "profileCfg":
            profile = parts
        elif parts[0] == "frameCfg":
            frame = parts
        elif parts[0] == "chirpCfg":
            chirps.append(parts)
    if profile is None or frame is None or not chirps:
        raise ValueError("CFG must contain profileCfg, frameCfg and chirpCfg")
    start_chirp, end_chirp, loops = int(frame[1]), int(frame[2]), int(frame[3])
    tx_masks = [int(item[8]) for item in chirps if len(item) > 8]
    tx_order = [mask.bit_length() - 1 for mask in tx_masks if mask > 0]
    return {"profile_id": int(profile[1]), "start_frequency_ghz": float(profile[2]), "adc_samples": int(profile[10]), "frame_start_chirp": start_chirp, "frame_end_chirp": end_chirp, "loops_per_frame": loops, "chirps_per_frame": (end_chirp - start_chirp + 1) * loops, "chirp_count": len(chirps), "tx_masks": tx_masks, "tx_order_bit_index": tx_order, "rx_count": 4, "source_sha256": _sha256(path)}


def build_manifest(cfg_path: Path, capture_id: str = "replace_with_capture_id") -> dict:
    cfg = parse_cfg(cfg_path)
    return {"schema_version": "awr2944p-known-angle-capture-v0.4.111", "capture_id": capture_id, "capture_file": "capture.bin", "capture_sha256": None, "target": {"type": "corner_reflector", "azimuth_deg": 0.0, "elevation_deg": 0.0, "range_m": 10.0, "radial_velocity_mps": 0.0, "truth_source": "survey_or_total_station"}, "radar_pose": {"installation_height_m": 1.0, "boresight_azimuth_deg": 0.0, "boresight_elevation_deg": 0.0, "board_to_radar_transform_candidate": "identity", "pose_source": "mechanical_measurement_required"}, "capture": {"cfg_file": cfg_path.name, "cfg_sha256": cfg["source_sha256"], "raw_dtype": "int16", "wire_order_assumption": "sample_rx_iq", "tdm_tx_sequence": cfg["tx_order_bit_index"], "chirps": cfg["chirps_per_frame"], "samples_per_chirp": cfg["adc_samples"], "rx_count": cfg["rx_count"], "frame_count": 0, "channel_order_verified": False, "capture_timestamp": "replace_with_iso8601", "imu_or_platform_pose_reference": "replace_with_imu_log_or_none"}, "calibration": {"ti_calibration_status": "not_measured", "calibration_file": None, "calibration_sha256": None}, "cfg_geometry": cfg, "evidence_status": "template_awaiting_capture"}


def run(cfg_path: Path, output: Path) -> dict:
    cfg = parse_cfg(cfg_path)
    output.mkdir(parents=True, exist_ok=True)
    manifest = build_manifest(cfg_path)
    (output / "known_angle_capture_manifest.template.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "cfg_geometry.json").write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding="utf-8")
    summary = {"status": "cfg_linked_manifest_created", "cfg": str(cfg_path.resolve()), "output": str(output.resolve()), "cfg_geometry": cfg, "hardware_aoa_validated": False, "next_action": "copy the template into a capture case, put capture.bin beside it, then fill target, pose, timestamp, channel and calibration fields"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.111 与 CFG 绑定的已知角采集清单", "", f"CFG：`{cfg_path.resolve()}`", f"CFG SHA-256：`{cfg['source_sha256']}`", "", "## 从 CFG 自动提取的几何", "", f"- ADC samples/chirp：`{cfg['adc_samples']}`", f"- RX：`{cfg['rx_count']}`", f"- chirps/frame：`{cfg['chirps_per_frame']}`", f"- TDM TX 顺序（按 chirp mask bit）：`{cfg['tx_order_bit_index']}`", "", "## 使用限制", "", "这只是与当前 CFG 一致的采集模板，尚未包含真实 capture.bin、通道顺序实测、TI 校准矩阵、目标测量和船体姿态。因此不能直接进入硬件 AoA 精度结论。相比旧 V0.4.95 模板，本模板不再硬编码 128 samples/64 chirps，而是从实际 CFG 读取。", "", "## 下一步", "", "将模板复制到一个具体已知角案例目录，放入原始 capture.bin 和同一份 profile.cfg，填写目标真值、安装姿态、时间戳、IMU 参考、wire order 与校准记录，再通过 V0.4.94/V0.4.90 完整性检查。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--cfg", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.cfg.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
