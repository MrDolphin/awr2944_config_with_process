"""Decode a capture using the dimensions and TX order in a V0.4.111 manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py

from simulation.run_v04_50_dca_reference_calibration import process_capture


UART_SYNC = bytes.fromhex("02 01 04 03 06 05 08 07")


def run(manifest_path: Path, output: Path) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    root = manifest_path.parent
    capture_path = root / str(manifest.get("capture_file", "capture.bin"))
    capture = manifest.get("capture", {})
    required = {"samples_per_chirp": int(capture.get("samples_per_chirp", 0)), "chirps": int(capture.get("chirps", 0)), "rx_count": int(capture.get("rx_count", 0))}
    issues = [f"missing_or_invalid:{name}" for name, value in required.items() if value <= 0]
    if not capture_path.is_file():
        issues.append("capture_file_missing")
    elif capture_path.read_bytes()[:8] == UART_SYNC:
        issues.append("input_is_uart_point_cloud_not_dca1000_iq")
    output.mkdir(parents=True, exist_ok=True)
    if issues:
        result = {"status": "awaiting_valid_dca_iq", "manifest": str(manifest_path.resolve()), "capture": str(capture_path.resolve()), "issues": issues, "required_geometry": required, "decoded": False, "hardware_aoa_validated": False}
        (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        (output / "output_analysis.md").write_text("# V0.4.112 Manifest 绑定 IQ 解码\n\n当前未进入解码。\n\n问题：`" + "`, `".join(issues) + "`。\n\n只有真正的 DCA1000 ADC IQ、与 manifest 同 CFG、且通道顺序/校准证据完整后，才允许进入硬件 AoA 结论。\n", encoding="utf-8")
        return result
    tx_sequence = tuple(int(value) for value in capture.get("tdm_tx_sequence", []))
    if not tx_sequence:
        result = {"status": "awaiting_valid_dca_iq", "manifest": str(manifest_path.resolve()), "capture": str(capture_path.resolve()), "issues": ["tdm_tx_sequence_missing"], "decoded": False, "hardware_aoa_validated": False}
        (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        return result
    decoded_h5 = output / "decoded_capture.h5"
    calibration = process_capture(capture_path, decoded_h5, chirps=required["chirps"], samples_per_chirp=required["samples_per_chirp"], rx_count=required["rx_count"], expected_range_m=float(manifest.get("target", {}).get("range_m", 10.0)), expected_velocity_mps=float(manifest.get("target", {}).get("radial_velocity_mps", 0.0)), tx_sequence=tx_sequence)
    with h5py.File(decoded_h5, "r") as handle:
        decoded_shape = list(handle["/decoded/iq"].shape)
    result = {"status": "decoded_dca_iq_channel_order_unverified", "manifest": str(manifest_path.resolve()), "capture": str(capture_path.resolve()), "decoded_h5": str(decoded_h5.resolve()), "decoded_iq_shape": decoded_shape, "tx_sequence": list(tx_sequence), "calibration": calibration, "decoded": True, "channel_order_verified": bool(capture.get("channel_order_verified", False)), "ti_calibration_status": manifest.get("calibration", {}).get("ti_calibration_status"), "hardware_aoa_validated": False}
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text("# V0.4.112 Manifest 绑定 IQ 解码\n\n已按 manifest 解码，但通道顺序、TI 校准和硬件 AoA 仍未验证，不能作为最终精度结论。\n\n" + f"IQ 形状：`{result['decoded_iq_shape']}`；TDM 顺序：`{list(tx_sequence)}`。\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--manifest", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.manifest.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
