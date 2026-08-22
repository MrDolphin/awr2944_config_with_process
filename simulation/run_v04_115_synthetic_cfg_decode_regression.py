"""Run a clearly labelled synthetic full-dimension IQ regression for V0.4.112."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_50_dca_reference_calibration import encode_capture_for_test
from simulation.run_v04_111_cfg_linked_manifest import build_manifest, parse_cfg
from simulation.run_v04_112_manifest_iq_decode import run as decode_manifest


def run(cfg_path: Path, output: Path) -> dict:
    cfg = parse_cfg(cfg_path)
    output.mkdir(parents=True, exist_ok=True)
    capture_path = output / "synthetic_capture.bin"
    fast = np.arange(cfg["adc_samples"])[None, :, None]
    slow = np.arange(cfg["chirps_per_frame"])[:, None, None]
    phase = 2.0 * np.pi * (12.0 * fast / cfg["adc_samples"] + 0.0 * slow)
    iq = (1200.0 * np.exp(1j * phase) * np.ones((1, 1, cfg["rx_count"]))).astype(complex)
    capture_path.write_bytes(encode_capture_for_test(iq))
    manifest = build_manifest(cfg_path, capture_id="synthetic_cfg_regression")
    manifest["capture_file"] = capture_path.name
    manifest["capture"]["capture_timestamp"] = "synthetic_fixture_not_hardware"
    manifest["evidence_status"] = "synthetic_regression_only"
    manifest_path = output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    decoded = decode_manifest(manifest_path, output / "decoded")
    shape = None
    if decoded.get("decoded_h5"):
        with h5py.File(decoded["decoded_h5"], "r") as handle:
            shape = list(handle["/decoded/iq"].shape)
    result = {"status": "completed_synthetic_full_dimension_decode_regression", "cfg": str(cfg_path.resolve()), "capture": str(capture_path.resolve()), "manifest": str(manifest_path.resolve()), "decoded": decoded, "decoded_iq_shape": shape, "expected_iq_shape": [cfg["chirps_per_frame"], cfg["adc_samples"], cfg["rx_count"]], "synthetic_only": True, "hardware_aoa_validated": False}
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text("# V0.4.115 当前 CFG 全尺寸合成 IQ 回归\n\n本阶段只验证解码链路，不代表硬件测量。\n\n" + f"输入合成 IQ：`{capture_path.name}`；期望形状：`{result['expected_iq_shape']}`；实际形状：`{shape}`；TDM 顺序：`{cfg['tx_order_bit_index']}`。\n\n合成文件明确标记为 `synthetic_only=true`，不能用于 AWR2944P 实测 AoA、探测距离或海杂波结论。真实 capture.bin 到来后，应替换输入并保留独立结果目录。\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--cfg", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.cfg.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
