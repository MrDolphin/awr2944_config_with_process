"""Exercise the V0.4.141 -> V0.4.142 path with explicitly synthetic IQ."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_50_dca_reference_calibration import encode_capture_for_test
from simulation.run_v04_111_cfg_linked_manifest import build_manifest, parse_cfg
from simulation.run_v04_142_center_iq_decode_gate import run as gate


def run(cfg_path: Path, output: Path) -> dict:
    cfg_path = cfg_path.resolve()
    output.mkdir(parents=True, exist_ok=True)
    case = output / "synthetic_center_case"
    case.mkdir(parents=True, exist_ok=True)
    cfg = parse_cfg(cfg_path)
    capture = case / "synthetic_capture.bin"
    fast = np.arange(cfg["adc_samples"])[None, :, None]
    slow = np.arange(cfg["chirps_per_frame"])[:, None, None]
    phase = 2.0 * np.pi * (12.0 * fast / cfg["adc_samples"] + 0.0 * slow)
    iq = 1200.0 * np.exp(1j * phase) * np.ones((1, 1, cfg["rx_count"]))
    capture.write_bytes(encode_capture_for_test(iq.astype(complex)))
    manifest = build_manifest(cfg_path, capture_id="synthetic_center_gate")
    manifest["capture_file"] = capture.name
    manifest["target"].update({"azimuth_deg": 0.0, "elevation_deg": -10.0, "range_m": 10.0})
    manifest["capture"]["capture_timestamp"] = "synthetic_fixture_not_hardware"
    manifest["capture"]["imu_or_platform_pose_reference"] = "synthetic_fixture_not_hardware"
    manifest["evidence_status"] = "synthetic_regression_only"
    (case / cfg_path.name).write_bytes(cfg_path.read_bytes())
    (case / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    gated = gate(case, output / "gate")
    result = {
        "status": "completed_synthetic_gate_regression",
        "synthetic_only": True,
        "cfg": str(cfg_path),
        "case": str(case.resolve()),
        "gate": gated,
        "decoded": bool(gated.get("decoded")),
        "hardware_aoa_validated": False,
    }
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.143 中心角准入到 IQ 解码合成回归\n\n"
        "本阶段使用明确标记的 synthetic IQ，验证 V0.4.141 准入检查通过后，V0.4.142 能够调用 manifest 绑定解码器。\n\n"
        f"解码状态：`{gated.get('status')}`；IQ 结果：`{gated.get('decoded')}`。\n\n"
        "这是软件链路回归，不是 DCA1000 真实采集，不可用于 AWR2944P 硬件 AoA、探测距离或海杂波结论。\n",
        encoding="utf-8",
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.cfg, args.output), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
