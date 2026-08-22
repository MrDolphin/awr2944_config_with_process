"""Run the center-capture readiness gate before manifest-bound IQ decoding."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from simulation.run_v04_112_manifest_iq_decode import run as decode_manifest
from simulation.run_v04_141_center_capture_readiness import run as readiness_check


def run(case_dir: Path, output: Path) -> dict:
    case_dir = case_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    readiness = readiness_check(case_dir, output / "readiness")
    if not readiness["ready_for_decode"]:
        result = {
            "status": "decode_not_started_readiness_gate_failed",
            "case_dir": str(case_dir),
            "readiness": readiness,
            "decode_started": False,
            "decoded": False,
            "hardware_aoa_validated": False,
            "next_action": "complete readiness issues, then rerun this command",
        }
    else:
        manifest = case_dir / "manifest.json"
        decoded = decode_manifest(manifest, output / "decoded")
        result = {
            "status": "decode_completed_readiness_gate_passed" if decoded.get("decoded") else "decode_failed_after_readiness_gate",
            "case_dir": str(case_dir),
            "readiness": readiness,
            "decode": decoded,
            "decode_started": True,
            "decoded": bool(decoded.get("decoded")),
            "hardware_aoa_validated": False,
            "next_action": "inspect IQ shape and verify channel order before AoA" if decoded.get("decoded") else "inspect decoder issues",
        }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    if not result["decode_started"]:
        body = "# V0.4.142 中心角 IQ 解码门控\n\n准入检查未通过，因此没有启动 IQ 解码。\n\n"
        body += "缺项：`" + "`、`".join(readiness["issues"]) + "`。\n\n"
        body += "这不是解码失败，而是有意阻止在原始采集资料不完整时产生误导性结果。\n"
    else:
        body = "# V0.4.142 中心角 IQ 解码门控\n\n准入检查已通过，已调用 V0.4.112 manifest 绑定解码器。\n\n"
        body += f"解码状态：`{result['status']}`。\n\n"
        body += "即使解码成功，通道顺序、TI 校准和硬件 AoA 仍需独立验证。\n"
    (output / "output_analysis.md").write_text(body, encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.case, args.output), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
