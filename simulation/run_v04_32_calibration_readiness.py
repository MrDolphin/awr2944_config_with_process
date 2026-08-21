"""Check whether capture/calibration inputs are sufficient for real AoA validation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py


def inspect_hdf5(path: Path) -> dict:
    result = {"path": str(path.resolve()), "exists": path.is_file(), "status": "missing"}
    if not path.is_file():
        return result
    try:
        with h5py.File(path, "r") as handle:
            datasets = sorted(handle.keys())
            required = ["decoded", "recovered"]
            result.update({"status": "readable", "datasets": datasets, "has_decoded": "decoded" in handle, "has_recovered": "recovered" in handle, "has_calibrated": "calibrated" in handle, "channel_order_verified": bool(handle.attrs.get("channel_order_verified", False)), "schema_version": str(handle.attrs.get("schema_version", ""))})
            result["ready_for_aoa"] = all(result[key] for key in ("has_recovered",)) and result["channel_order_verified"]
    except Exception as exc:
        result["status"] = f"read_error:{type(exc).__name__}"
    return result


def inspect_calibration(path: Path) -> dict:
    result = {"path": str(path.resolve()), "exists": path.is_file(), "status": "missing", "measured": False, "shape_valid": False}
    if not path.is_file():
        return result
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        amplitude, phase = data.get("amplitude"), data.get("phase_deg")
        result.update({"status": "readable", "shape_valid": len(amplitude) == 4 and all(len(row) == 4 for row in amplitude) and len(phase) == 4 and all(len(row) == 4 for row in phase), "calibration_status": data.get("calibration_status", ""), "measured": data.get("calibration_status", "").lower() in {"measured", "hardware_measured", "ti_measured"}})
    except Exception as exc:
        result["status"] = f"read_error:{type(exc).__name__}"
    return result


def run(capture: Path | None, calibration: Path | None, cfg: Path | None, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    capture_info = inspect_hdf5(capture) if capture else {"status": "not_supplied", "ready_for_aoa": False}
    calibration_info = inspect_calibration(calibration) if calibration else {"status": "not_supplied", "measured": False, "shape_valid": False}
    cfg_info = {"path": str(cfg.resolve()), "exists": cfg.is_file(), "status": "present" if cfg.is_file() else "missing"} if cfg else {"status": "not_supplied"}
    checks = {
        "capture_readable": capture_info.get("status") == "readable",
        "capture_channel_order_verified": capture_info.get("channel_order_verified", False),
        "measured_calibration": calibration_info.get("measured", False),
        "calibration_shape_4x4": calibration_info.get("shape_valid", False),
        "cfg_present": cfg_info.get("exists", False),
    }
    summary = {"status": "completed_calibration_readiness_check", "checks": checks, "real_aoa_ready": all(checks.values()), "capture": capture_info, "calibration": calibration_info, "cfg": cfg_info}
    (output / "readiness_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.32 DCA1000/校准数据就绪性", "", f"- 真实 AoA 就绪：{'是' if summary['real_aoa_ready'] else '否'}", "", "| 检查项 | 结果 |", "|---|---|"]
    for key, value in checks.items():
        lines.append(f"| {key} | {'通过' if value else '未通过'} |")
    lines += ["", "## 解释", "", "未通过不代表代码错误，只表示当前输入不能支持真实阵列 AoA 结论。尤其是 `channel_order_verified` 和 `measured_calibration` 必须通过实测已知角目标或 TI 校准流程确认，不能用合成 fixture 代替。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", type=Path)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--cfg", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.capture.resolve() if args.capture else None, args.calibration.resolve() if args.calibration else None, args.cfg.resolve() if args.cfg else None, args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
