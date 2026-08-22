"""Validate the center-angle candidate capture directory without decoding or mutating it."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run(case_dir: Path, output: Path) -> dict:
    case_dir = case_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = case_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else {}
    capture = manifest.get("capture", {})
    cfg_name = str(capture.get("cfg_file", "profile.cfg"))
    cfg_path = case_dir / cfg_name
    capture_name = str(manifest.get("capture_file", "capture.bin"))
    capture_path = case_dir / capture_name
    issues: list[str] = []
    if not manifest_path.is_file():
        issues.append("manifest_missing")
    if not cfg_path.is_file():
        issues.append("cfg_missing")
    actual_cfg_sha = sha256(cfg_path) if cfg_path.is_file() else None
    expected_cfg_sha = capture.get("cfg_sha256")
    if not expected_cfg_sha or actual_cfg_sha != expected_cfg_sha:
        issues.append("cfg_sha256_mismatch")
    if not capture_path.is_file():
        issues.append("capture_missing")
    elif capture_path.stat().st_size == 0:
        issues.append("capture_empty")
    if manifest.get("target", {}).get("azimuth_deg") != 0.0:
        issues.append("not_center_azimuth")
    if manifest.get("target", {}).get("elevation_deg") != -10.0:
        issues.append("unexpected_center_elevation")
    for name, present in (("capture_timestamp", bool(capture.get("capture_timestamp") and "replace_with" not in str(capture.get("capture_timestamp")))), ("imu_or_platform_pose_reference", bool(capture.get("imu_or_platform_pose_reference") and "replace_with" not in str(capture.get("imu_or_platform_pose_reference"))))):
        if not present:
            issues.append(f"{name}_missing")
    result = {
        "status": "center_capture_ready_for_decode" if not issues else "awaiting_center_capture_inputs",
        "case_dir": str(case_dir),
        "manifest": str(manifest_path),
        "cfg": str(cfg_path),
        "capture": str(capture_path),
        "cfg_sha256_expected": expected_cfg_sha,
        "cfg_sha256_actual": actual_cfg_sha,
        "capture_bytes": capture_path.stat().st_size if capture_path.is_file() else 0,
        "issues": issues,
        "ready_for_decode": not issues,
        "channel_order_verified": bool(capture.get("channel_order_verified", False)),
        "ti_calibration_status": manifest.get("calibration", {}).get("ti_calibration_status"),
        "hardware_aoa_validated": False,
        "next_action": "run V0.4.112 manifest IQ decode" if not issues else "complete the listed capture inputs, then rerun this check",
    }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    issue_text = "、".join(issues) if issues else "无"
    (output / "output_analysis.md").write_text(
        "# V0.4.141 中心角短时采集准入检查\n\n"
        f"案例目录：`{case_dir}`\n\n"
        f"检查状态：`{result['status']}`\n\n"
        f"缺项或不一致：`{issue_text}`\n\n"
        "## 判定含义\n\n"
        "本检查只验证 manifest、CFG 副本、CFG SHA-256、capture.bin 非空以及中心角记录是否齐全；它不解码 IQ，也不证明 LVDS wire order、TI 校准或硬件 AoA 精度。即使 `ready_for_decode=true`，仍需运行 V0.4.112 并完成已知角度通道顺序和校准验证。\n\n"
        f"下一步：{result['next_action']}。\n",
        encoding="utf-8",
    )
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
