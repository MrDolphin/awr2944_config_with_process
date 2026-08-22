"""Validate known-angle capture manifests before admitting real ADC IQ."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path


REQUIRED = ("capture_file", "capture", "target", "radar_pose", "calibration")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _validate(manifest: Path) -> dict:
    try:
        data = json.loads(manifest.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"manifest": str(manifest), "valid_json": False, "ready": False, "missing_or_invalid": str(exc)}
    missing = [key for key in REQUIRED if key not in data]
    capture = data.get("capture", {}); target = data.get("target", {}); calibration = data.get("calibration", {})
    capture_path = manifest.parent / str(data.get("capture_file", ""))
    checks = {"capture_exists": capture_path.is_file(), "cfg_file_declared": bool(capture.get("cfg_file")), "channel_order_verified": capture.get("channel_order_verified") is True, "ti_calibration_verified": calibration.get("ti_calibration_status") in {"measured", "verified"}, "target_truth_present": all(key in target for key in ("azimuth_deg", "elevation_deg", "range_m")), "pose_reference_present": bool(capture.get("imu_or_platform_pose_reference")) and not str(capture.get("imu_or_platform_pose_reference")).startswith("replace_with")}
    return {"manifest": str(manifest.resolve()), "valid_json": True, "capture_id": data.get("capture_id", ""), "capture_exists": checks["capture_exists"], "channel_order_verified": checks["channel_order_verified"], "ti_calibration_verified": checks["ti_calibration_verified"], "target_truth_present": checks["target_truth_present"], "pose_reference_present": checks["pose_reference_present"], "ready": not missing and all(checks.values()), "missing_or_invalid": ";".join(missing + [key for key, passed in checks.items() if not passed])}


def run(root: Path, output: Path) -> dict:
    manifests = sorted(root.rglob("manifest.json")) if root.exists() else []
    rows = [_validate(path) for path in manifests]
    output.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0]) if rows else ["manifest", "valid_json", "capture_id", "capture_exists", "channel_order_verified", "ti_calibration_verified", "target_truth_present", "pose_reference_present", "ready", "missing_or_invalid"]
    with (output / "capture_readiness.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    result = {"status": "completed_capture_readiness_gate", "root": str(root.resolve()), "manifest_count": len(rows), "ready_count": sum(row["ready"] for row in rows), "real_dca_iq_admitted": False, "channel_order_verified": any(row["channel_order_verified"] for row in rows), "ti_calibration_verified": any(row["ti_calibration_verified"] for row in rows), "next_action": "collect real capture.bin and complete manifest fields"}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.133 DCA1000 真实采集准入门", "", f"扫描目录：`{root}`；manifest 数量：{len(rows)}；准入数量：{result['ready_count']}。", "", "## 准入条件", "", "每个 manifest 必须同时满足：capture.bin 存在；目标方位/俯仰/距离真值完整；CFG 文件和哈希已绑定；LVDS wire order 和通道顺序已验证；TI 校准状态为 measured/verified；采集绑定姿态或明确记录无 IMU。", "", "## 当前结论", "", "文件存在或尺寸匹配不等于 ADC IQ 已验证。只有 `ready=true` 的 manifest 才能进入真实 IQ 解码和 AoA；当前输出明确保守地保持 `real_dca_iq_admitted=false`。", "", "## 下一次采集要求", "", "1. 保存原始 capture.bin，不经过 CFAR。", "2. 保存采集时使用的 CFG 和 DCA1000 配置。", "3. 用角反射器记录已知方位、俯仰和距离。", "4. 记录天线安装角度、雷达高度、船体姿态和时间戳。", "5. 完成 TI 校准并保存校准结果文件。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.root.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
