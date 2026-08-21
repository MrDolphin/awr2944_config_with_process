"""Validate the manifest and evidence package required for measured AoA."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


REQUIRED_SCENE_KEYS = ("scene_id", "capture", "cfg", "truth")


def check_manifest(manifest_path: Path) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")); root = manifest_path.parent; scenes = manifest.get("scenes", []); scene_checks = []
    for scene in scenes:
        missing_keys = [key for key in REQUIRED_SCENE_KEYS if key not in scene]
        capture = root / scene["capture"] if "capture" in scene else None; cfg = root / scene["cfg"] if "cfg" in scene else None; truth = root / scene["truth"] if "truth" in scene else None; calibration = root / scene["calibration"] if scene.get("calibration") else None
        scene_checks.append({"scene_id": scene.get("scene_id", "<missing>"), "missing_keys": missing_keys, "capture_exists": bool(capture and capture.is_file()), "cfg_exists": bool(cfg and cfg.is_file()), "truth_exists": bool(truth and truth.is_file()), "calibration_exists": bool(calibration and calibration.is_file()), "truth_fields_present": _truth_fields(truth), "calibration_measured": _calibration_measured(calibration)})
    distinct_angles = {(check["scene_id"],) for check in scene_checks}
    checks = {"scene_count_at_least_4": len(scene_checks) >= 4, "all_scene_keys_present": all(not check["missing_keys"] for check in scene_checks), "all_capture_files_present": all(check["capture_exists"] for check in scene_checks), "all_cfg_files_present": all(check["cfg_exists"] for check in scene_checks), "all_truth_files_present": all(check["truth_exists"] and check["truth_fields_present"] for check in scene_checks), "all_measured_calibration_present": all(check["calibration_exists"] and check["calibration_measured"] for check in scene_checks), "angle_distance_coverage_declared": bool(manifest.get("coverage", {}).get("azimuth_deg")) and bool(manifest.get("coverage", {}).get("elevation_deg")) and bool(manifest.get("coverage", {}).get("range_m"))}
    return {"status": "completed_measured_aoa_package_check", "manifest": str(manifest_path.resolve()), "scene_checks": scene_checks, "checks": checks, "measured_aoa_ready": all(checks.values()), "distinct_scene_count": len(distinct_angles)}


def _truth_fields(path: Path | None) -> bool:
    if path is None or not path.is_file():
        return False
    try:
        data = json.loads(path.read_text(encoding="utf-8")); return all(key in data for key in ("azimuth_deg", "elevation_deg", "range_m"))
    except (OSError, json.JSONDecodeError):
        return False


def _calibration_measured(path: Path | None) -> bool:
    if path is None or not path.is_file():
        return False
    try:
        data = json.loads(path.read_text(encoding="utf-8")); return str(data.get("calibration_status", data.get("status", ""))).lower() in {"measured", "hardware_measured", "ti_measured"}
    except (OSError, json.JSONDecodeError):
        return False


def run(manifest_path: Path, output: Path) -> dict:
    summary = check_manifest(manifest_path); output.mkdir(parents=True, exist_ok=True); (output / "package_check.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.56 实测 AoA 验收包检查", "", f"真实 AoA 就绪：{'是' if summary['measured_aoa_ready'] else '否'}", "", "| 检查项 | 结果 |", "|---|---|"]
    lines.extend(f"| {key} | {'通过' if value else '未通过'} |" for key, value in summary["checks"].items())
    lines += ["", "## 场景逐项状态", "", "| 场景 | capture | CFG | truth | calibration |", "|---|---|---|---|---|"]
    lines.extend(f"| {scene['scene_id']} | {'有' if scene['capture_exists'] else '缺失'} | {'有' if scene['cfg_exists'] else '缺失'} | {'有' if scene['truth_exists'] and scene['truth_fields_present'] else '缺失/字段不全'} | {'实测' if scene['calibration_measured'] else '缺失/非实测'} |" for scene in summary["scene_checks"])
    lines += ["", "## 解释", "", "该检查器只判断证据包是否完整，不会把合成 fixture 或 identity calibration 当作实测通过。所有场景必须同时具备原始 IQ、CFG、角度/距离真值和实测校准 provenance，才允许进入 V0.55 综合 AoA 门禁。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8"); return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--manifest", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.manifest.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
