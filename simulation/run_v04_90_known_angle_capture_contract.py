"""Create and validate the metadata contract for a real known-angle capture."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from simulation.run_v04_79_raw_iq_contract import validate as validate_raw_iq


SCHEMA = "awr2944p-known-angle-capture-v0.4.90"


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def template() -> dict:
    return {
        "schema_version": SCHEMA,
        "capture_id": "replace_with_unique_capture_id",
        "capture_file": "capture.bin",
        "capture_sha256": None,
        "target": {
            "type": "corner_reflector",
            "azimuth_deg": 0.0,
            "elevation_deg": 0.0,
            "range_m": 10.0,
            "radial_velocity_mps": 0.0,
            "truth_source": "survey_or_total_station",
        },
        "radar_pose": {
            "installation_height_m": 1.0,
            "boresight_azimuth_deg": 0.0,
            "boresight_elevation_deg": 0.0,
            "board_to_radar_transform_candidate": "identity",
            "pose_source": "mechanical_measurement_required",
        },
        "capture": {
            "cfg_file": "profile.cfg",
            "cfg_sha256": None,
            "raw_dtype": "int16",
            "wire_order_assumption": "sample_rx_iq",
            "tdm_tx_sequence": [0, 1, 2, 3],
            "frame_count": 0,
            "channel_order_verified": False,
            "capture_timestamp": "replace_with_iso8601",
            "imu_or_platform_pose_reference": "replace_with_imu_log_or_none",
        },
        "calibration": {
            "ti_calibration_status": "not_measured",
            "calibration_file": None,
            "calibration_sha256": None,
        },
        "evidence_status": "template_awaiting_capture",
    }


def validate_manifest(manifest_path: Path, root: Path) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    issues: list[str] = []
    required = ["schema_version", "capture_id", "target", "radar_pose", "capture", "calibration"]
    issues.extend(f"missing:{key}" for key in required if key not in manifest)
    if manifest.get("schema_version") != SCHEMA:
        issues.append("schema_version_mismatch")
    capture_file = root / str(manifest.get("capture_file", ""))
    if not capture_file.is_file():
        issues.append("capture_file_missing")
        iq_result = {"status": "not_supplied"}
    else:
        iq_result = validate_raw_iq(capture_file)
        if iq_result["status"] == "invalid_contract":
            issues.append("raw_iq_contract_invalid")
    target = manifest.get("target", {})
    for key in ("azimuth_deg", "elevation_deg", "range_m"):
        if key not in target:
            issues.append(f"target_missing:{key}")
    cfg_file = root / str(manifest.get("capture", {}).get("cfg_file", ""))
    if not cfg_file.is_file():
        issues.append("cfg_file_missing")
    elif manifest.get("capture", {}).get("cfg_sha256") != _sha256(cfg_file):
        issues.append("cfg_sha256_mismatch")
    status = "ready_for_known_angle_processing" if not issues else "awaiting_or_invalid_capture_evidence"
    return {"status": status, "schema_version": SCHEMA, "manifest": str(manifest_path.resolve()), "issues": issues, "raw_iq": iq_result, "capture_sha256": _sha256(capture_file), "cfg_sha256": _sha256(cfg_file)}


def run(output: Path, manifest: Path | None = None) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = manifest.resolve() if manifest else output / "known_angle_capture_manifest.template.json"
    if manifest is None:
        manifest_path.write_text(json.dumps(template(), indent=2, ensure_ascii=False), encoding="utf-8")
        result = {"status": "template_created", "schema_version": SCHEMA, "manifest": str(manifest_path.resolve()), "next_action": "copy template, fill target/capture/calibration fields, then validate it"}
    else:
        result = validate_manifest(manifest_path, manifest_path.parent)
    (output / "validation.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.90 已知角实测采集契约", "", "本阶段把角反射器/已知角目标实测所需的输入固定成一个可复用的 JSON manifest。它不生成实测数据，也不把模板当作硬件证据。", "", f"状态：`{result['status']}`", "", "## 采集时必须记录", "", "- 目标方位角、俯仰角、距离及测量来源；", "- 雷达安装高度、安装姿态和 PCB→雷达变换候选；", "- 使用的 CFG 文件及 SHA-256；", "- DCA1000 原始 BIN/HDF5 文件及 SHA-256；", "- LVDS wire order、TDM TX 顺序和帧边界；", "- 通道顺序是否已由已知角目标验证；", "- TI 校准状态和校准矩阵来源；", "- 船体姿态/IMU 时间参考。", "", "## 通过条件", "", "只有原始 IQ 契约有效、CFG 文件存在且哈希匹配、目标真值字段完整，并且采集元数据完整时，才允许进入 V0.4.91 的四种坐标候选 AoA 比较。`channel_order_verified=false` 或校准状态未测量时，结果只能标记为排列/校准敏感性分析。", "", f"当前问题：`{result.get('issues', [])}`", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.output.resolve(), args.manifest.resolve() if args.manifest else None), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
