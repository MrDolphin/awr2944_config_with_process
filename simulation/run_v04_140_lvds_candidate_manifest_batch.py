"""Generate a review-only batch of capture manifests for the LVDS candidate CFG."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from simulation.run_v04_111_cfg_linked_manifest import build_manifest


CASES = [
    ("lv001_az-30_el-10_r10", -30.0, -10.0, 10.0),
    ("lv002_az-15_el-10_r10", -15.0, -10.0, 10.0),
    ("lv003_az+00_el-10_r10", 0.0, -10.0, 10.0),
    ("lv004_az+15_el-10_r10", 15.0, -10.0, 10.0),
    ("lv005_az+30_el-10_r10", 30.0, -10.0, 10.0),
]


def run(cfg: Path, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    for capture_id, azimuth, elevation, range_m in CASES:
        case = output / capture_id
        case.mkdir(parents=True, exist_ok=True)
        manifest = build_manifest(cfg, capture_id=capture_id)
        manifest["target"].update({"azimuth_deg": azimuth, "elevation_deg": elevation, "range_m": range_m})
        manifest["capture"]["cfg_file"] = cfg.name
        manifest["evidence_status"] = "candidate_cfg_awaiting_operator_review"
        manifest["candidate_cfg"] = {
            "authoritative_hardware_cfg": False,
            "review_required": True,
            "source_stage": "V0.4.139",
        }
        (case / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
        shutil.copy2(cfg, case / cfg.name)
        (case / "README.md").write_text(
            f"# {capture_id}\n\n目标：方位 {azimuth}°，俯仰 {elevation}°，距离 {range_m} m。\n\n"
            "这是加入 `lvdsStreamCfg -1 0 1 0` 的非权威候选 CFG。先由操作员确认 TI SDK/固件兼容、DCA1000 连接和安全停机条件，再放入 capture.bin、DCA 配置、TI 校准文件和姿态记录。\n\n"
            "当前目录是采集模板，不代表已下发配置或已完成实测。\n",
            encoding="utf-8",
        )
    sha = build_manifest(cfg)["capture"]["cfg_sha256"]
    result = {
        "status": "completed_lvds_candidate_manifest_batch",
        "case_count": len(CASES),
        "cfg": str(cfg.resolve()),
        "cfg_sha256": sha,
        "authoritative_hardware_cfg": False,
        "real_capture_present": False,
        "hardware_aoa_validated": False,
        "evidence_status": "candidate_cfg_awaiting_operator_review",
        "next_action": "operator_confirm_sdk_firmware_and_run_short_center_angle_capture",
    }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.140 LVDS 候选配置五角度采集模板\n\n"
        "## 本阶段做了什么\n\n"
        "在 V0.4.135 正式 CFG 模板之外，使用 V0.4.139 生成的非权威 LVDS 候选 CFG，建立五个独立的已知角度目录。每个目录包含 manifest.json、CFG 副本和 README，且记录候选 CFG SHA-256。\n\n"
        "## 结果如何解读\n\n"
        f"- 模板数量：{len(CASES)}；方位为 -30°、-15°、0°、+15°、+30°，俯仰 -10°，距离 10 m。\n"
        f"- 候选 CFG SHA-256：`{sha}`。\n"
        "- `authoritative_hardware_cfg=false`：没有修改正式 CFG，也没有自动下发雷达。\n"
        "- `real_capture_present=false`、`hardware_aoa_validated=false`：目前只能说明采集资料已准备，不能作为硬件 AoA 精度结论。\n\n"
        "## 操作员准入\n\n"
        "1. 确认当前 AWR2944P SDK/固件支持该 LVDS 命令，并确认 DCA1000 CLI 与采集主机连接。\n"
        "2. 先只使用中心角 `lv003_az+00_el-10_r10` 做短时采集。\n"
        "3. 保存 capture.bin、DCA 配置、TI 校准输出、线缆/通道顺序记录和 IMU/安装姿态。\n"
        "4. 通过 V0.4.133 准入门后，才进入原始 IQ 解码和实测 AoA。\n\n"
        "若 SDK/固件不兼容，应停止使用本批候选模板，回退到已验证的数据输出方案；不要把候选 CFG 当作 TI 官方配置。\n",
        encoding="utf-8",
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.cfg.resolve(), args.output.resolve()), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
