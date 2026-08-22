"""Generate five CFG-linked known-angle manifest templates."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from simulation.run_v04_111_cfg_linked_manifest import build_manifest


CASES = [
    ("ka001_az-30_el-10_r10", -30.0, -10.0, 10.0),
    ("ka002_az-15_el-10_r10", -15.0, -10.0, 10.0),
    ("ka003_az+00_el-10_r10", 0.0, -10.0, 10.0),
    ("ka004_az+15_el-10_r10", 15.0, -10.0, 10.0),
    ("ka005_az+30_el-10_r10", 30.0, -10.0, 10.0),
]


def run(cfg: Path, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    for capture_id, azimuth, elevation, range_m in CASES:
        case = output / capture_id; case.mkdir(parents=True, exist_ok=True)
        manifest = build_manifest(cfg, capture_id=capture_id)
        manifest["target"].update({"azimuth_deg": azimuth, "elevation_deg": elevation, "range_m": range_m})
        manifest["capture"]["cfg_file"] = cfg.name
        manifest["evidence_status"] = "template_awaiting_capture"
        (case / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
        shutil.copy2(cfg, case / cfg.name)
        (case / "README.md").write_text(f"# {capture_id}\n\n目标：方位 {azimuth}°，俯仰 {elevation}°，距离 {range_m} m。\n\n把真实 `capture.bin`、DCA1000 配置、TI 校准文件和姿态记录放入本目录后，先运行 V0.4.133 准入门。当前是模板，不代表已采集。\n", encoding="utf-8")
    result = {"status": "completed_batch_known_angle_manifest_templates", "case_count": len(CASES), "cfg": str(cfg.resolve()), "cfg_sha256": build_manifest(cfg)["capture"]["cfg_sha256"], "real_capture_present": False, "evidence_status": "template_awaiting_capture"}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    (output / "output_analysis.md").write_text("# V0.4.135 五工况 CFG 绑定 manifest 模板\n\n本阶段为首批五个已知角工况生成独立目录、manifest.json、同一份 CFG 副本和 README。目标角度/距离已填入，capture.bin、通道顺序、TI 校准和姿态记录仍待真实采集。\n\n只有完成 V0.4.133 准入检查后，才允许进入 IQ 解码。\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--cfg", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.cfg.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
