"""Create the 30-case known-angle capture scaffold bound to the current CFG."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path

from simulation.run_v04_93_known_angle_plan import build_plan
from simulation.run_v04_111_cfg_linked_manifest import build_manifest, parse_cfg


def run(cfg_path: Path, output: Path) -> dict:
    cfg_path = cfg_path.resolve()
    cfg = parse_cfg(cfg_path)
    rows = build_plan()
    output.mkdir(parents=True, exist_ok=True)
    with (output / "known_angle_capture_plan.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    for row in rows:
        case_dir = output / row["case_id"]
        case_dir.mkdir(parents=True, exist_ok=True)
        manifest = build_manifest(cfg_path, capture_id=row["case_id"])
        manifest["target"].update({"type": row["target_type"], "azimuth_deg": float(row["azimuth_deg"]), "elevation_deg": float(row["elevation_deg"]), "range_m": float(row["range_m"]), "radial_velocity_mps": float(row["radial_velocity_mps"])})
        manifest["evidence_status"] = "planned_awaiting_capture"
        (case_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
        shutil.copy2(cfg_path, case_dir / cfg_path.name)
        (case_dir / "README.md").write_text("# 待采集案例\n\n将真实 DCA1000 `capture.bin` 放在本目录，并填写 `manifest.json` 中的时间戳、姿态、wire order、通道顺序和校准字段。当前目录只包含计划和 CFG 快照，不代表已经采集。\n", encoding="utf-8")
    summary = {"status": "cfg_linked_capture_scaffold_generated", "case_count": len(rows), "root": str(output.resolve()), "cfg": str(cfg_path), "cfg_sha256": cfg["source_sha256"], "cfg_geometry": cfg, "planned_only": True, "hardware_aoa_validated": False, "next_action": "put capture.bin in each case and complete the manifest before V0.4.112"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.113 CFG 绑定的 30 工况已知角采集骨架", "", f"共 {len(rows)} 个计划工况；方位 `{sorted({row['azimuth_deg'] for row in rows})}°`，俯仰 `{sorted({row['elevation_deg'] for row in rows})}°`，距离 `{sorted({row['range_m'] for row in rows})} m`。", "", f"当前 CFG：`{cfg_path}`", f"CFG SHA-256：`{cfg['source_sha256']}`", f"ADC samples/chirp：`{cfg['adc_samples']}`；chirps/frame：`{cfg['chirps_per_frame']}`；TDM TX：`{cfg['tx_order_bit_index']}`。", "", "## 每个案例目录", "", "每个目录都包含与当前 CFG 同步的 `manifest.json`、CFG 快照和 README；真实采集后再放入 `capture.bin`。所有 manifest 初始状态均为 `planned_awaiting_capture`。", "", "## 采集顺序建议", "", "先完成距离 10 m、俯仰 0° 的五个方位点，再完成方位 0° 的三个俯仰点，最后完成距离 20 m 的重复点。每个案例必须保存完整 TDM 原始帧，不要只保存 CFAR 点云。", "", "## 证据边界", "", "这是采集骨架，不是测量结果。只有真实 ADC IQ、目标测量、姿态、通道顺序和 TI 校准证据齐全后，才能进入 V0.4.112 和 AoA 验证。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--cfg", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.cfg.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
