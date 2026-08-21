"""Scaffold per-case directories and manifests from the known-angle plan."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from simulation.run_v04_90_known_angle_capture_contract import template
from simulation.run_v04_93_known_angle_plan import build_plan


def run(output: Path, plan_csv: Path | None = None) -> dict:
    rows = build_plan()
    if plan_csv:
        with plan_csv.open(encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
    output.mkdir(parents=True, exist_ok=True)
    for row in rows:
        case_dir = output / row["case_id"]
        case_dir.mkdir(parents=True, exist_ok=True)
        manifest = template()
        manifest["capture_id"] = row["case_id"]
        manifest["evidence_status"] = "planned_awaiting_capture"
        manifest["target"].update({"type": row.get("target_type", "corner_reflector"), "azimuth_deg": float(row["azimuth_deg"]), "elevation_deg": float(row["elevation_deg"]), "range_m": float(row["range_m"]), "radial_velocity_mps": float(row.get("radial_velocity_mps", 0.0))})
        (case_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    summary = {"status": "capture_manifest_scaffold_generated", "case_count": len(rows), "root": str(output.resolve()), "planned_only": True, "hardware_aoa_validated": False, "next_action": "copy actual capture.bin and profile.cfg into each case directory, then fill hashes and metadata"}
    (output / "scaffold_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "README.md").write_text("# V0.4.95 采集目录骨架\n\n每个子目录已生成 `manifest.json`。当前所有 manifest 都是 `planned_awaiting_capture`，不代表已经采集。把真实 `capture.bin`/HDF5 和 `profile.cfg` 放入对应目录，再填写文件哈希、采集时间、姿态和通道/校准状态。\n\n完成后使用 V0.4.94 检查完整性，再使用 V0.4.92 批处理。\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--plan", type=Path); args = parser.parse_args(); print(json.dumps(run(args.output.resolve(), args.plan.resolve() if args.plan else None), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
