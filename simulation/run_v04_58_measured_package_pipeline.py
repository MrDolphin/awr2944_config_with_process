"""Run the measured-package preflight and dispatch available HDF5 scenes."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from simulation.run_v04_53_hdf5_channel_order_validation import validate
from simulation.run_v04_56_measured_aoa_package import check_manifest


def run(manifest_path: Path, output: Path, mapping: Path | None = None) -> dict:
    preflight = check_manifest(manifest_path); manifest = json.loads(manifest_path.read_text(encoding="utf-8")); root = manifest_path.parent; output.mkdir(parents=True, exist_ok=True); rows = []; hdf5_summaries = []
    for scene in manifest.get("scenes", []):
        capture = root / scene["capture"] if scene.get("capture") else None; scene_output = output / scene.get("scene_id", "unnamed")
        row = {"scene_id": scene.get("scene_id", "<missing>"), "capture": str(capture) if capture else "", "capture_exists": bool(capture and capture.is_file()), "processing_status": "missing_capture", "source_is_hardware_measurement": False, "channel_order_verified": False, "best_azimuth_error_deg": None, "best_elevation_error_deg": None}
        if capture and capture.is_file() and capture.suffix.lower() in {".h5", ".hdf5"}:
            try:
                summary = validate(capture, mapping, scene_output); best = summary["best_candidates"][0]; row.update({"processing_status": "hdf5_aoa_validated_synthetic_or_measured", "source_is_hardware_measurement": summary["source_is_hardware_measurement"], "channel_order_verified": summary["channel_order_verified"], "best_azimuth_error_deg": best["azimuth_error_deg"], "best_elevation_error_deg": best["elevation_error_deg"]}); hdf5_summaries.append(summary)
            except Exception as exc:
                row["processing_status"] = f"hdf5_error:{type(exc).__name__}"
        elif capture and capture.is_file() and capture.suffix.lower() == ".bin":
            row["processing_status"] = "bin_present_requires_cfg_linked_v051"
        rows.append(row)
    with (output / "scene_status.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["scene_id"]); writer.writeheader(); writer.writerows(rows)
    summary = {"status": "completed_measured_package_pipeline", "manifest": str(manifest_path.resolve()), "preflight": preflight, "scene_count": len(rows), "hdf5_processed_count": len(hdf5_summaries), "hardware_ready": bool(preflight["measured_aoa_ready"] and hdf5_summaries and all(item["source_is_hardware_measurement"] and item["channel_order_verified"] for item in hdf5_summaries)), "scene_status": rows}
    (output / "pipeline_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.58 实测 AoA 验收包流水线", "", f"场景数：{len(rows)}；HDF5 已处理：{len(hdf5_summaries)}；真实硬件就绪：{'是' if summary['hardware_ready'] else '否'}。", "", "| 场景 | 文件 | 状态 | source hardware | channel verified |", "|---|---|---|---|---|"]
    lines.extend(f"| {row['scene_id']} | {Path(row['capture']).name if row['capture'] else ''} | {row['processing_status']} | {'是' if row['source_is_hardware_measurement'] else '否'} | {'是' if row['channel_order_verified'] else '否'} |" for row in rows)
    lines += ["", "## 解释", "", "`.h5` 场景会直接进入 V0.53 HDF5 通道顺序验证；`.bin` 场景先通过 CFG 联动 V0.51 解码，再重新运行本流水线。即使 AoA 数值回归通过，只要数据 provenance 不是实测或通道顺序未确认，hardware_ready 仍为否。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8"); return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--manifest", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--mapping", type=Path); args = parser.parse_args(); print(json.dumps(run(args.manifest.resolve(), args.output.resolve(), args.mapping.resolve() if args.mapping else None), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
