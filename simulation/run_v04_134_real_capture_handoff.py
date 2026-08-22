"""Generate the first-five-case real-capture handoff package."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


CASES = [
    ("ka001_az-30_el-10_r10", -30, -10, 10),
    ("ka002_az-15_el-10_r10", -15, -10, 10),
    ("ka003_az+00_el-10_r10", 0, -10, 10),
    ("ka004_az+15_el-10_r10", 15, -10, 10),
    ("ka005_az+30_el-10_r10", 30, -10, 10),
]


def run(output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for capture_id, azimuth, elevation, distance in CASES:
        rows.append({"capture_id": capture_id, "azimuth_deg": azimuth, "elevation_deg": elevation, "range_m": distance, "directory": capture_id, "raw_file": "capture.bin", "cfg_file": "profile_3d_3Azim_1ElevTx_awr2944P.cfg", "dca1000_config_file": "dca1000_config.json", "calibration_file": "ti_calibration.json", "imu_pose_file": "imu_pose.csv", "required_before_decode": "capture.bin + manifest.json + CFG hash + DCA1000 config + calibration status + pose reference"})
    with (output / "first_five_capture_plan.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    checklist = [
        "Stop motion and set the radar/target geometry before capture.",
        "Record the same CFG used by the radar and compute its SHA-256.",
        "Save the DCA1000 capture configuration and raw capture.bin without CFAR.",
        "Record target azimuth, elevation and range from survey/total station.",
        "Record installation height, boresight, ship heading and IMU pose reference.",
        "Run TI calibration and save the resulting RX channel phase/gain data.",
        "Do not change channel order or reshape dimensions by hand after capture.",
        "Run V0.4.133 readiness gate before V0.4.112 decode.",
    ]
    (output / "REAL_CAPTURE_HANDOFF.md").write_text("# V0.4.134 首批真实 DCA1000 采集交接包\n\n## 首批工况\n\n" + "\n".join(f"- `{row['capture_id']}`：方位 {row['azimuth_deg']}°，俯仰 {row['elevation_deg']}°，距离 {row['range_m']} m" for row in rows) + "\n\n## 采集检查清单\n\n" + "\n".join(f"{index}. {item}" for index, item in enumerate(checklist, 1)) + "\n\n## 目录约定\n\n每个工况目录至少包含：`capture.bin`、`manifest.json`、CFG、DCA1000 配置、TI 校准文件和姿态引用。文件名和目录名应与 `first_five_capture_plan.csv` 一致。\n\n## 验收命令\n\n```powershell\npython -m simulation.run_v04_133_capture_readiness_gate `\n  --root <真实采集根目录> `\n  --output simulation/hardware/awr2944pev/v04_133_capture_readiness_gate_real\n\npython -m simulation.run_v04_112_manifest_iq_decode `\n  --manifest <工况目录>\\manifest.json `\n  --output <工况目录>\\decoded\n```\n\n准入门输出 `ready_count` 必须大于 0，随后才进入 IQ 解码。当前模板不包含真实 capture.bin，也不代表硬件已验证。\n", encoding="utf-8")
    result = {"status": "completed_real_capture_handoff", "case_count": len(rows), "cases": rows, "real_capture_present": False, "next_gate": "run V0.4.133 after placing capture.bin and completed manifests"}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
