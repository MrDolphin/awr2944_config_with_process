"""Generate a reproducible multi-angle known-target capture plan."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def build_plan(azimuths=(-30, -15, 0, 15, 30), elevations=(-10, 0, 10), ranges=(10, 20)) -> list[dict]:
    rows = []
    index = 1
    for distance in ranges:
        for elevation in elevations:
            for azimuth in azimuths:
                rows.append({"case_id": f"ka{index:03d}_az{azimuth:+03d}_el{elevation:+03d}_r{distance:02d}", "sequence": index, "azimuth_deg": azimuth, "elevation_deg": elevation, "range_m": distance, "radial_velocity_mps": 0.0, "target_type": "corner_reflector", "required_files": "manifest.json,capture.bin,profile.cfg", "status": "planned"})
                index += 1
    return rows


def run(output: Path) -> dict:
    rows = build_plan()
    output.mkdir(parents=True, exist_ok=True)
    with (output / "known_angle_capture_plan.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summary = {"status": "capture_plan_generated", "case_count": len(rows), "azimuths_deg": sorted({row["azimuth_deg"] for row in rows}), "elevations_deg": sorted({row["elevation_deg"] for row in rows}), "ranges_m": sorted({row["range_m"] for row in rows}), "target_type": "corner_reflector", "hardware_aoa_validated": False, "execution_status": "not_started"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.93 已知角多姿态采集计划", "", f"本计划包含 {len(rows)} 个采集工况：方位 {summary['azimuths_deg']}°，俯仰 {summary['elevations_deg']}°，距离 {summary['ranges_m']} m。", "", "## 设计目的", "", "用多方位、多俯仰和两个距离验证坐标镜像、通道顺序和阵列孔径误差是否稳定，避免用单个 0° 点误判坐标变换。", "", "## 每个工况必须保存", "", "- `manifest.json`：复制 V0.4.90 模板并填写目标真值；", "- `capture.bin` 或 HDF5：DCA1000 原始 IQ；", "- `profile.cfg`：实际运行配置快照；", "- 如已完成校准，保存校准矩阵和来源；", "- 记录目标测量方式和船体/雷达姿态。", "", "## 建议执行顺序", "", "1. 先执行距离 10 m、俯仰 0° 的五个方位点；", "2. 再执行方位 0° 的三个俯仰点；", "3. 最后执行距离 20 m 的重复点；", "4. 每个点至少保存一段完整 TDM 帧，不要只保存 CFAR 点；", "5. 每完成一组就运行 V0.4.92 检查结果。", "", "## 边界", "", "这是采集计划，不是测量结果。当前 `hardware_aoa_validated=false`，坐标冻结仍需真实数据和机械基准共同支持。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
