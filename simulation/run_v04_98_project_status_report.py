"""Generate a leadership-ready status report with explicit evidence boundaries."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def _read(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def run(root: Path, output: Path) -> dict:
    hardware = root / "simulation" / "hardware" / "awr2944pev"
    audit = _read(hardware / "v04_97_data_source_audit" / "summary.json")
    completeness = _read(hardware / "v04_94_capture_completeness" / "results" / "v04_95_scaffold" / "summary.json")
    plan = _read(hardware / "v04_93_known_angle_plan" / "summary.json")
    mechanical = _read(hardware / "v04_88_mechanical_frame" / "mechanical_frame.json")
    transform = _read(hardware / "v04_89_coordinate_transform_stress" / "summary.json")
    batch = _read(hardware / "v04_92_known_angle_batch" / "summary.json")
    report = {"status": "completed_project_status_report", "software_stages_completed": ["V0.1 flat sea geometry", "V0.2 dynamic sea states", "V0.3 complex echo", "V0.4 AoA/CFAR/point cloud", "V0.4.85 PCB candidate", "V0.4.88 mechanical frame", "V0.4.89 coordinate-transform screening", "V0.4.90-0.96 capture pipeline"], "mechanical_board_bbox_mm": mechanical.get("step_board", {}).get("bbox_mm"), "known_angle_transform_screen": {"best_synthetic_transform": transform.get("best_known_angle_transform"), "best_combined_rmse_deg": transform.get("best_known_angle_combined_rmse_deg")}, "capture_plan_count": plan.get("case_count", 0), "capture_coverage_fraction": completeness.get("coverage_fraction", 0.0), "iq_source_audit": audit.get("provenance_counts", {}), "known_angle_batch_status": batch.get("status", "not_run"), "hardware_aoa_validated": False, "current_gate": "real DCA1000 IQ + known-angle target + channel-order/TI calibration evidence"}
    output.mkdir(parents=True, exist_ok=True)
    (output / "summary.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# AWR2944P 船载海杂波与 AoA 仿真项目阶段汇报（V0.4.98）", "", "## 一句话结论", "", "软件仿真、PCB/CAD 候选阵列和真实采集处理链已经打通；当前尚未获得真实 DCA1000 原始 IQ，因此不能把合成海杂波或候选阵列结果报告为实板 AoA/探测性能。", "", "## 已完成工作", "", "| 模块 | 当前结论 | 证据 |", "|---|---|---|", "| 海况仿真 | 已覆盖平静、涟漪、正常、3级名义和3级上限（有效波高最高1 m） | V0.2 HDF5 与逐版本分析 |", "| 复数回波/距离-多普勒 | 软件链已通过合成回放测试 | V0.3/V0.4 回归测试 |", "| PCB/CAD | 已提取 RF 网络候选和 STEP 机械包围盒 | V0.4.85/V0.4.88 |", "| 坐标语义 | 四种镜像/旋转候选已完成合成已知角筛查 | V0.4.89 |", "| 实测入口 | manifest、哈希、完整性、单点和批处理工具已完成 | V0.4.90～V0.4.96 |", "| 数据审计 | 当前发现的 27 个 IQ 类文件全部为合成/回放 | V0.4.97 |", "", "## 可量化当前状态", "", f"- PCB STEP 包围盒：`{report['mechanical_board_bbox_mm']}`。", f"- 计划已知角工况：`{report['capture_plan_count']}` 个。", f"- 当前基础采集覆盖率：`{report['capture_coverage_fraction']:.1%}`。", f"- 合成已知角筛查最小候选：`{report['known_angle_transform_screen']['best_synthetic_transform']}`，综合 RMSE：`{report['known_angle_transform_screen']['best_combined_rmse_deg']}`°。该结果不是实测坐标结论。", f"- IQ 来源审计：`{report['iq_source_audit']}`。", "", "## 需要领导关注的边界", "", "1. 当前结果可以支撑软件链路、指标定义和海况敏感性分析。", "2. 当前结果不能支撑真实探测距离、虚警率、实板 AoA 精度或海上作战性能结论。", "3. 下一阶段关键输入是带目标方位/俯仰真值的 DCA1000 原始 IQ，以及 CFG、TDM 顺序、通道验证和 TI 校准证据。", "", "## 下一步验收门", "", "- 至少完成 V0.93 计划中的一组多方位角反射器采集；", "- V0.94 基础文件覆盖率达到 100%；", "- V0.92 批处理成功并输出四候选误差；", "- 通道顺序和 TI 校准状态可追溯；", "- 通过多个角度/距离重复测试后，才冻结 PCB→雷达坐标。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
