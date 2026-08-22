"""Create a non-executing, operator-reviewable DCA1000 capture command plan."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def run(cfg: Path, dca_json: Path, output: Path) -> dict:
    cfg = cfg.resolve(); dca_json = dca_json.resolve(); output.mkdir(parents=True, exist_ok=True)
    plan = {
        "status": "dry_run_command_plan_created",
        "execution_performed": False,
        "hardware_commands_executed": False,
        "candidate_cfg": str(cfg),
        "dca_json": str(dca_json),
        "commands": [
            {"stage": "inspect", "command": 'DCA1000EVM_CLI_Control.exe -h', "requires_operator_review": False},
            {"stage": "inspect", "command": 'DCA1000EVM_CLI_Record.exe -h', "requires_operator_review": False},
            {"stage": "configure_fpga", "command": f'DCA1000EVM_CLI_Control.exe fpga <verified_cf.json>', "requires_operator_review": True},
            {"stage": "configure_record", "command": f'DCA1000EVM_CLI_Control.exe record <verified_cf.json>', "requires_operator_review": True},
            {"stage": "start_record", "command": f'DCA1000EVM_CLI_Control.exe start_record <verified_cf.json>', "requires_operator_review": True},
            {"stage": "capture_listener", "command": f'python tools/dca1000_capture.py --cfg "{cfg}" --duration 3 --no-control', "requires_operator_review": True},
            {"stage": "stop_record", "command": f'DCA1000EVM_CLI_Control.exe stop_record <verified_cf.json>', "requires_operator_review": True},
        ],
        "safety_notes": [
            "本文件只生成命令文本，不调用 subprocess，不连接网卡，不启动雷达或 DCA1000。",
            "尖括号中的 JSON 必须由操作员现场确认，不能直接使用历史网络参数。",
            "先确认 SDK/固件与候选 CFG 兼容，再执行 configure_fpga。",
            "采集完成后必须保存 capture.bin、CFG、姿态、日志并运行 V0.4.141/V0.4.142。",
        ],
        "next_action": "operator_review_then_manual_center_angle_capture",
    }
    (output / "command_plan.json").write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.146 DCA1000 中心角采集 dry-run 命令计划", "", "本阶段只生成命令文本，没有执行任何命令。", "", "## 命令顺序", ""]
    for item in plan["commands"]:
        lines.append(f"- `{item['stage']}`：`{item['command']}`" + ("（需现场审核）" if item["requires_operator_review"] else ""))
    lines += ["", "## 安全边界", ""] + [f"- {note}" for note in plan["safety_notes"]] + [""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return plan


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--cfg", type=Path, required=True); parser.add_argument("--dca-json", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.cfg, args.dca_json, args.output), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
