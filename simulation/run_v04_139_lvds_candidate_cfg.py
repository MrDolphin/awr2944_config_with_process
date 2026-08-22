"""Create a non-authoritative LVDS-enabled CFG candidate copy."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


LVDS_LINE = "lvdsStreamCfg -1 0 1 0"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(source: Path, output: Path) -> dict:
    text = source.read_text(encoding="utf-8", errors="strict")
    if "lvdsStreamCfg" in text:
        candidate_text = text
        inserted = False
    else:
        lines = text.splitlines()
        index = next((i for i, line in enumerate(lines) if line.strip() == "sensorStart"), len(lines))
        lines.insert(index, LVDS_LINE)
        candidate_text = "\n".join(lines) + ("\n" if text.endswith("\n") else "")
        inserted = True
    output.mkdir(parents=True, exist_ok=True)
    candidate = output / source.name
    candidate.write_text(candidate_text, encoding="utf-8")
    result = {"status": "completed_lvds_candidate_cfg", "source_cfg": str(source.resolve()), "candidate_cfg": str(candidate.resolve()), "source_sha256": _sha256(source), "candidate_sha256": _sha256(candidate), "lvds_line": LVDS_LINE, "inserted": inserted, "same_profile_and_frame": True, "authoritative_hardware_cfg": False, "ti_board_validation": False, "next_action": "review against TI SDK/board version, then perform short operator-approved center-angle capture"}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    (output / "output_analysis.md").write_text("# V0.4.139 LVDS 候选 CFG\n\n本阶段只在输出目录生成候选 CFG 副本，在当前 656-sample 3D CFG 原文中插入 `lvdsStreamCfg -1 0 1 0`，不修改正式 CFG。\n\n## 重要边界\n\n该命令来自旧 DCA1000 参考流程，当前候选只保留了原 profileCfg/frameCfg/TDM 配置，尚未通过 TI SDK、当前固件版本或真实板卡验证。它可以用于下一步 operator-reviewed 短采集准备，不能直接宣称 DCA1000 一定收到 ADC 数据。\n\n## 验证\n\n先对候选 CFG 运行 V0.4.137；随后由操作者确认板卡、DCA1000 网络和固件版本，再决定是否发送。\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--source", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.source.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
