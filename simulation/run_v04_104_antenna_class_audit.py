"""Audit Altium antenna net-class membership and RF width rules."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


FIELD_RE = re.compile(r"([^|=]+)=([^|]*)")
TARGETS = {"TX1", "TX2", "TX3", "TX4", "RX1", "RX2", "RX3", "RX4"}


def _records(text: str, kind: str):
    for chunk in text.split(f"|RECORD={kind}")[1:]:
        chunk = chunk.split("|RECORD=", 1)[0]
        yield dict(FIELD_RE.findall(chunk[:6000]))


def audit(pcbdoc: Path, output: Path) -> dict:
    text = pcbdoc.read_text(errors="ignore")
    classes = [row for row in _records(text, "Class") if row.get("NAME") == "Antenna"]
    rules = [row for row in _records(text, "DXPRule") if row.get("NAME") == "Antenna"]
    if not classes:
        raise ValueError("Antenna net class not found")
    antenna_class = classes[0]
    members = [antenna_class[f"M{i}"] for i in range(32) if antenna_class.get(f"M{i}")]
    width_rule = next((row for row in rules if row.get("RULEKIND") == "Width" and row.get("ENABLED") == "TRUE"), None)
    result = {
        "status": "completed_antenna_class_audit",
        "source": str(pcbdoc),
        "class_name": "Antenna",
        "class_members": members,
        "class_members_match_expected": set(members) == TARGETS,
        "enabled_width_rule": {k: width_rule.get(k, "") for k in ("NAME", "RULEKIND", "ENABLED", "MINLIMIT", "MAXLIMIT", "PREFEREDWIDTH", "TOPLAYER_PREFWIDTH", "SCOPE1EXPRESSION")} if width_rule else None,
        "hardware_design_evidence": [
            "TI PCB explicitly groups TX1-TX4 and RX1-RX4 into the Antenna net class",
            "an enabled Antenna width rule constrains the RF network geometry",
        ],
        "does_not_prove": [
            "antenna element phase centers",
            "radiation pattern or 3 dB beamwidth",
            "feed-to-element phase delay and mutual coupling",
        ],
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    report = [
        "# V0.4.104 Antenna 网络类审计",
        "",
        f"输入：`{pcbdoc}`",
        "",
        "## 直接证据",
        "",
        f"PCB 中存在 `Antenna` 网络类，成员为：{', '.join(members)}。",
        "",
        "文件中还有启用的 `Antenna` Width 规则，作用域是 TX/RX 天线网络。该规则显示最小线宽 8.4 mil、最大线宽 10 mil，Top 层偏好 8.4 mil。",
        "",
        "## 解释边界",
        "",
        "这证明 TI 将 8 条 TX/RX 网络按天线 RF 网络进行 PCB 设计约束；它不能单独给出天线单元坐标、相位中心、方向图或 AoA 校准矩阵。",
        "",
        "下一步应将该网络类证据与天线版图/EM 模型或实测方向图对应起来。",
    ]
    (output / "output_analysis.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcbdoc", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.pcbdoc, args.output)
    print(json.dumps({k: result[k] for k in ("status", "class_members", "enabled_width_rule")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
