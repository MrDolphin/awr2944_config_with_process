"""Compare the project 3D CFG with the legacy LVDS-enabled reference CFG."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256(); digest.update(path.read_bytes()); return digest.hexdigest()


def _commands(path: Path) -> dict[str, str]:
    result = {}
    for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        parts = raw.split()
        if parts and not parts[0].startswith("%"):
            result[parts[0]] = " ".join(parts[1:])
    return result


def run(target_cfg: Path, reference_cfg: Path, output: Path) -> dict:
    target = _commands(target_cfg); reference = _commands(reference_cfg); names = sorted(set(target) | set(reference))
    rows = [{"command": name, "target_value": target.get(name, ""), "reference_value": reference.get(name, ""), "same": target.get(name, "") == reference.get(name, "")} for name in names]
    output.mkdir(parents=True, exist_ok=True)
    with (output / "cfg_command_comparison.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    result = {"status": "completed_cfg_lvds_comparison", "target_cfg": str(target_cfg.resolve()), "reference_cfg": str(reference_cfg.resolve()), "target_sha256": _sha256(target_cfg), "reference_sha256": _sha256(reference_cfg), "target_lvds_present": "lvdsStreamCfg" in target, "reference_lvds_present": "lvdsStreamCfg" in reference, "target_profile": target.get("profileCfg", ""), "reference_profile": reference.get("profileCfg", ""), "target_frame": target.get("frameCfg", ""), "reference_frame": reference.get("frameCfg", ""), "safe_to_replace_target_cfg": False}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.138 CFG 与 LVDS 参考配置对账", "", f"目标 CFG：`{target_cfg}`", f"参考 CFG：`{reference_cfg}`", "", "## 关键结论", "", f"- 目标 CFG 是否含 `lvdsStreamCfg`：`{result['target_lvds_present']}`。", f"- 参考 CFG 是否含 `lvdsStreamCfg`：`{result['reference_lvds_present']}`。", f"- 目标 profileCfg：`{result['target_profile']}`。", f"- 参考 profileCfg：`{result['reference_profile']}`。", f"- 目标 frameCfg：`{result['target_frame']}`。", f"- 参考 frameCfg：`{result['reference_frame']}`。", "", "## 不能直接复制参考 CFG", "", "参考 CFG 的 LVDS 命令证明旧流程曾启用 DCA1000 输出，但它的采样点数、frame/chirp 组织、天线几何和其他命令与当前 3D CFG 不同。直接整文件替换会破坏当前 AWR2944P 3D AoA 的维度/映射合同。", "", "## 建议动作", "", "先由 TI SDK/板卡实际版本确认适用于当前 AWR2944P profile 的 LVDS 参数，再在副本 CFG 中加入并重新计算 SHA-256；完成预检后先做短时中心角度采集，不要覆盖原始 CFG。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--target", type=Path, required=True); parser.add_argument("--reference", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.target.resolve(), args.reference.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
