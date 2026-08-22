"""Perform a non-invasive preflight for the DCA1000 capture environment."""

from __future__ import annotations

import argparse
import importlib.util
import json
import tempfile
from pathlib import Path


def _module(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def run(cfg: Path, output: Path, dca_tool_root: Path | None = None) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    cfg_exists = cfg.is_file(); cfg_text = cfg.read_text(encoding="utf-8", errors="ignore") if cfg_exists else ""
    lvds_enabled = "lvdsStreamCfg" in cfg_text
    tool_root = dca_tool_root or Path(r"D:\hp-laptop\USV\awr2944_config_and_process_with_trace\mathlab\PostProc")
    checks = {
        "cfg_exists": cfg_exists,
        "lvds_stream_config_present": lvds_enabled,
        "dca1000_cli_control_present": (tool_root / "DCA1000EVM_CLI_Control.exe").is_file(),
        "dca1000_cli_record_present": (tool_root / "DCA1000EVM_CLI_Record.exe").is_file(),
        "python_serial_present": _module("serial"),
        "python_numpy_present": _module("numpy"),
        "python_h5py_present": _module("h5py"),
    }
    try:
        with tempfile.NamedTemporaryFile(dir=output, prefix="preflight_", delete=True):
            checks["output_writable"] = True
    except OSError:
        checks["output_writable"] = False
    result = {"status": "completed_capture_environment_preflight", "checks": checks, "hardware_connected": False, "capture_started": False, "ready_for_operator_capture": all(checks.values()), "next_action": "connect hardware and run the handoff command only after operator review"}
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# V0.4.137 DCA1000 采集环境预检", "", "本阶段只检查本地文件、Python 依赖和 CFG，不打开串口、不连接 DCA1000、不启动雷达。", "", "## 检查结果", ""]
    lines.extend(f"- `{key}`：`{'PASS' if value else 'FAIL'}`" for key, value in checks.items())
    lines += ["", "## 解释", "", "`lvdsStreamCfg` 缺失时，雷达 CFG 可能只输出 UART 点云，DCA1000 不一定收到 ADC 数据。CLI 工具存在只说明软件文件在本机，不代表网络、DCA1000 或雷达已经连通。", "", f"本次 `ready_for_operator_capture`：`{result['ready_for_operator_capture']}`。该状态不是硬件验证，只是采集前本地环境检查。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--cfg", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--dca-tool-root", type=Path); args = parser.parse_args(); print(json.dumps(run(args.cfg.resolve(), args.output.resolve(), args.dca_tool_root.resolve() if args.dca_tool_root else None), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
