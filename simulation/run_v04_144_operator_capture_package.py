"""Build a review-only DCA1000 operator package; never start hardware commands."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run(cfg: Path, dca_reference: Path, output: Path) -> dict:
    cfg = cfg.resolve(); dca_reference = dca_reference.resolve(); output.mkdir(parents=True, exist_ok=True)
    dca = json.loads(dca_reference.read_text(encoding="utf-8"))
    # Do not copy internal network identifiers into the repository or remote.
    sanitized = copy.deepcopy(dca)
    config = sanitized.get("DCA1000Config", {})
    for section in (config.get("ethernetConfig", {}), config.get("ethernetConfigUpdate", {})):
        for key in ("DCA1000IPAddress", "systemIPAddress", "DCA1000MACAddress"):
            if key in section:
                section[key] = "<operator_verify>"
    (output / "candidate_cfg_snapshot.cfg").write_bytes(cfg.read_bytes())
    (output / "dca1000_reference_sanitized.json").write_text(json.dumps(sanitized, ensure_ascii=False, indent=2), encoding="utf-8")
    result = {
        "status": "operator_review_package_created",
        "candidate_cfg": str(cfg),
        "candidate_cfg_sha256": sha256(cfg),
        "dca_reference": str(dca_reference),
        "dca_reference_sha256": sha256(dca_reference),
        "hardware_commands_executed": False,
        "authoritative_dca_config": False,
        "next_action": "operator_verify_network_and_cli_help_then_capture_center_case",
    }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    ip = sanitized.get("DCA1000Config", {}).get("ethernetConfig", {})
    capture = dca.get("DCA1000Config", {}).get("captureConfig", {})
    lines = [
        "# V0.4.144 操作员采集包（只读准备）", "",
        "本目录只保存候选 CFG、DCA1000 历史参考配置和人工操作清单；脚本没有执行任何雷达、DCA1000 或网络命令。", "",
        "## 文件证据", "",
        f"- 候选 CFG：`{cfg}`", f"- 候选 CFG SHA-256：`{result['candidate_cfg_sha256']}`",
        f"- DCA 参考文件：`{dca_reference}`", f"- DCA 参考 SHA-256：`{result['dca_reference_sha256']}`", "",
        "## 从历史参考文件读取的参数（不是当前硬件确认值）", "",
        f"- DCA IP：`{ip.get('DCA1000IPAddress')}`；配置端口：`{ip.get('DCA1000ConfigPort')}`；数据端口：`{ip.get('DCA1000DataPort')}`",
        f"- 历史采集模式：`{sanitized.get('DCA1000Config', {}).get('dataTransferMode')}`；LVDS mode：`{sanitized.get('DCA1000Config', {}).get('lvdsMode')}`",
        f"- 历史文件前缀：`{capture.get('filePrefix')}`；历史帧数：`{capture.get('framesToCapture')}`", "",
        "## 人工执行顺序", "",
        "1. 确认 AWR2944P SDK/固件接受 V0.4.139 候选 CFG；确认不使用正式配置覆盖。",
        "2. 确认 PC 网卡、DCA1000 IP、MAC、配置端口和数据端口与现场实际值一致；历史 JSON 中的地址不能直接照搬。",
        "3. 当前已验证这套 CLI 使用 `-h`（不是 `--help`）：运行 `DCA1000EVM_CLI_Control.exe -h` 和 `DCA1000EVM_CLI_Record.exe -h`；再按本机 CLI 版本确认 JSON 参数顺序。",
        "4. 先布置中心角 10 m 角反射器，记录安装高度、方位/俯仰和 IMU 姿态。",
        "5. 手工执行 FPGA/configure、record/start、短时停止；不要在本项目脚本中自动下发。",
        "6. 把原始文件复制到 V0.4.140 的 `lv003_az+00_el-10_r10`，并记录文件 SHA-256。",
        "7. 运行 V0.4.141，再运行 V0.4.142；未通过准入时不得解码。", "",
        "## 证据边界", "",
        "本包不证明 DCA1000 已连接，也不证明历史网络参数仍适用。只有现场执行日志、capture.bin、CFG 快照、姿态记录和校准记录齐全后，才能进入真实 IQ/AoA 验证。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--cfg", type=Path, required=True); parser.add_argument("--dca-reference", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.cfg, args.dca_reference, args.output), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
