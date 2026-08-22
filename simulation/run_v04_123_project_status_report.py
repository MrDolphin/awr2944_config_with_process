"""Build a reproducible leadership-facing status report from staged outputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def _json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def run(project_root: Path, output: Path) -> dict:
    hardware = project_root / "simulation" / "hardware" / "awr2944pev"
    v114 = _json(hardware / "v04_114_capture_completeness_v111" / "summary.json")
    v115 = _json(hardware / "v04_115_synthetic_cfg_decode_regression" / "summary.json")
    v118 = _json(hardware / "v04_118_dynamic_beam_coverage" / "summary.json")
    v120 = _json(hardware / "v04_120_masked_microfacet_clutter" / "summary.json")
    v121 = _json(hardware / "v04_121_cfar_physics_join" / "summary.json")
    v122 = _json(hardware / "v04_122_detection_physics_join" / "summary.json")
    output.mkdir(parents=True, exist_ok=True)
    result = {"status": "completed_project_status_report", "evidence": {"v04_114_capture_ready": v114["ready_for_v0112_count"], "v04_114_planned": v114["planned_count"], "v04_115_iq_shape": v115["decoded_iq_shape"], "v04_118_case_count": v118["case_count"], "v04_118_frame_count": v118["frame_count"], "v04_120_row_count": v120["row_count"], "v04_121_rows": v121["row_count"], "v04_122_rows": v122["row_count"]}, "hardware_aoa_validated": False, "real_dca_iq_available": False, "next_gate": "collect first real DCA1000 ADC IQ in ka001-ka005 and rerun V0.114/V0.112"}
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# AWR2944P 船载海杂波仿真项目阶段总报告（V0.4.123）", "", "## 一句话状态", "", "仿真链路已经从动态海面高度推进到几何波束覆盖、三维海面点云、微元散射/Doppler 代理、合成 Range-Doppler、CFAR 和目标检测边界；真实 DCA1000 ADC IQ 尚未到位，因此当前还不能宣称实板 AoA、实船虚警率或探测距离。", "", "## 已完成并有文件证据的工作", "", "1. **海况输入**：五组海况覆盖 Hs=0、0.05、0.30、0.85、1.00 m，三级海况上限按 1 m 控制。", "2. **安装几何**：雷达高度 1 m、安装俯仰 5° 的中心波束理想海面交点约 11.430 m；3 dB 几何边界约 7.115–28.636 m。", "3. **动态波束覆盖**：V0.118 处理 5 组海况、205 帧，逐帧计算方位、俯仰、斜距和 3/6 dB 掩膜。", "4. **三维点云**：V0.119 输出海面 `x/y/z`、斜距、方位、俯仰和波束掩膜；它是几何点云，不是雷达检测点云。", "5. **海杂波物理代理**：V0.120 输出法向、入射余弦、散射权重、径向速度和 Doppler 代理；ss3_upper 3 dB Doppler 代理约 −86.85～85.71 Hz。", "6. **CFAR/目标边界**：V0.121、V0.122 将合成 CFAR、目标注入和海况代理联表，用于区分海况效应和算法参数效应。", "7. **IQ 链路**：V0.115 已用当前 CFG 的全尺寸合成 IQ 验证 `[64,656,4]` 解码和 HDF5 输出。", "", "## 当前硬件数据状态", "", f"- 30 个已知角工况计划；V0.114 可进入 V0.112 的案例：`{v114['ready_for_v0112_count']}/{v114['planned_count']}`。", "- 真实 DCA1000 ADC IQ：未发现。旧 record BIN 已审计为 UART 点云。", "- 合成解码回归：实际 IQ 形状 `" + str(v115["decoded_iq_shape"]) + "`，但 `synthetic_only=true`。", "- 通道顺序验证：未完成。", "- TI 校准矩阵：未完成。", "- 真实 TX/RX 相位中心和方向图：未完成。", "- 硬件 AoA 验证：`false`。", "", "## 领导汇报时可以说什么", "", "> 当前已完成从海况参数到三维海面杂波候选、Doppler、CFAR 和目标检测边界的仿真链路，能够比较五级海况和安装角度的影响。当前结果用于方法验证和工况筛选，尚未替代实测硬件数据。下一阶段以五个已知角工况采集 DCA1000 原始 IQ，验证通道顺序、TI 校准和阵元相位中心，再把实测结果与仿真逐项对齐。", "", "## 不能宣称的内容", "", "- 不能把 3 dB/6 dB 波束宽度当成芯片角度极限；", "- 不能把几何点云当成真实杂波检测点云；", "- 不能把合成 CFAR 点数当成实船虚警率；", "- 不能把目标注入检测概率当成实际探测距离；", "- 不能把 PCB 走线端点当成真实天线相位中心。", "", "## 下一步验收门", "", "1. 采集 `ka001–ka005` 的真实 `capture.bin`；2. 保存同一份 CFG、DCA1000 配置和 SHA-256；3. 运行 V0.114 完整性门；4. 运行 V0.112 解码并检查 IQ 形状/帧边界；5. 用已知角目标确认通道顺序；6. 导入 TI 校准矩阵；7. 再比较理想阵列、PCB 候选和实测相位中心模型。", "", "## 证据路径", "", "本报告的原始证据保存在同一硬件结果目录下的 `v04_114`、`v04_115`、`v04_118`、`v04_120`、`v04_121` 和 `v04_122` 子目录；每个版本都有独立 CSV/JSON/PNG 和 `output_analysis.md`。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--project-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.project_root.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
