# AWR2944PEVM 来源追踪

## 本地资料

- `C:\Users\56461\Downloads\2944p资料\EVM User's Guide AWR2944EVM, AWR2944PEVM AWR2944AWR2944P Evaluation Module.pdf`
  - Figure 2-17：板载天线布局；Figure 2-18：虚拟天线阵列；Figure 2-19：方位方向图；Figure 2-20：俯仰方向图。
  - 第 20 页文字：峰值增益约 13 dBi；水平 3 dB 约 ±30°、6 dB 约 ±45°；俯仰 3 dB 约 ±3°、6 dB 约 ±5°。
- `C:\Users\56461\Downloads\2944p资料\Design Guide TIDEP-01027  High-End Corner Radar Reference Design.pdf`
  - 用于后续射频/结构参考，不直接作为本 CSV 的阵元坐标来源。
- `C:\Users\56461\Downloads\2944p资料\TI mmWave Radar sensor RF PCB Design, Manufacturing.pdf`
  - 用于后续 PCB 材料、走线和制造约束参考。

## 当前可信度边界

- `virtual_array_coordinates.csv` 的数值坐标暂留空：公开用户指南图示不足以证明完整的相位中心坐标表，需要 CAD/Gerber 或实测。
- 两个方向图 CSV 是基于官方波束宽度/曲线的粗粒度数字化占位，用于软件回归，不是完整测量方向图。
- `calibration_schema.json` 明确记录当前未进行角反射器校准。
- 项目 CFG 的原始命令和 FOV 快照见 `simulation/stages/v04_aoa_cfar_point_cloud/config/awr2944p_cfg_snapshot.json`。
- `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_ASCII.PcbDoc` 是 Altium ASCII PCB 数据；已提取 TX1～TX4/RX1～RX4 铜区 Region 的顶点包围盒和几何中心到 `pcb_antenna_regions.csv`。
- `cad_virtual_array_coordinates.csv` 用 TX/RX 铜区几何中心求和生成虚拟通道相对坐标；它是 CAD-derived 几何近似，不是电气相位中心，也不替代 HFSS/CST 或角反射器校准。

## 允许使用的用途

当前文件可用于数据追踪、仿真接口、方向图插值框架和误差分析。不能用于声称实板阵元相位中心、实测 AoA 精度或船载安装后的最终方向图。
