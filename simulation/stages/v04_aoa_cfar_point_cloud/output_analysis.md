# V0.4.3 硬件资料包输出分析

本阶段输出 `simulation/hardware/awr2944pev/`，将 TI AWR2944PEVM 用户指南和项目 CFG
转换为可追溯的仿真输入接口。

## 如何分析这些文件

- `antenna_geometry.yaml`：先看来源和可信度状态，确认当前不是实测坐标。
- `virtual_array_coordinates.csv`：每行是一个 TX/RX 虚拟通道；坐标为空表示公开图纸尚未给出可证明的数值相位中心，不能自行补成理想半波长阵列。
- `antenna_pattern_azimuth.csv` 与 `antenna_pattern_elevation.csv`：横轴是角度，纵轴是相对增益；当前是官方曲线的粗粒度数字化占位，只适合方向图插值框架和回归。
- `calibration_schema.json`：检查 `calibration_status`、`range_bias_m` 和通道补偿数组；当前 `not_measured` 表示没有角反射器校准。
- `source_traceability.md`：记录 PDF、页码、参数来源和禁止过度解释的边界。

## 当前结论

TI 官方资料确认 AWR2944PEVM 有 4RX/4TX 板载蚀刻天线、二维虚拟阵列、约 13 dBi 峰值增益，
并给出约 ±30°/±3° 的 3 dB 方位/俯仰波束范围。但资料图示不足以直接生成完整的相位中心坐标表，
所以本阶段没有伪造精确坐标。下一步需要 EVM CAD/Gerber 或角反射器实测；在此之前，V0.4 AoA 结果
只能标记为图纸推导/未校准结果。
