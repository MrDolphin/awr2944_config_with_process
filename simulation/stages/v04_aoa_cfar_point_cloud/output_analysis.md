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

V0.4.6 进一步生成 `simulation/hardware/awr2944pev/antgeometry_mapping.csv`，把 SDK 的
`Tx0Rx0...Tx3Rx3` 顺序和 CFG 的 row/column 索引展开。该表解决“通道顺序”的追踪问题，
但不替代 PCB 电气相位中心和 DCA1000 通道验证。

V0.4.7 已加入 DCA1000 原始 IQ 接口。当前没有实测 `.bin/.raw` 文件，解码器只验证了合成
int16 交织模式，所有真实输出必须保留 `channel_order_verified=false`，直到用已知信号和
`antGeometryCfg` 完成通道顺序验证。

V0.4.9 已完成合成 IQ 端到端闭环：编码、DCA1000 解码、4TX TDM 重排和 AoA 恢复误差均有记录；
该闭环的 `channel_order_verified` 仍为 `synthetic_only`，不代表真实硬件抓包已验证。

V0.4.8 另外验证了当前 4TX TDM `chirpCfg` 序列到虚拟通道张量的重排：输出为
`(frame, sample, rx, tx)`，但真实 LVDS lane 顺序、I/Q 符号和 TX 归属仍待 DCA1000 抓包确认。

## V0.4.10 通道排列故障指纹

在真值方位 20°、俯仰 10° 的同一组合上，分别注入 RX 反序、TX 反序、TX0/TX1 交换和 I/Q 共轭。结果是：RX 反序只使俯仰变为 -10°；TX 反序只使方位变为 -20°；部分 TX 交换产生约 -4.12° 方位偏差；I/Q 共轭使方位和俯仰同时翻转。详细表格和真实数据应用步骤见 `docs/reports/v04_channel_diagnostics.md`。

这些是当前坐标约定下的合成误差指纹，不是实测 EVM 结论。真实 `.bin` 到来后，应对同一数据运行五种排列并与角反射器真值比较；在此之前仍保留 `channel_order_verified=synthetic_only`。
