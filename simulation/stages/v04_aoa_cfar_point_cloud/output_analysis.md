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

## V0.4.11 PCB/CAD 资料审计

本阶段读取用户提供的 `sprr440a (1)` 和 `sprr441a` 目录，生成了带 SHA-256 的文件清单 `output/pcb_package_inventory.md`。审计结果表明：

- `PROC113D_ASCII.PcbDoc` 可自动识别 8 个 TX/RX RF 铜区；本次独立复核输出 `output/pcb_antenna_regions_latest.csv`，每个区域 72 个顶点，已换算为毫米坐标。
- `PROC113D_BRD.PcbDoc` 的文件头是 Altium OLE 二进制格式，当前没有直接把它解析成阵列坐标；需要 Altium 导出 ASCII、IPC-2581/ODB++ 或经过验证的 OLE 解析器。
- `PROC113D_BRD.step` 可用于板框和机械安装面，不足以单独提供 RF 电气相位中心。
- 装配图、层叠图、原理图用于确认朝向、层号、馈电关系和版本；BOM 只做版本追溯。

因此，当前 `cad_virtual_array_coordinates.csv` 仍是 PCB 铜区几何中心近似，不能当作真实相位中心。下一步应提取馈电点、板面坐标系和 `Tx0Rx0...Tx3Rx3` 映射，再用电磁仿真或角反射器标定升级坐标可信度。

## V0.4.12 PCB 坐标与 CFG 通道对齐

已将 16 个 PCB 虚拟通道按 `Tx0Rx0...Tx3Rx3` 顺序与 `antGeometryCfg` 的行/列坐标逐通道比较。77 GHz 下使用 `0.5λ` 方位间距和 `0.8λ` 俯仰间距，输出逐通道 `pcb_x/y_mm`、`cfg_x/y_mm` 和差值。该比较没有进行旋转、镜像、缩放或最佳配准，目的只是暴露坐标定义不一致，不能直接解释为 AoA 误差。

结果目录为 `output/v04_12_coordinate_mapping/`，详细报告见 `docs/reports/v04_coordinate_mapping.md`。当前结论仍是 `pcb_centroid_is_not_electrical_phase_center`；下一步需要确认板面法向、X/Y 方向、原点和馈电/相位中心。

本次数值还发现：X 方向 RMS 差约 0.0693 mm，和 0.5λ 网格基本一致；Y 方向 RMS 差约 3.1198 mm，且 CFG row=0/row=1 与 PCB 的两个 Y 位置呈交换关系。当前优先级应是确认 `antGeometryCfg` 的 row 语义、PCB 板面朝向和坐标镜像，而不是马上调整阵元间距。

## V0.4.13 坐标变换 AoA 筛查

已用 PCB 坐标生成相位、用 CFG 理想阵列估计角度，分别筛查 raw、X 镜像、Y 镜像和 180° 旋转。该设计能暴露坐标模型差异，避免“生成和估计使用同一错误坐标”造成假性零误差。结果目录为 `output/v04_13_coordinate_transform_scan/`，详细边界见 `docs/reports/v04_coordinate_transform_scan.md`。

使用 CFG 展开的阵列坐标后，raw 的方位/俯仰 RMSE 为 38.2415°/21.8899°，Y 镜像为 38.2415°/22.4927°，X 镜像和 180° 旋转的方位 RMSE 更大。四种简单变换均未达到可用于真实 AoA 的程度，因此不能把某一种变换直接冻结为 EVM 阵列坐标。

## V0.4.14 RF 网络馈电候选

已从 ASCII PCB 的 8 个 RF 网络提取 Pad 和 Track 端点，单独保存为 `output/v04_14_rf_port_extraction/`。Pad 坐标可作为芯片端/器件端连接参考，Track 端点范围可用于追踪走线朝向；二者都没有被标记为天线相位中心。下一步需要建立 Track/Arc/Via 连通图，并结合层叠与电磁仿真或角反射器校准。

## V0.4.15 RF 网络几何连通图

已对 Pad、Track、Arc、Region 边界进行 1 mil 容差的几何连通分析，输出 `output/v04_15_rf_connectivity/rf_connectivity_summary.csv`。连通标志只表示 PCB ASCII 几何对象可连接，最短路径只是几何长度，不作为 77 GHz 相位延迟或相位中心输入。下一步需要层叠介质参数、RF 走线层信息和电磁/实测校准。

## V0.4.16 PCB 层叠与 RF 对象层

已从 ASCII PCB 的 `V9_STACK_LAYER*` 字段提取层叠参数，并统计 8 个 TX/RX 网络的 Track、Arc、Pad、Region 层。当前 RF 对象全部位于 TOP 层；层叠中可见 RO3003（约 εr=3.000、5 mil）和 FR-4 High Tg（约 εr=4.040、5/10 mil）等字段。结果见 `output/v04_16_pcb_layer_stack/`。这些是电磁建模输入，不是已完成的相位补偿。

## V0.4.17 走线相位敏感性

已将 TX/RX 几何路径长度接入可配置 `ε_eff` 的相对相位扰动模型，扫描 `ε_eff=1、2.5、3.0、4.04`。结果见 `output/v04_17_phase_sensitivity/`；该模型只用于敏感性分析，不代表有效介电常数或真实 EVM 相位校准。

## V0.4.18 复数通道校准接口

已建立 4×4 `amplitude`/`phase_deg` JSON 数据契约，并用可重复的合成幅相误差验证无校准、理想逆补偿和错误补偿。结果见 `output/v04_18_calibration_scan/`；当前仍标记为合成校准，不是 TI EVM 实测校准。

本次无校准方位/俯仰 RMSE 为 39.0193°/20.7787°，理想逆补偿恢复到 38.3208°/22.6125° 的合成基线。该结果证明了校准接口方向，但没有消除 PCB/CFG 坐标模型误差。

当前输出中 `ε_eff=1` 的方位/俯仰 RMSE 为 38.3208°/22.6125°，`ε_eff=4.04` 为 38.0000°/23.6879°。变化量小于当前整体坐标模型误差，说明现在不能靠调整 ε_eff 修复 AoA；必须先解决相位中心、通道坐标和真实校准问题。
