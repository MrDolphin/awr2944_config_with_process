# V0.4.28 AWR2944P EVM PCB/CAD 资料审计

## 输入资料

本阶段读取用户提供的两个资料根目录：

```text
C:\Users\56461\Downloads\2944p资料\sprr440a (1)
C:\Users\56461\Downloads\2944p资料\sprr441a
```

其中 `sprr441a/SPRR441/PROC113D_ASCII.PcbDoc` 可按 Altium ASCII 记录直接解析；同时保留 STEP、BOM、原理图、层叠 PDF 和工程文件的 SHA-256 清单。

## 已提取结果

| 项目 | 数量/状态 |
|---|---:|
| `Board` 记录 | 27 |
| `Component` 记录 | 734 |
| `Net` 记录 | 514 |
| `Pad` 记录 | 2407 |
| `Track` 记录 | 63758 |
| `Via` 记录 | 1913 |
| 层叠记录 | 21 |
| 名称疑似 TX/RX/RF 的网络 | 58 |
| 已确认天线相位中心 | 否 |

候选网络进一步分为：

```text
radar_channel_name_candidate       16
digital_interface_name_candidate   38
other_rf_name_candidate              4
```

这里的 `radar_channel_name_candidate` 只表示网络名类似 `TX0_P`、`RX1`，不能直接证明它是天线馈电点；`LVDS`、`UART`、`CSI`、`CAN` 等名称已单独标为数字接口候选，避免误并入毫米波天线阵列。

## 层叠初步结果

ASCII PCB 中可以读取到 Top Layer、GND1、SIG1、PWR1、PWR2、SIG2、GND2、Bottom Layer 以及介质层。部分可读参数包括：

- RO3003 介质，介电常数约 3.000，厚度约 5 mil；
- FR-4 High Tg 介质，介电常数约 4.040；
- 顶层/底层铜厚字段约 1.6 mil；
- SIG1/SIG2 铜厚字段约 0.7 mil。

这些字段可用于后续微带线、参考平面和层间耦合分析，但不能单独推出天线方向图。

## 产物

结果位于：

```text
simulation/hardware/awr2944pev/v04_28_pcb_audit/
```

主要文件：

- `source_manifest.csv`：所有输入文件、大小、扩展名、SHA-256 和用途；
- `pcb_component_inventory.csv`：PCB 元件坐标和封装记录；
- `rf_net_inventory.csv`：TX/RX/RF 名称候选及分类；
- `rf_pad_candidates.csv`：所有焊盘几何候选，状态为未确认；
- `layer_stack.csv`：ASCII PCB 解析出的层叠字段；
- `audit_summary.json`：机器可读汇总；
- `output_analysis.md`：本阶段边界和结论。

## 结论边界

本阶段已经证明“资料可读取、PCB 结构可审计、网络和焊盘候选可导出”，但还没有证明真实 TX/RX 天线相位中心坐标。下一阶段需要把候选网络与 TI 的射频封装、天线区域、原理图页、封装库和 `antGeometryCfg` 逐一交叉验证，再生成 `confirmed/candidate/inferred/not_available` 状态的阵元坐标表。
