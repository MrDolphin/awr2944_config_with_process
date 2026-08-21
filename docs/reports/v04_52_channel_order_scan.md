# V0.4.52 已知角度 RX/TX 通道顺序扫描

## 本阶段目的

真实 DCA1000 AoA 处理中，RX/TX 通道顺序一旦错位，距离-Doppler 峰值仍可能正常，但阵列复数相位关系会改变，最终导致方位和俯仰错误。本阶段用 PCB 铜区质心候选阵列生成已知角度复数通道，枚举 RX/TX 排列，量化这种风险。

测试真值：

- `(方位, 俯仰)=(10°, 2°)`；
- `(方位, 俯仰)=(30°, 5°)`。

每个真值枚举 `24×24=576` 个 RX/TX 排列，共 1152 个候选。

## 结果

- 正确身份排列：两个真值均估计为真实角度，方位和俯仰误差均为 `0°`（合成回归）；
- 错误排列方位 RMSE：`42.286°`；
- 错误排列俯仰 RMSE：`11.962°`；
- 错误排列仍落在方位和俯仰各 ±2° 内的比例：约 `0.261%`。

这说明“偶然得到一个看起来合理的角度”不能证明通道顺序正确。绝大多数错误排列会造成很大的 AoA 偏差。

## 如何分析输出数据

打开 [channel_order_candidates.csv](../../simulation/hardware/awr2944pev/v04_52_channel_order_scan/channel_order_candidates.csv)：

1. 按 `truth_azimuth_deg/truth_elevation_deg` 分组；
2. 查看 `rx_order`、`tx_order`；
3. 计算 `estimated_* - truth_*`；
4. 将 `identity_order=true` 的行与错误排列分开；
5. 如果真实 DCA1000 已知角度数据接入，应以实测误差最小且跨多个角度稳定的排列作为候选，而不是只看一个角度。

## 工程意义

V0.51 已从 CFG 推导出当前配置的 TX 轮询顺序 `TX0→TX2→TX3→TX1`。V0.52 进一步说明：即使 CFG 顺序正确，仍需要确认 DCA1000 原始 LVDS 数据的 RX 排列、软件重排和 TI SDK 虚拟阵列坐标是否一致。

真实验证建议使用：

- 已知距离角反射器；
- 至少两个方位角和两个俯仰角；
- DCA1000 未经 CFAR 的原始 IQ；
- 同一次采集使用的 CFG；
- 通道顺序和校准矩阵的 provenance 记录。

## 边界

本阶段使用合成导向矢量，PCB 坐标是铜区质心候选，不是电气相位中心；未包含真实方向图、互耦、幅相误差、TI SDK AoA 和实测 IQ。因此这些误差用于风险排序和测试设计，不能作为 AWR2944P 的实测 AoA 指标。

## 输出文件

- [channel_order_candidates.csv](../../simulation/hardware/awr2944pev/v04_52_channel_order_scan/channel_order_candidates.csv)
- [summary.json](../../simulation/hardware/awr2944pev/v04_52_channel_order_scan/summary.json)
- [output_analysis.md](../../simulation/hardware/awr2944pev/v04_52_channel_order_scan/output_analysis.md)

