# V0.4.47 目标检测边界扫描

本阶段在 V0.4.43 合成距离-Doppler 海杂波谱上，分别扫描目标 SNR、距离、径向速度、方位和俯仰；海况选取 `ss2_normal` 与有效波高 1 m 的 `ss3_upper`。每个点使用 41 帧、CA-CFAR 训练窗 `(2,2)`、保护窗 `(1,1)`，并比较 `Pfa=10^-2/10^-3`。

## 结果读取

`detection_probability` 是目标单元 ±1 网格内的命中帧比例；`mean_false_alarms_per_frame` 是扣除目标命中后的平均检测点数；AoA RMSE 只在命中帧上统计。

## 每组扫描的摘要

| 海况 | 扫描变量 | Pfa | 最小检测概率 | 最大检测概率 | 最大虚警点/帧 |
|---|---|---:|---:|---:|---:|
| ss2_normal | snr_db | 0.01 | 0.000 | 1.000 | 0.000 |
| ss2_normal | snr_db | 0.001 | 0.000 | 1.000 | 0.000 |
| ss2_normal | range_m | 0.01 | 1.000 | 1.000 | 0.000 |
| ss2_normal | range_m | 0.001 | 1.000 | 1.000 | 0.000 |
| ss2_normal | velocity_mps | 0.01 | 1.000 | 1.000 | 0.000 |
| ss2_normal | velocity_mps | 0.001 | 1.000 | 1.000 | 0.000 |
| ss2_normal | azimuth_deg | 0.01 | 1.000 | 1.000 | 0.000 |
| ss2_normal | azimuth_deg | 0.001 | 1.000 | 1.000 | 0.000 |
| ss2_normal | elevation_deg | 0.01 | 1.000 | 1.000 | 0.000 |
| ss2_normal | elevation_deg | 0.001 | 1.000 | 1.000 | 0.000 |
| ss3_upper | snr_db | 0.01 | 0.000 | 1.000 | 0.268 |
| ss3_upper | snr_db | 0.001 | 0.000 | 1.000 | 0.146 |
| ss3_upper | range_m | 0.01 | 1.000 | 1.000 | 0.268 |
| ss3_upper | range_m | 0.001 | 1.000 | 1.000 | 0.146 |
| ss3_upper | velocity_mps | 0.01 | 1.000 | 1.000 | 0.268 |
| ss3_upper | velocity_mps | 0.001 | 1.000 | 1.000 | 0.146 |
| ss3_upper | azimuth_deg | 0.01 | 1.000 | 1.000 | 0.268 |
| ss3_upper | azimuth_deg | 0.001 | 1.000 | 1.000 | 0.146 |
| ss3_upper | elevation_deg | 0.01 | 1.000 | 1.000 | 0.268 |
| ss3_upper | elevation_deg | 0.001 | 1.000 | 1.000 | 0.146 |

## 初步结论

扫描结果用于确定后续重点工况：检测概率下降的位置是目标检测边界候选，虚警点随海况和 Pfa 的变化用于 CFAR 参数权衡。由于目标单元仍为受控相干注入，结果不能替代实测探测距离、真实目标 RCS、TI SDK AoA 或海试检测概率。

## 输出文件

- `detection_boundary_sweep.csv`：逐海况、逐扫描点、逐 Pfa 的完整指标。
- `summary.json`：输入与扫描配置。
- 本文件：面向汇报的解释和边界。
