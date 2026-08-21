# V0.4.71 CFAR/能量门检测器对照

## 阶段目的

在相同五海况、相同距离-多普勒功率谱上，对照四种保留逻辑：

1. `ca_cfar_local_peak`：当前 CA-CFAR 阈值 + 3×3 局部峰值；
2. `ca_threshold_only`：只使用 CA-CFAR 阈值，关闭局部峰值条件；
3. `os_cfar_local_peak`：探索性排序统计量门限 + 局部峰值；
4. `fixed_energy_local_peak`：每个案例 P99 固定能量门限 + 局部峰值。

## 输出

- [detector_comparison.csv](../simulation/hardware/awr2944pev/v04_71_detector_comparison/synthetic_run/detector_comparison.csv)
- [summary.json](../simulation/hardware/awr2944pev/v04_71_detector_comparison/synthetic_run/summary.json)
- [output_analysis.md](../simulation/hardware/awr2944pev/v04_71_detector_comparison/synthetic_run/output_analysis.md)

## 当前观察

以 `ss0_flat/pfa1e2_train2` 为例：

- `ca_cfar_local_peak`：0 个；
- `ca_threshold_only`：1599 个；
- `os_cfar_local_peak`：0 个；
- `fixed_energy_local_peak`：0 个。

这个对照说明，当前合成谱中有大量单元可以超过 CA 阈值，但关闭局部峰值后点数急剧增加；因此局部峰值规则对零检测现象有实质影响。与此同时，固定 P99 能量门 + 局部峰值并未产生点，说明“全局能量超过某个百分位”与“局部可分离峰”不是同一概念。

## 如何解读

- `detection_count`：当前网格上被该逻辑保留的单元数；
- `detections_per_frame`：平均每帧保留单元数；
- `detection_rate_valid_cells`：相对于 CFAR 可评估单元的保留比例；
- `ca_threshold_only` 与 `ca_cfar_local_peak` 的差异：局部峰值规则的影响；
- CA 与 OS 的差异：噪声统计方式的敏感性；
- 固定能量门与 CFAR 的差异：全局百分位门限是否能替代局部自适应门限。

## 证据边界

OS-CFAR 的排序位置和缩放因子是本项目的探索性对照，不声称等价于 TI SDK；固定 P99 也不是雷达固件的检测器。所有输入均为合成数据，不能报告实船虚警率、探测距离或硬件 AoA 性能。
