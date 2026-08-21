# V0.4.66 海杂波点云统计与延迟敏感性

## 阶段目的

将 V0.4.65 的逐点延迟传播结果汇总为按海况/案例和延迟的统计量，为后续报告和海况对比提供统一表格。统计量包括固定姿态和补偿姿态的方位/俯仰均值、标准差、P05/P50/P95，以及姿态补偿造成的坐标变化。

## 本次输入

- V0.4.65 延迟扫描结果；
- 案例：`ss3_nominal`、`ss3_upper`；
- 点数：每个案例 2 个点；
- 延迟：-10、-5、0、+5、+10 ms。

## 结果

| 指标 | 结果 |
|---|---:|
| 案例/海况数 | 2 |
| 延迟数 | 5 |
| 按平均方位+俯仰变化的最小延迟候选 | +10 ms |
| 最小候选的平均角度变化 | 1.770226° |
| 按最大坐标变化的最差延迟候选 | -10 ms |
| 最差候选最大坐标变化 | 0.402018 m |

## 输出文件

- [sea_state_delay_statistics.csv](../simulation/hardware/awr2944pev/v04_66_sea_clutter_statistics/synthetic_run/sea_state_delay_statistics.csv)
- [delay_summary.csv](../simulation/hardware/awr2944pev/v04_66_sea_clutter_statistics/synthetic_run/delay_summary.csv)
- [summary.json](../simulation/hardware/awr2944pev/v04_66_sea_clutter_statistics/synthetic_run/summary.json)
- [output_analysis.md](../simulation/hardware/awr2944pev/v04_66_sea_clutter_statistics/synthetic_run/output_analysis.md)

## 如何从表格分析

1. 先按 `case_id` 分开，避免把不同海况混成一个分布。
2. 比较 `fixed_*` 和 `comp_*` 的均值、标准差和 P05/P50/P95，观察姿态补偿对点云中心和离散度的影响。
3. 查看 `az_change_*`、`el_change_*` 和 `mean/max_position_change_m`，量化时间同步误差传递到点云的程度。
4. 点数很少时只验证程序链路，不把均值或百分位当成稳健海况统计；后续需要完整帧序列和更多海面散射点。

## 结论边界

“+10 ms 是当前示例的最小平均角度变化候选”不等于真实系统的最佳延迟；它只是在当前合成姿态轨迹、点云和扫描范围下的数学结果。当前 `source_is_hardware=false`，不能作为实船虚警率、检测距离或真实 AoA 精度结论。
