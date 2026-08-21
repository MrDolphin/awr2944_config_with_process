# V0.4.67 全海况点云覆盖矩阵

## 阶段目的

将 V0.44 保存的五个海况 HDF5 全部纳入分析，明确记录每个海况的帧数、CFAR 点数、检测帧率、距离/速度/角度统计，以及是否已经进入 V0.65 的姿态/延迟传播链路。

## 覆盖结果

| 案例 | 帧数 | CFAR 点数 | 检测帧率 | CFAR 状态 | 姿态/延迟传播 |
|---|---:|---:|---:|---|---|
| `ss0_flat` | 41 | 0 | 0 | 无 CFAR 检测点 | 无 |
| `ss1_rippled` | 41 | 0 | 0 | 无 CFAR 检测点 | 无 |
| `ss2_normal` | 41 | 0 | 0 | 无 CFAR 检测点 | 无 |
| `ss3_nominal` | 41 | 2 | 1/41 | 有合成检测点 | 有 |
| `ss3_upper` | 41 | 2 | 1/41 | 有合成检测点 | 有 |

## 关键结论

- 五个海况都进入了覆盖矩阵；
- 只有 `ss3_nominal` 和 `ss3_upper` 有 CFAR 保留点；
- `ss0_flat`、`ss1_rippled`、`ss2_normal` 的零点不是“没有海杂波”，而是当前 CFAR 参数和仿真功率下没有保留下来的检测点；
- V0.65 的姿态/延迟传播目前只覆盖有点云的两个 ss3 案例；
- 零检测案例仍被保留，避免把数据缺口写成物理结论。

## 输出文件

- [all_sea_state_coverage.csv](../simulation/hardware/awr2944pev/v04_67_all_sea_states/synthetic_run/all_sea_state_coverage.csv)
- [summary.json](../simulation/hardware/awr2944pev/v04_67_all_sea_states/synthetic_run/summary.json)
- [output_analysis.md](../simulation/hardware/awr2944_sea_clutter_v02/simulation/hardware/awr2944pev/v04_67_all_sea_states/synthetic_run/output_analysis.md)

## 领导汇报时的表述

可以说：“当前五级海况数据链路已全部覆盖；在当前 CA-CFAR 参数下，三级海况样例产生了可分析点云，0～2 级海况暂未产生 CFAR 检测点，需要进一步调节 CFAR/Pfa 或直接分析未 CFAR 复数回波。”

不应说：“低海况没有海杂波”或“雷达只在三级海况工作”。这两种说法都超出了当前数据证据。

## 证据边界

当前 HDF5 来自合成海杂波，通道顺序和硬件 AoA 尚未实测确认；该覆盖矩阵用于验证海况比较和数据缺口管理，不是实船虚警率、检测距离或真实海杂波强度结论。
