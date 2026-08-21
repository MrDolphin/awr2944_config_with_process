# V0.4.68 CFAR 参数与未 CFAR 能量矩阵

## 阶段目的

针对 V0.67 中 `ss0_flat`、`ss1_rippled`、`ss2_normal` 的零 CFAR 检测，固定同一份 V0.43 距离-多普勒输入，扫描 V0.45 的 9 组 Pfa/训练窗/保护窗，并同时统计未经过 CFAR 的功率和复数谱形状。

## 覆盖范围

- 海况：`ss0_flat`、`ss1_rippled`、`ss2_normal`、`ss3_nominal`、`ss3_upper`；
- CFAR 场景：9 组；
- 每个输入形状：功率 `[41,64,64]`，复数谱 `[41,64,64,4,4]`。

## 未 CFAR 能量结果

| 案例 | P95 | P99 | P99.9 | 最大/中位数 |
|---|---:|---:|---:|---:|
| `ss0_flat` | 3.72e-4 | 0.4351 | 9.6381e4 | 177.90 dB |
| `ss1_rippled` | 4.69e-4 | 0.7155 | 1.5493e5 | 183.60 dB |
| `ss2_normal` | 7.15e-4 | 3.3516 | 1.5409e5 | 181.88 dB |
| `ss3_nominal` | 1.31e-3 | 9.1136 | 1.8678e5 | 180.96 dB |
| `ss3_upper` | 1.58e-3 | 13.0395 | 1.6457e5 | 182.98 dB |

三个低海况虽然所有 9 组 CFAR 都没有保留点，但未 CFAR 功率仍然存在长尾，尤其 P99.9 和最大值远高于中位数。因此当前证据更支持“CFAR 局部峰值/训练窗/边界条件未保留这些能量”，而不是“低海况没有海杂波”。这仍需要进一步检查峰值位置、噪声估计和 CFAR 保护区，不能直接下硬件结论。

## 输出文件

- [pre_cfar_energy.csv](../simulation/hardware/awr2944pev/v04_68_cfar_energy_matrix/synthetic_run/pre_cfar_energy.csv)
- [cfar_energy_matrix.csv](../simulation/hardware/awr2944pev/v04_68_cfar_energy_matrix/synthetic_run/cfar_energy_matrix.csv)
- [summary.json](../simulation/hardware/awr2944pev/v04_68_cfar_energy_matrix/synthetic_run/summary.json)
- [output_analysis.md](../simulation/hardware/awr2944pev/v04_68_cfar_energy_matrix/synthetic_run/output_analysis.md)

## 如何继续分析

1. 从 `cfar_energy_matrix.csv` 观察每个 Pfa/训练窗组合是否改变点数；
2. 对零检测海况定位未 CFAR 的最大功率单元，检查它是否位于 CFAR 不可评估的边界；
3. 比较该单元的噪声估计、阈值和邻域峰值条件；
4. 若仍无检测，再降低 Pfa 或改用 OS-CFAR/非 CFAR 能量统计；
5. 最终用 DCA1000 原始 IQ 重复同一矩阵，才有实测意义。

## 证据边界

输入是合成距离-多普勒功率谱和复数谱，`hardware_validated=false`。本阶段可以解释仿真中 CFAR 与能量的关系，不能报告实船虚警率、检测距离或真实海杂波强度。
