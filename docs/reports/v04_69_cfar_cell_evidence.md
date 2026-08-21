# V0.4.69 高能量单元与 CFAR 决策证据

## 阶段目的

定位每个海况未 CFAR 功率谱最高的 5 个单元，逐一计算 CA-CFAR 的训练窗噪声、阈值、功率/阈值比、邻域最大值以及两个保留条件。这样可以解释“功率很高但为什么没有形成 CFAR 点”。

## 覆盖

- 海况：5 个；
- 每个海况最高能量单元：5 个；
- CFAR 场景：9 组；
- 证据单元总数：225 行。

## 本次结果

```text
每个海况最高能量单元：5 个
任一 CFAR 场景满足两个保留条件的高能量单元：0 个
```

这个结果说明当前“最高功率的几个单元”并不等于 CA-CFAR 最终检测点。可能原因包括：

- 单元位于 CFAR 无法评估的边界；
- 功率虽高但训练窗噪声估计更高，阈值未超过；
- 功率超过阈值但不是 3×3 局部峰值；
- 高能量单元来自仿真中的孤立谱结构，而不是当前 CFAR 参数定义的目标峰。

## 输出文件

- [cfar_cell_evidence.csv](../simulation/hardware/awr2944pev/v04_69_cfar_cell_evidence/synthetic_run/cfar_cell_evidence.csv)
- [top_energy_cells.csv](../simulation/hardware/awr2944pev/v04_69_cfar_cell_evidence/synthetic_run/top_energy_cells.csv)
- [summary.json](../simulation/hardware/awr2944pev/v04_69_cfar_cell_evidence/synthetic_run/summary.json)
- [output_analysis.md](../simulation/hardware/awr2944pev/v04_69_cfar_cell_evidence/synthetic_run/output_analysis.md)

## 如何逐单元分析

1. 先看 `valid_for_cfar`；若为 false，说明单元位于训练窗/保护窗无法完整覆盖的边界，不能直接评价漏检。
2. 看 `power_to_threshold_db`；负值表示功率没有超过门限。
3. 看 `above_threshold`；这是功率门限条件。
4. 看 `local_maximum`；这是邻域峰值条件。
5. 只有 `above_threshold=true` 且 `local_maximum=true`，`would_detect` 才为 true。

## 证据边界

当前输入是合成距离-多普勒功率谱，不是 DCA1000 实测 IQ；此阶段可以解释当前 CA-CFAR 实现的决策路径，不能直接代表 TI SDK 或实板 CFAR 行为。
