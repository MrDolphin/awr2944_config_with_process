# V0.4.70 CFAR 有效区域证据

## 阶段目的

V0.69 发现最高能量单元经常位于 CFAR 边界。本阶段对每个海况、每个 CFAR 场景，只在训练窗和保护窗可以完整展开的有效区域内重新选取最高 20 个功率单元，然后分别统计阈值条件、局部峰值条件和最终保留条件。

## 输出

- [valid_region_cfar_evidence.csv](../simulation/hardware/awr2944pev/v04_70_valid_region_cfar/synthetic_run/valid_region_cfar_evidence.csv)
- [valid_region_cfar_matrix.csv](../simulation/hardware/awr2944pev/v04_70_valid_region_cfar/synthetic_run/valid_region_cfar_matrix.csv)
- [summary.json](../simulation/hardware/awr2944pev/v04_70_valid_region_cfar/synthetic_run/summary.json)
- [output_analysis.md](../simulation/hardware/awr2944_sea_clutter_v02/simulation/hardware/awr2944pev/v04_70_valid_region_cfar/synthetic_run/output_analysis.md)

## 结果含义

有效区域内的证据把 CFAR 保留拆成三类：

- `above_threshold_count`：功率超过 CFAR 阈值；
- `local_maximum_count`：单元是 3×3 邻域峰值；
- `would_detect_count`：两者同时满足。

例如 `ss0_flat/pfa1e2_train2` 的 20 个有效区域候选中，有 20 个超过阈值，但 0 个是局部峰值，因此最终仍为 0 个检测点。这说明“超过阈值”并不等于“会被当前峰值检测逻辑保留”。训练窗更大的场景则可能连阈值条件也无法通过。

## 当前结论

低海况零检测不能简单归因于海面没有能量：

1. V0.68 已证明未 CFAR 功率存在长尾；
2. V0.69 已证明全局最高点常在 CFAR 边界；
3. V0.70 在有效区域中仍观察到“超过阈值但不是局部峰值”的单元；
4. 因此当前零点现象同时受到边界、训练窗阈值和局部峰值规则影响。

## 证据边界

本阶段描述的是项目内 CA-CFAR 实现的决策路径，不代表 TI SDK 的完整 CFAR/峰值实现，也不是实测 DCA1000 数据。要形成硬件结论，必须用同一套证据表处理实测 IQ，并确认 CFAR 参数与固件配置一致。
