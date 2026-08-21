# V0.4.54 多场景已知角度 AoA 联合验证

## 本阶段目的

单个已知角度只能排除一部分错误通道排列。本阶段使用三个合成 HDF5 场景，在多个方位、俯仰和距离上联合评估 576 个 RX/TX 排列候选：

| 场景 | 方位 | 俯仰 | 距离 |
|---|---:|---:|---:|
| scene 1 | 5° | 2° | 20 m |
| scene 2 | 15° | 8° | 30 m |
| scene 3 | 30° | 5° | 40 m |

几何源为与 fixture 生成一致的 `simulation.v04.virtual_array_positions`。

## 结果

身份排列 RX=`0,1,2,3`、TX=`0,1,2,3`：

- 方位 RMSE：`0°`；
- 俯仰 RMSE：`0°`；
- 综合 RMSE：`0°`；
- 平均相关性：`1.0`。

排名靠前的部分错误排列虽然方位 RMSE 仍可能是 `0°`，但俯仰 RMSE 已达到约 `1.29°`，说明只看方位不能确认通道顺序，必须联合分析方位和俯仰。

## 如何分析图文数据

打开 [multi_scene_candidate_metrics.csv](../../simulation/hardware/awr2944pev/v04_54_multiscene_aoa_validation/results/multi_scene_candidate_metrics.csv)：

1. 按 `combined_rmse_deg` 从小到大排序；
2. 检查候选在所有场景上的 `max_abs_azimuth_error_deg` 和 `max_abs_elevation_error_deg`；
3. 比较 `identity_order` 与排名第一候选是否一致；
4. 如果错误排列在某一个场景很接近、但多场景 RMSE 明显变大，则说明单场景存在角度歧义；
5. 对真实 DCA1000 数据，应把此表作为通道顺序和校准矩阵候选筛选结果，而不是直接作为最终硬件精度。

## 工程结论

多角度、多距离联合验证比单个 fixture 更适合作为真实硬件验收门槛。建议实测至少覆盖：

- 两个以上方位角；
- 两个以上俯仰角；
- 两个以上距离；
- 一个接近波束中心、一个接近波束边缘的目标。

只有一个候选在全部场景上稳定达到误差门限时，才可以把 `channel_order_verified` 从 `false` 变成经过人工审查的候选状态。自动脚本不会替用户宣称硬件已校准。

## 边界

本阶段输入均为合成 known-angle HDF5，不是实测 DCA1000 IQ；PCB 电气相位中心、真实方向图、TI SDK AoA 和通道校准仍未验证。

## 输出文件

- [multi_scene_errors.csv](../../simulation/hardware/awr2944pev/v04_54_multiscene_aoa_validation/results/multi_scene_errors.csv)
- [multi_scene_candidate_metrics.csv](../../simulation/hardware/awr2944pev/v04_54_multiscene_aoa_validation/results/multi_scene_candidate_metrics.csv)
- [summary.json](../../simulation/hardware/awr2944pev/v04_54_multiscene_aoa_validation/results/summary.json)
- [output_analysis.md](../../simulation/hardware/awr2944pev/v04_54_multiscene_aoa_validation/results/output_analysis.md)

