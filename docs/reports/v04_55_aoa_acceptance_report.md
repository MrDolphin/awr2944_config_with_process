# V0.4.55 AoA 综合验收门禁

## 本阶段目的

将 V0.53 的单场景 HDF5 通道顺序验证和 V0.54 的多场景 RMSE 聚合，整理成一个明确的验收门禁，区分：

- 合成算法回归是否通过；
- 真实 AWR2944P 硬件 AoA 是否已经具备放行条件。

## 当前门禁结果

| 门禁 | 结果 |
|---|---|
| 多场景数据存在 | 通过 |
| 候选排列数为 576 | 通过 |
| 最佳候选综合 RMSE ≤ 2° | 通过 |
| 最佳候选最大误差 ≤ 5° | 通过 |
| 最佳候选与身份排列一致 | 通过 |
| HDF5 已知角度元数据存在 | 通过 |
| 输入为真实硬件采集 | 未通过 |
| 硬件通道顺序已实测确认 | 未通过 |

因此当前结论是：

- **合成 AoA 回归：通过**；
- **真实硬件 AoA 放行：未通过**。

## 为什么不能把合成回归当成硬件完成

本次输入的 provenance 为：

```text
source_type = synthetic_known_angle_regression_only
calibration_status = synthetic_unity_not_measured
geometry_source = simulation.v04.virtual_array_positions
```

同时：

```text
channel_order_hardware_verified = false
```

因此 0° RMSE 只说明：在几何源一致、导向矢量已知、无真实硬件误差的条件下，软件排列和 AoA 统计链路工作正常。

## 真实硬件放行条件

要将门禁推进到“真实硬件 AoA 已验证”，必须重新运行 V0.51–V0.55，输入：

1. 真实 DCA1000 原始 IQ；
2. 同一次采集所使用的 CFG；
3. 已知距离和已知 RCS 的角反射器；
4. 至少两个方位角、两个俯仰角和两个距离；
5. 实测校准矩阵及其来源记录；
6. 确认的 RX/TX 物理通道顺序；
7. 确认的 PCB/CAD 电气相位中心坐标。

## 输出文件

- [acceptance_summary.json](../../simulation/hardware/awr2944pev/v04_55_aoa_acceptance_report/acceptance_summary.json)
- [output_analysis.md](../../simulation/hardware/awr2944pev/v04_55_aoa_acceptance_report/output_analysis.md)

