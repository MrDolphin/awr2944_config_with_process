# V0.4.65 延迟误差到海杂波点云指标传播

## 阶段目的

将 V0.4.64 的固定延迟扫描继续传递到 V0.4.61 的逐帧点云姿态补偿，形成：

```text
时间延迟假设 → IMU/雷达对齐 → 逐帧姿态 → 点云坐标 → 方位/俯仰指标
```

## 输入

- 点云：V0.4.44 `point_cloud.csv`，包含 4 个历史合成海杂波点；
- 雷达 frame：`synthetic_radar_frames.csv`，覆盖 frame 25～29；
- IMU：V0.4.63 毫秒单位合成 IMU；
- 延迟：`-10、-5、0、+5、+10 ms`；
- 名义姿态：V0.4.59 板面垂直候选。

## 结果

| 指标 | 结果 |
|---|---:|
| 扫描延迟数 | 5 |
| 通过时间质量门 | 5 |
| 通过延迟范围 | -10～+10 ms |
| 通过场景最大坐标变化 | 0.402018 m |
| 通过场景最大方位变化 | 4.472247° |
| 通过场景最大俯仰变化 | 1.540757° |

## 如何理解

这些指标是“相对于固定名义姿态的点云坐标解释变化”。它们表示时间同步假设经过姿态补偿后，海杂波点在船体/世界参考坐标中的位置、方位和俯仰会怎样变化；不是海面目标真实移动，也不是毫米波雷达 AoA 硬件误差。

如果某个延迟的时间质量门失败，则该延迟不会进入点云指标比较，避免用超出 IMU 覆盖范围的外推姿态制造伪结果。

## 输出

- `simulation/hardware/awr2944pev/v04_65_latency_pointcloud/synthetic_run/latency_pointcloud_metrics.csv`
- `simulation/hardware/awr2944pev/v04_65_latency_pointcloud/synthetic_run/summary.json`
- `simulation/hardware/awr2944pev/v04_65_latency_pointcloud/synthetic_run/output_analysis.md`
- 每个延迟目录下保存完整的归一化、对齐和补偿结果。

## 结论边界

当前点云和 IMU 都是合成/历史仿真输入，`point_cloud_source_is_hardware=false`、`imu_source_is_hardware=false`。真实实验中应把实测 DCA1000 点云、真实 IMU、实测时间延迟和校准矩阵替换进同一链路，然后再报告海杂波方位/俯仰变化。
