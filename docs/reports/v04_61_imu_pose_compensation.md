# V0.4.61 IMU 姿态补偿前后点云比较

## 阶段目的

将 V0.4.60 的姿态敏感性扫描落实为逐帧处理：每个点云 frame 使用对应的 roll/pitch/yaw，而固定姿态基线把全部点按同一个安装姿态投影。这样可以量化船体姿态变化对海杂波点云空间位置、方位角和俯仰角的影响。

## 输入

- 点云：`simulation/hardware/awr2944pev/v04_44_sea_cfar_point_cloud/point_cloud.csv`。
- 姿态：`simulation/hardware/awr2944pev/v04_61_imu_pose_compensation/synthetic_imu_trace.csv`。
- 名义安装姿态：V0.4.59 的 `candidate_vertical_pose.json`。

本次点云包含 `ss3_nominal` 和 `ss3_upper` 的 4 个点，使用 frame 25 和 frame 29；姿态轨迹为 frame 25 的 `(90°,0°,0°)` 和 frame 29 的 `(91.5°,1°,2°)`。

## 结果

| 指标 | 结果 |
|---|---:|
| 点数 | 4 |
| 帧数 | 2 |
| 最大坐标变化 | 0.402018 m |
| 最大方位变化 | 4.472247° |
| 最大俯仰变化 | 1.540757° |

## 如何理解

固定姿态和姿态补偿的差异，表示的是“同一雷达点在不同船体姿态解释下的坐标差异”。它不是海面目标真的移动了 0.402 m，也不是 AoA 算法本身产生了 4.47° 误差。对于船载低高度雷达，这个差异会改变海面点的方位、俯仰和局部掠射角，因此后续海杂波统计应在补偿后的船体/世界坐标中进行。

## 输出

- `simulation/hardware/awr2944pev/v04_61_imu_pose_compensation/synthetic_run/point_cloud_pose_comparison.csv`
- `simulation/hardware/awr2944pev/v04_61_imu_pose_compensation/synthetic_run/summary.json`
- `simulation/hardware/awr2944pev/v04_61_imu_pose_compensation/synthetic_run/output_analysis.md`

## 证据边界

当前 IMU 轨迹和点云都是合成/历史仿真输入，不能作为船上实测姿态补偿性能。接入真实数据前必须确认：IMU 与雷达时间戳同步、坐标轴方向、角度单位、初始安装外参、姿态滤波延迟，以及点云 frame 与 IMU frame 的对应关系。`source_is_hardware_imu=false` 和 `point_cloud_source_is_hardware=false` 是当前正确状态。
