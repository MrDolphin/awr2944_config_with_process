# V0.4.62 IMU/雷达时间对齐

## 阶段目的

V0.4.61 已经能按 frame 使用姿态，但真实系统中雷达 frame 和 IMU 样本通常不是同一频率，也不一定时间戳相同。本阶段增加时间覆盖、单调性、最大采样间隔和线性插值质量门，生成可直接供姿态补偿使用的 `aligned_imu.csv`。

## 输入

- 雷达 frame：`simulation/hardware/awr2944pev/v04_62_imu_time_alignment/synthetic_radar_frames.csv`。
- IMU 样本：`simulation/hardware/awr2944pev/v04_62_imu_time_alignment/synthetic_imu_samples.csv`。
- 当前示例雷达 frame 时间：0～0.1 s，间隔 0.025 s。
- 当前示例 IMU 样本时间：0～0.1 s，间隔 0.05 s。
- 最大允许 IMU 源间隔：0.06 s。

## 结果

| 指标 | 结果 |
|---|---:|
| 雷达 frame 数 | 5 |
| IMU 样本数 | 3 |
| 雷达时间范围 | 0～0.1 s |
| IMU 时间范围 | 0～0.1 s |
| 最大 IMU 源间隔 | 0.05 s |
| 最大对齐间隔 | 0.05 s |
| 时间覆盖 | 通过 |
| 质量门 | 通过 |

## 如何分析输出

打开 `aligned_imu.csv`，每行对应一个雷达 frame：

- `imu_left_time_s` 和 `imu_right_time_s`：该 frame 落在的 IMU 样本区间；
- `interpolation_alpha`：线性插值比例；
- `roll_deg/pitch_deg/yaw_deg`：供 V0.4.61 使用的对齐姿态；
- `alignment_status`：若超出覆盖范围、时间戳不单调或采样间隔超限，则为 `quality_gate_failed`。

## 结论

当前时间对齐闭环已经可运行，且示例数据质量门通过。真实 IMU 接入时，不能只看文件是否生成，还必须检查 `coverage_ok`、`max_source_imu_gap_s` 和 `quality_pass`。超出 IMU 覆盖范围的雷达 frame 不应静默外推，否则会把姿态补偿结果伪装成有效数据。

## 证据边界

当前输入为合成时间序列，`source_is_hardware_imu=false`。真实系统还需要记录时钟来源、时间偏差、串口/网络延迟、姿态滤波延迟和雷达 frame 触发时刻；这些因素会直接影响海杂波点云的方位、俯仰和掠射角补偿。
