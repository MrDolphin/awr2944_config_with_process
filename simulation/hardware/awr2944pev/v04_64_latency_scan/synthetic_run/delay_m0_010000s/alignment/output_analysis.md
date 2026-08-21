# V0.4.62 IMU/雷达时间对齐

雷达 frame 数：5；IMU 样本数：3。
雷达时间范围：0.010000～0.090000 s；IMU 时间范围：-0.010000～0.090000 s。
最大 IMU 源间隔：0.050000 s；允许最大间隔：0.060000 s；coverage_ok=True；quality_pass=True。

## 如何解释

`aligned_imu.csv` 的每一行对应一个雷达 frame。姿态由相邻 IMU 样本线性插值得到；`interpolation_alpha` 为 0 表示恰好落在左侧样本，1 表示右侧样本。任何雷达时间超出 IMU 覆盖范围、IMU 时间戳不单调或采样间隔超过门限，都会使质量门失败。

## 证据边界

当前输入为合成时间序列，source_is_hardware_imu=false。真实数据接入时还需加入时间戳来源、时钟偏差、串口延迟和姿态滤波延迟记录。
