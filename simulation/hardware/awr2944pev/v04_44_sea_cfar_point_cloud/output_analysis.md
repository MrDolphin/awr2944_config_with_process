# V0.4.44 合成海杂波 CA-CFAR 三维点云

本阶段读取 V0.4.43 的距离-多普勒功率谱和 4×4 复数通道谱，对每个海况、每个时间帧执行二维 CA-CFAR，并对每个保留检测点执行稀疏阵列 AoA。

| 海况 | 检测点数 | 平均点/帧 | 平均距离(m) | 平均速度(m/s) | 平均 AoA 相关性 |
|---|---:|---:|---:|---:|---:|
| ss0_flat | 0 | 0.00 | nan | nan | nan |
| ss1_rippled | 0 | 0.00 | nan | nan | nan |
| ss2_normal | 0 | 0.00 | nan | nan | nan |
| ss3_nominal | 2 | 0.05 | 8.7830 | 0.000000 | 0.4245 |
| ss3_upper | 2 | 0.05 | 8.7830 | 0.000000 | 0.3570 |

## 点云坐标

使用 x=R·cos(el)·sin(az)、y=R·cos(el)·cos(az)、z=R·sin(el)。每个 HDF5 的 /point_cloud 包含 frame、range、velocity、azimuth、elevation、power、noise、threshold、aoa_score 和 x/y/z。

## 边界

CFAR 输入和复数谱来自 V0.4.43 合成海杂波，不是 DCA1000 实测 IQ；PCB 阵列仍是铜区质心候选；CFAR 和 AoA 参数不是 TI SDK 默认实现。因此点云用于验证数据结构、指标和海况差异，不能作为实板虚警率或检测距离结论。
