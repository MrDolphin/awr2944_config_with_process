# V0.4.72 检测器 AoA 点云

本阶段将 V0.4.71 在相同距离-多普勒谱上保留的检测单元转换为方位、俯仰和三维坐标。`detection_count` 是全部保留单元，CSV 仅保存每组功率最高的前 N 个点；因此 `stored_point_count` 不代表检测器总数。

## 结果解读

- 比较同一海况、同一 CFAR 场景下四种 detector 的 detection_count，可看检测规则对海杂波保留量的影响。
- 比较 azimuth/elevation 的均值和标准差，可看保留点云的空间偏置与离散程度。
- `x_m/y_m/z_m` 采用雷达坐标系：x 为右舷横向、y 为前向、z 为上向；距离和速度来自合成谱坐标轴。

## 证据边界

输入为 MATLAB 合成 HDF5，不是 DCA1000 实测 IQ；阵列坐标仍是候选映射而非电气相位中心；OS-CFAR 为探索性实现，不等价于 TI SDK。
