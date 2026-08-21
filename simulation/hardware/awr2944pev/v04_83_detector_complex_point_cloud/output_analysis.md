# V0.4.83 检测器→复数 bin AoA→海杂波三维点云

本阶段把每个检测器保留的距离-多普勒单元直接映射到对应的 4×4 复数通道矩阵，逐点估计方位/俯仰，再计算雷达坐标系 XYZ。输出字段包含检测器、CFAR 场景、距离、速度、角度、功率、噪声、门限和三维坐标。

海况数量：5；CFAR 场景数量：9；检测器：['ca_cfar_local_peak', 'ca_threshold_only', 'os_cfar_local_peak', 'fixed_energy_local_peak']；每组最多保存 256 点。

## 解释方法

先比较 `detection_count` 判断检测规则保留了多少距离-多普勒单元，再比较 `azimuth_deg/elevation_deg` 的均值和标准差判断点云空间偏置，最后查看 `x_m/y_m/z_m` 的距离投影。`stored_point_count` 只是功率最高点的保存上限，不等于总检测数。

## 证据边界

输入是 MATLAB 合成 HDF5，阵列坐标是候选 mapping，OS-CFAR 是探索性实现，通道顺序和 TI 校准未验证；因此本阶段是软件管线验证，不是实测 AoA 或真实海杂波虚警率。
