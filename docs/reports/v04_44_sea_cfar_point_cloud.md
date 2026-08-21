# V0.4.44 合成海杂波 CA-CFAR 三维点云

## 目标

对 V0.4.43 的每个海况距离-多普勒功率谱执行二维 CA-CFAR，并使用同一距离-Doppler 单元的 4×4 复数通道谱进行稀疏阵列 AoA，形成带距离、速度、方位、俯仰和笛卡尔坐标的点云。

## 参数

- CA-CFAR：二维 CA；
- 训练单元：距离/速度各 4；
- 保护单元：距离/速度各 1；
- `Pfa=1e-3`；
- 每帧最多保留 32 个检测点；
- AoA 搜索网格：方位 1°、俯仰 1°；
- 输入通道顺序：未经过真实硬件验证。

## 结果

| 海况 | 检测点数 | 平均点/帧 | 平均距离 | 平均速度 | 平均 AoA 相关性 |
|---|---:|---:|---:|---:|---:|
| `ss0_flat` | 0 | 0.00 | — | — | — |
| `ss1_rippled` | 0 | 0.00 | — | — | — |
| `ss2_normal` | 0 | 0.00 | — | — | — |
| `ss3_nominal` | 2 | 0.05 | 8.7830 m | 0.0000 m/s | 0.4245 |
| `ss3_upper` | 2 | 0.05 | 8.7830 m | 0.0000 m/s | 0.3570 |

## 如何解释零检测点

`ss0`～`ss2` 在当前 `Pfa=1e-3`、训练/保护单元和合成散射代理下没有通过 CA-CFAR，并不等于“雷达看不到海面”。可能原因包括：

- 当前近距离散射代理集中在低距离单元；
- 近距离单元被训练窗边缘排除；
- 多微元相干叠加造成局部功率不满足 CFAR 峰值条件；
- 当前 Pfa 和训练单元对合成谱过于严格；
- V0.4.43 的散射权重和噪声底尚未按实测数据标定。

这正是 CFAR 参数敏感性需要单独评估的原因，不能把零检测直接当作海况探测结论。

## 点云坐标

每个检测点保存：

```text
frame
range_m
velocity_mps
azimuth_deg
elevation_deg
power_linear
noise_linear
threshold_linear
aoa_score
x_m, y_m, z_m
```

坐标转换为：

```text
x = R cos(el) sin(az)
y = R cos(el) cos(az)
z = R sin(el)
```

## 结果文件

结果目录：`simulation/hardware/awr2944pev/v04_44_sea_cfar_point_cloud/`

- `*_point_cloud.h5`：各海况点云 HDF5；
- `point_cloud.csv`：所有海况点云汇总；
- `case_summary.csv`：海况统计；
- `summary.json`：机器可读摘要；
- `output_analysis.md`：阶段结论。

## 边界

输入功率谱和复数通道谱来自 V0.4.43 合成海杂波，不是 DCA1000 实测 IQ；PCB 阵列仍是 RF 铜区质心候选；CFAR 和 AoA 不是 TI SDK 默认实现。因此本阶段验证的是“距离-速度-角度-点云”数据契约和参数敏感性，不是实板虚警率、探测距离或真实海况性能。
