# V0.4.24 海况点云统计

## 目的

对多个 V0.4.23 点云运行进行统一统计，输出：

- 检测点数；
- 距离、速度、方位、俯仰的均值和标准差；
- 绝对值 P95；
- 最大距离和最大功率；
- 后续可扩展的距离分箱密度。

## 使用方式

```powershell
python -m simulation.run_v04_point_cloud_stats `
  --input ss0 simulation\...\ss0\point_cloud.h5 `
  --input ss1 simulation\...\ss1\point_cloud.h5 `
  --input ss2 simulation\...\ss2\point_cloud.h5 `
  --input ss3 simulation\...\ss3\point_cloud.h5 `
  --output simulation\stages\v04_aoa_cfar_point_cloud\output\v04_24_point_cloud_stats
```

## 解释边界

点云数量不是目标检测概率；角度标准差也不是硬件角度精度。比较海况时必须固定雷达 CFG、CFAR 参数、采集时长、船体姿态和坐标系。
