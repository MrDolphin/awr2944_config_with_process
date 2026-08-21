# V0.4.23 CA-CFAR 三维点云

## 处理链

```text
距离-多普勒功率图
    ↓
2D CA-CFAR
    ↓
局部峰值筛选
    ↓
峰值单元 4×4 AoA
    ↓
(range, velocity, azimuth, elevation, power, x, y, z)
```

默认参数：`Pfa=1e-3`、距离/速度训练单元 `(4,4)`、保护单元 `(1,1)`。

## 输出

结果目录：`simulation/stages/v04_aoa_cfar_point_cloud/output/v04_23_cfar_point_cloud/`。

HDF5 中每个 `/point_cloud/<field>` 是一个点云字段，JSON 保存检测参数和完整点列表。

## 边界

- 这是 CA-CFAR，不是 TI 固件 CFAR 的逐项复刻；
- 最大峰和局部峰不能自动区分海杂波与目标；
- 三维坐标依赖当前坐标系约定；
- `channel_order_verified=false` 时，点云角度不能当作实板精度。
