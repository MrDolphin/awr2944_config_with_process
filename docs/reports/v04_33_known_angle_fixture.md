# V0.4.33 已知角度 HDF5 回归夹具与几何一致性诊断

## 目的

在真实 DCA1000 `.bin` 尚未提供前，建立一个带有明确真值和元数据的 HDF5 输入契约，用于验证：

- `/decoded/iq` 和 `/recovered/virtual_iq` 数据集结构；
- TX TDM 顺序元数据；
- 通道顺序标志；
- 真值方位/俯仰/距离/速度元数据；
- V0.4.22 距离-多普勒-峰值单元 AoA 处理链。

## 重要诊断结果

使用同一份 `antgeometry_mapping.csv` 生成和估计时：

| 夹具 | 真值 | AoA 估计 | 解释 |
|---|---|---|---|
| 低角度 | `(5°, 2°)` | `(5°, 2°)` | 局部相位斜率回归通过 |
| 高角度 | `(15°, 8°)` | `(约 2.30°, -0.41°)` | 暴露空间间距/相位展开混叠风险 |

高角度结果非常重要：它说明当前 `antGeometryCfg` 展开坐标不能直接假设在整个方位/俯仰 FOV 内都能被简单相位展开正确解释。必须确认 row/column 语义、阵元间距和阵面坐标，之后才可以把 AoA 扫描扩展到大角度。

## 输出目录

```text
simulation/hardware/awr2944pev/v04_33_known_angle_fixture/
```

包含：

- `known_angle_az015_el008.h5`：高角度诊断夹具；
- `known_angle_cfg_geometry.h5`：同一 CFG 几何的高角度夹具；
- `known_angle_cfg_geometry_low.h5`：低角度回归夹具；
- `range_doppler_cfg_geometry_low.h5`：低角度 AoA 处理输出；
- 各 HDF5 对应的 JSON 元数据；
- `output_analysis.md`：目录级分析。

## 结论边界

这些数据全部标记为 `synthetic_known_angle_regression_only`，不代表实测。低角度通过只能证明代码契约一致；高角度失败说明几何解释存在待解决问题，不能被包装成真实硬件性能。
