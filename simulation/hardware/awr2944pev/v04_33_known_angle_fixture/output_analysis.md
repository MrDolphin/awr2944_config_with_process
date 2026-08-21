# V0.4.33 已知角度 HDF5 回归夹具

## 夹具性质

本目录中的 HDF5 是软件回归夹具，不是 DCA1000 实测数据：

```text
source_type = synthetic_known_angle_regression_only
calibration_status = synthetic_unity_not_measured
```

它记录了真值方位、俯仰、距离、速度、TX 顺序和 geometry 来源，用于验证输入契约和 AoA 处理器的可重复性。

## 同一 antGeometryCfg 几何下的结果

低角度夹具：

```text
真值：azimuth=5°，elevation=2°
估计：azimuth=5°，elevation=2°
```

这是在当前 `antgeometry_mapping.csv` 几何语义下的数值回归通过，不代表硬件精度。

高角度诊断夹具：

```text
真值：azimuth=15°，elevation=8°
估计：azimuth≈2.30°，elevation≈-0.41°
```

该差异暴露了当前展开几何中较大空间间距和相位展开/空间混叠风险。它不是可以忽略的“随机误差”，说明在进入真实 EVM AoA 前必须继续确认 `antGeometryCfg` 的 row/column 语义、阵元间距和阵面坐标。

## 结论边界

低角度结果证明软件输入、HDF5 元数据和处理器可以闭环；高角度结果证明当前几何解释不能未经筛查地用于整个 FOV。两者都不能替代真实角反射器、DCA1000 原始数据、TI 校准流程或实测天线方向图。
