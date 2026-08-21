# V0.4.34 阵列几何语义与空间混叠扫描

## 扫描模型

本阶段不假设某个模型是真实 EVM 阵列，而是逐对比较候选“实际几何”和“估计几何”：

```text
cfg_raw
cfg_swap_row_column
tx_rx_regular
tx_rx_swapped
cfg_y_mirror
regular_y_mirror
```

扫描范围：

```text
方位：-60°、-40°、-20°、0°、20°、40°、60°
俯仰：-20°、-10°、0°、10°、20°
```

## 实际观察

1. `tx_rx_regular` 与自身匹配时误差接近数值零，证明理想半波长几何的算法回归是自洽的。
2. `cfg_raw` 与自身匹配并不能在整个扫描范围内保证小误差，说明当前 CFG 展开位置存在空间间距/相位展开风险。
3. 不同候选模型之间的误差可能很大；这不是单纯的噪声问题，而是坐标语义、阵元排列或镜像定义不一致的表现。
4. “成对扫描中 RMSE 最小”只能作为筛查结果，不能据此宣布硬件真实几何。

## 结果文件

```text
simulation/hardware/awr2944pev/v04_34_geometry_semantics_scan/
```

- `geometry_semantics_pairwise.csv`：36 个实际/估计模型组合；
- `summary.json`：角度网格和每个实际模型的最佳估计模型；
- `output_analysis.md`：边界说明。

## 结论

V0.4.33 的高角度失败已经被 V0.4.34 扩展为系统性模型比较。当前必须先获得 Altium 网表/封装、机械坐标基准或已知角 DCA1000 实测数据，才能确定 `antGeometryCfg` 的真实 row/column 语义和大角度可用范围。当前扫描结果不能替代实测 AoA 性能。
