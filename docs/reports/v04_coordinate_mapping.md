# V0.4.12 PCB 坐标与 CFG 阵列映射

本阶段将 `cad_virtual_array_coordinates.csv` 中的 16 个 TX/RX 虚拟通道，与 `antgeometry_mapping.csv` 展开的 `antGeometryCfg` 理想行列坐标逐通道对齐。

## 比较方式

在 77 GHz 下使用波长约 3.8934 mm，将 CFG 的 `column × 0.5λ` 作为理想方位坐标，将 `row × 0.8λ` 作为理想俯仰坐标，再与 PCB 铜区几何中心和原点平移后的坐标比较。没有偷偷进行旋转、镜像或缩放。

## 结果如何理解

逐通道结果保存在：

`simulation/stages/v04_aoa_cfar_point_cloud/output/v04_12_coordinate_mapping/coordinate_mapping.csv`

汇总和 RMS 差保存在同目录的 `summary.json` 与 `output_analysis.md`。这些 RMS 差只是在检查“坐标定义是否一致”，不是 AoA RMSE，也不是天线方向图误差。

本次实际输出显示：X 方向 RMS 差约 0.0693 mm，说明当前 TX/RX 铜区中心的方位向间距与 `0.5λ` 理想网格相当接近；Y 方向 RMS 差约 3.1198 mm，接近 `0.8λ`，且逐通道表现为 CFG 的 row=1 通道在 PCB 上位于 y≈0、CFG 的 row=0 通道在 PCB 上位于 y≈3.135 mm。这更像是 PCB 坐标的 Y 方向与 CFG 行定义存在交换、镜像或阵面朝向差异，不能直接当成天线尺寸误差。

## 工程结论

- 如果 PCB 坐标与 CFG 理想网格存在平移、旋转或镜像差异，首先应修正板面坐标系和安装姿态；
- 如果经过刚体配准后仍存在毫米级非规则残差，说明铜区几何中心不能代表电气相位中心；
- 只有馈电点、相位中心或角反射器校准可用时，才应把该模型接入正式 AoA 精度结论；
- 当前结果仍标记为 `pcb_centroid_is_not_electrical_phase_center`。
