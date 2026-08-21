# V0.4.4 PCB 天线坐标提取记录

本次分析使用：

```text
C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_ASCII.PcbDoc
```

该文件是 Altium ASCII PCB 数据，不是普通图片。文件中明确存在 TX1～TX4、RX1～RX4 网络，
并存在对应的 `Region` 铜区几何。已提取 8 个 Region 的 72 个顶点、包围盒和几何中心。

## 提取结果

RX 阵列候选中心沿一条方向排列，中心间距约 1.900 mm；77 GHz 波长约 3.893 mm，因此约为
0.488 λ。TX 阵列候选中心包含约 3.918 mm 和 7.838 mm 级别的间隔，体现出非简单均匀半波长
阵列。实际数值见 `simulation/hardware/awr2944pev/pcb_antenna_regions.csv`。

随后以 TX/RX 铜区几何中心相加、以 TX1+RX1 为原点，生成 16 个虚拟通道相对坐标：

```text
simulation/hardware/awr2944pev/cad_virtual_array_coordinates.csv
```

## 物理含义和限制

这些坐标已经比“理想半波长阵列”更接近 EVM PCB，但铜区几何中心不等于天线电气相位中心。
天线的有效相位中心还受馈电点、走线、参考地、介质叠层、铜形状、封装、雷达罩和频率影响。
因此文件中的状态明确标为：

```text
cad_copper_centroid_sum_approximation_not_phase_center
```

可以用于阵列布局敏感性分析、空间混叠分析和 AoA 误差回归；不能直接作为实测 AoA 精度或最终
自定义阵面设计依据。下一步应把该坐标接入 V0.4 AoA 扫描，并与理想半波长阵列比较 RMSE、
栅瓣和 FOV；最终仍需角反射器校准或全波电磁仿真确认。
