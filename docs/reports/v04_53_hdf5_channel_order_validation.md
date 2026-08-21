# V0.4.53 HDF5 已知角度通道顺序验证

## 本阶段目的

将 V0.52 的通道排列扫描接入实际 HDF5 虚拟 IQ 数据结构，支持：

- `/virtual/iq`；
- `/recovered/virtual_iq`；
- `/decoded/iq`。

程序先对 HDF5 做距离-Doppler FFT，在最大峰值单元取 4×4 复数通道，再枚举 `24×24=576` 个 RX/TX 排列，使用已知角度元数据计算 AoA 误差。

## 本次输入

- 文件：V0.4.33 known-angle fixture；
- 数据集：`/recovered/virtual_iq`；
- 真值：方位 `15°`、俯仰 `8°`；
- 几何源：`simulation.v04.virtual_array_positions`；
- 输入来源：`synthetic_known_angle_regression_only`。

## 结果

几何源与数据生成源一致时：

- 身份排列 RX=`0,1,2,3`、TX=`0,1,2,3`；
- 估计方位：`15°`；
- 估计俯仰：`8°`；
- 方位误差：`0°`；
- 俯仰误差：`0°`；
- 相关性得分：约 `1.0`。

错误排列中，有些候选可能只偏 `2°~3°`，所以不能只凭单个角度判断通道顺序。必须使用多个已知角度和真实硬件采集。

## 重要的几何源规则

如果输入是合成 fixture，必须使用 fixture 生成时的几何源；如果输入是 AWR2944P 实测 DCA1000 IQ，必须使用经过 PCB/CAD、CFG 和机械坐标确认的硬件几何。不能用 PCB 候选几何去评判由另一套虚拟几何生成的合成 IQ，否则会把“几何不一致”误判为“通道顺序错误”。

命令示例：

```powershell
# 合成 fixture：不传 --mapping，自动使用 fixture 声明的 simulation.v04 几何
python -m simulation.run_v04_53_hdf5_channel_order_validation `
  --input simulation\hardware\awr2944pev\v04_33_known_angle_fixture\known_angle_az015_el008.h5 `
  --output simulation\hardware\awr2944pev\v04_53_hdf5_channel_order_validation

# 实测硬件 HDF5：必须显式传入硬件阵列映射
python -m simulation.run_v04_53_hdf5_channel_order_validation `
  --input measured_reference.h5 `
  --mapping simulation\hardware\awr2944pev\antgeometry_mapping.csv `
  --output simulation\hardware\awr2944pev\v04_53_hdf5_channel_order_validation\measured
```

## 边界

本次输入不是实测 DCA1000 数据，`channel_order_verified` 仍保持 `false`。真实通道顺序仍需已知角反射器、CFG 联动解码、实测校准矩阵和多角度稳定性共同确认。

## 输出文件

- [hdf5_channel_order_candidates.csv](../../simulation/hardware/awr2944pev/v04_53_hdf5_channel_order_validation/hdf5_channel_order_candidates.csv)
- [summary.json](../../simulation/hardware/awr2944pev/v04_53_hdf5_channel_order_validation/summary.json)
- [output_analysis.md](../../simulation/hardware/awr2944pev/v04_53_hdf5_channel_order_validation/output_analysis.md)

