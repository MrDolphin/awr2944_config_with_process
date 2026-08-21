# V0.58 实测 AoA 验收包流水线

场景数：3；HDF5 已处理：3；真实硬件就绪：否。

| 场景 | 文件 | 状态 | source hardware | channel verified |
|---|---|---|---|---|
| scene_az005_el002_r020 | scene_az005_el002_r020.h5 | hdf5_aoa_validated_synthetic_or_measured | 否 | 否 |
| scene_az015_el008_r030 | scene_az015_el008_r030.h5 | hdf5_aoa_validated_synthetic_or_measured | 否 | 否 |
| scene_az030_el005_r040 | scene_az030_el005_r040.h5 | hdf5_aoa_validated_synthetic_or_measured | 否 | 否 |

## 解释

`.h5` 场景会直接进入 V0.53 HDF5 通道顺序验证；`.bin` 场景先通过 CFG 联动 V0.51 解码，再重新运行本流水线。即使 AoA 数值回归通过，只要数据 provenance 不是实测或通道顺序未确认，hardware_ready 仍为否。
