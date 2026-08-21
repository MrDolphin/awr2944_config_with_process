# V0.4.22 距离-多普勒-峰值单元 AoA

## 处理链

```text
HDF5 virtual IQ
    ↓
距离 FFT（sample 轴）
    ↓
Doppler FFT（TDM frame 轴）
    ↓
最大功率距离-多普勒单元
    ↓
4×4 虚拟通道 AoA
```

输出保留 range-Doppler 功率、复数谱、距离轴、速度轴和峰值单元复数通道。

## 输出

运行器：`simulation/run_v04_range_doppler_aoa.py`。

结果目录：`simulation/stages/v04_aoa_cfar_point_cloud/output/v04_22_range_doppler_aoa/`。

## 边界

- 峰值单元不是 CFAR 检测结果；
- 没有做海杂波/目标分类；
- AoA 仍依赖通道顺序、CFG 阵列坐标和校准；
- `channel_order_verified=false` 时不能宣称真实硬件角度精度。
