# V0.3 复数回波与Range-Doppler

状态：V0.3.4 三海况加权多散射对比已生成；输入必须引用
一个通过验收且不可覆盖的 V0.2 动态海面运行目录。本阶段加入 AWR2944P
4TX/4RX FMCW 复数回波、距离处理和多普勒处理，但暂不输出 AoA、CFAR 或三维
检测点云。

结果保存到 `results/python/<run_id>/` 或 `results/matlab/<run_id>/`，并记录所
引用的 V0.2 `run_id`、CFG、虚拟阵列顺序、窗函数和 FFT 参数。

## V0.3.1 当前基线

当前 Python 最小模型使用：

| 参数 | 值 |
|---|---:|
| 载频 | 77 GHz |
| FMCW 带宽 | 1 GHz |
| Chirp 时长 | 60 µs |
| ADC 采样率 | 25 MSPS |
| 每 Chirp 采样点 | 256 |
| 每帧 Chirp 数 | 64 |
| Chirp PRI | 100 µs |
| 发射/接收通道 | 4 TX × 4 RX |
| 距离分辨率 | 约 0.15 m |
| 速度分辨率 | 约 0.304 m/s |

复数 IQ 合同为 `(chirp, sample, rx, tx)`，当前单散射微元的 16 个通道使用
相干同相响应；它们还不是带有真实虚拟阵列相位差的 AoA 输入。

## 已验证运行

运行器：`simulation/run_v03.py`。示例：

```powershell
python -m simulation.run_v03 `
  --input-truth simulation/stages/v02_dynamic_sea_truth/results/python/v02b_forward_seed101_hs1m_analysis/data/ss2_normal_seed101_truth.h5 `
  --results-root simulation/stages/v03_complex_echo_range_doppler/results `
  --run-id v03_single_ss2_seed101
```

已验证结果目录：

```text
results/python/v03_single_ss2_seed101/
```

该运行从 V0.2 `ss2_normal` 的一个海面网格单元读取斜距和斜距变化率，生成
单个散射微元 IQ，并保存 `data/single_scatterer.h5`、`summary.json` 和
`validation.md`。实测峰值为：真值斜距约 50.049 m，Range-Doppler 峰值约
50.063 m，距离误差约 0.014 m；真值速度约 0.0005 m/s，小于当前速度分辨率，
因此速度峰落在 0 m/s bin 是预期结果。

## 当前边界

V0.3.1 只验证单个散射微元的 FMCW beat、距离 FFT 和多普勒 FFT。它不代表：

- 完整海杂波功率统计；
- 多海面微元叠加；
- 水质点三维轨道速度；
- 实际天线方向图和虚拟阵列相位；
- AoA、CFAR 或点云；
- 实测 AWR2944P 性能。

## V0.3.3 加权多散射微元

运行器：`simulation/run_v03_weighted_multiscatterer.py`。当前权重为归一化
`1/R²` 幅度，并可将局部掠射角不大于 0° 的微元置零。这里的“正面照射”只是
局部法向代理，不是全局射线遮挡；也没有加入介电常数、极化、海面复反射系数
或实测 RCS。因此该版本只用于比较权重模型对 Range-Doppler 形状的影响，不能
直接解释为绝对海杂波功率或探测距离。

每次运行仍需在运行目录保存 `summary.json`、`data/*.h5`、图像和
`output_analysis.md`。分析顺序是：先核对输入工况和散射点数量，再看距离方向
的亮区/展宽，再看速度方向的亮区/展宽，最后报告 FFT 分辨率和模型边界。
