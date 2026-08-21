# V0.4.32 DCA1000/校准数据就绪性检查

## 目的

为真实 AWR2944P AoA 验证建立一个自动验收闸门，检查：

- DCA1000 解码/虚拟通道 HDF5 是否存在并可读；
- 虚拟通道顺序是否已经验证；
- 校准矩阵是否为 4×4；
- 校准数据是否来自实测，而不是合成 fixture；
- 对应 CFG 是否存在。

## 当前项目结果

本次使用现有的合成校准文件和 CFG 快照运行检查：

| 检查项 | 结果 |
|---|---|
| DCA1000 捕获 HDF5 | 未提供 |
| 通道顺序已验证 | 未通过 |
| 实测校准矩阵 | 未通过 |
| 校准矩阵 4×4 形状 | 通过 |
| CFG 快照 | 通过 |
| 真实 AoA 就绪 | 否 |

现有 `synthetic_calibration.json` 的状态是：

```text
synthetic_injected_error_not_measured
```

因此它只能用于软件回归和校准方向测试，不能证明硬件校准已经完成。

## 输出

结果目录：

```text
simulation/hardware/awr2944pev/v04_32_calibration_readiness/
```

- `readiness_summary.json`：机器可读的验收结果；
- `output_analysis.md`：本次检查的解释。

脚本：

```text
simulation/run_v04_32_calibration_readiness.py
```

使用真实数据时：

```powershell
python -m simulation.run_v04_32_calibration_readiness `
  --capture path\known_angle_capture.h5 `
  --calibration path\measured_calibration.json `
  --cfg Config\profile_3d_3Azim_1ElevTx_awr2944P.cfg `
  --output simulation\hardware\awr2944pev\v04_32_calibration_readiness
```

## 结论

当前代码已经具备真实数据接入前的自动检查能力，但项目没有把“文件存在”误判为“真实 AoA 已验证”。必须同时满足已知角捕获、通道顺序验证、实测校准矩阵和 CFG 一致性，才允许进入真实阵列 AoA 结论阶段。
