# V0.4.19 CFG 校准参数解析

## 解析规则

将 TI CFG 命令：

```text
compRangeBiasAndRxChanPhase <rangeBias> <real0> <imag0> ... <real15> <imag15>
```

解析为项目使用的 4×4 矩阵。输入顺序保留为 `Tx0Rx0, Tx0Rx1, ..., Tx3Rx3`，输出矩阵索引为 `[rx][tx]`，每个复数值同时转换为 `amplitude` 和 `phase_deg`。

## 当前配置结果

当前项目 CFG 的 32 个实部/虚部值为单位复数序列：

```text
1 + j0
```

因此解析状态为：

```text
cfg_identity_not_measured
```

这只表示 CFG 当前没有施加非单位补偿，不能证明已经执行过 TI 的 `measureRangeBiasAndRxChanPhase` 实测校准。

输出：

`simulation/stages/v04_aoa_cfar_point_cloud/output/v04_19_cfg_calibration/parsed_calibration.json`

## 边界

当前工具将 CFG 值按“直接复数校正因子”解释，但 TI SDK 固件具体应用方向仍需要结合 SDK 文档和真实角反射器结果确认；必要时应对矩阵取复数逆后进行 A/B 验证。
