# V0.4.18 复数通道校准接口与 AoA 扫描

## 数据契约

每个 4×4 虚拟通道使用：

```json
{
  "amplitude": [[...], [...], [...], [...]],
  "phase_deg": [[...], [...], [...], [...]],
  "channel_order": "rx0..rx3 rows, tx0..tx3 columns"
}
```

`complex_matrix(amplitude, phase_deg)` 生成复数通道因子，`apply_channel_correction` 将补偿因子乘到通道数据上。真实数据接入前必须明确因子是“原始通道误差”还是“补偿因子”，避免把方向反过来。

## 合成验证

本阶段注入可重复的幅度和相位误差，比较：

- 不校准；
- 理想逆补偿；
- 错误的半相位补偿。

结果目录：`simulation/stages/v04_aoa_cfar_point_cloud/output/v04_18_calibration_scan/`。

当前扫描结果为：无校准方位/俯仰 RMSE 39.0193°/20.7787°；理想逆补偿 38.3208°/22.6125°；错误半相位补偿 38.3210°/22.6126°。理想逆补偿恢复到“无注入相位误差”的合成基线，说明补偿乘法方向和通道顺序接口工作；剩余大误差来自当前 PCB/CFG 坐标模型，不是校准接口本身。

理想逆补偿恢复到合成基线，只能证明接口乘法方向和通道顺序契约正确。它不能证明 TI EVM 的真实校准参数，也不能替代 `measureRangeBiasAndRxChanPhase` 实测流程。
