# V0.4.32 DCA1000/校准数据就绪性

- 真实 AoA 就绪：否

| 检查项 | 结果 |
|---|---|
| capture_readable | 通过 |
| capture_channel_order_verified | 通过 |
| measured_calibration | 未通过 |
| calibration_shape_4x4 | 通过 |
| cfg_present | 通过 |

## 解释

未通过不代表代码错误，只表示当前输入不能支持真实阵列 AoA 结论。尤其是 `channel_order_verified` 和 `measured_calibration` 必须通过实测已知角目标或 TI 校准流程确认，不能用合成 fixture 代替。
