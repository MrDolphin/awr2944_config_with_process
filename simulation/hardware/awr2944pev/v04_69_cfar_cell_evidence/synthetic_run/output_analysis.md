# V0.4.69 高能量单元与 CFAR 决策证据

每个海况取未 CFAR 功率最高的 5 个单元，共 5 个海况、9 组 CFAR 参数。

## 字段解释

`noise_linear` 是训练窗噪声估计，`threshold_linear` 是 CA-CFAR 阈值，`power_to_threshold_db` 是单元功率相对阈值的 dB 比值，`above_threshold` 和 `local_maximum` 分别表示两个保留条件，`would_detect` 表示该单元在该场景下是否会被保留。

## 证据边界

高能量单元若位于 CFAR 不可评估边界，`valid_for_cfar=false`，不能据此判断算法漏检；输入仍是合成距离-多普勒谱，不是实测 IQ。
