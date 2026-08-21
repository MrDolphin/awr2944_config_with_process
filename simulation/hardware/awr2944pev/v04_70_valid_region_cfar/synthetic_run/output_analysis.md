# V0.4.70 CFAR 有效区域证据

每个海况/CFAR 场景在可评估区域内取最高 20 个功率单元。

## 统计字段

`above_threshold_count` 只反映功率门限条件；`local_maximum_count` 只反映邻域峰值条件；`would_detect_count` 同时满足两者。若有效区域内仍然 `would_detect_count=0`，才有理由继续检查阈值或峰值规则，而不是把边界效应当作漏检。

## 证据边界

输入仍是合成距离-多普勒谱；本阶段区分了 CFAR 边界和有效区域，但不代表 TI SDK 或实板 CFAR 实现。
