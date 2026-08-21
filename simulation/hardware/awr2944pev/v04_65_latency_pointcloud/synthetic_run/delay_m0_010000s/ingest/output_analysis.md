# V0.4.63 IMU 数据接入与时间归一化

原始时间字段：`timestamp`；原始单位：`ms`；换算到秒的比例：0.001。
固定延迟修正：-0.010000 s。约定为 `effective_time_s = raw_timestamp * scale + latency_correction_s`。
原始 IMU 样本：3；对齐质量门：True。

## 如何分析

先检查 `normalized_imu.csv`，确认时间是否已经转换成秒并应用延迟修正；再检查 `alignment/aligned_imu.csv` 和 `alignment/summary.json`。只有当 coverage、单调性和最大间隔均通过时，才能把对齐姿态送入 V0.4.61。

## 证据边界

当前示例用于验证单位和延迟逻辑，`source_is_hardware_imu=false`。真实数据接入时必须把 timestamp 来源、设备时钟、延迟测量方法和符号约定一并记录，不能仅凭文件名猜测单位。
