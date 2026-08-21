# V0.4.63 IMU 数据接入与时间归一化

## 阶段目的

在 V0.4.62 时间对齐之前，明确处理真实 IMU 常见的时间戳单位和固定延迟。程序支持秒、毫秒、微秒和纳秒，并将原始时间转换为秒，再加上显式的延迟修正后进入 V0.4.62 对齐质量门。

## 时间约定

程序使用：

```text
effective_time_s = raw_timestamp * timestamp_scale + latency_correction_s
```

本阶段的 `latency_correction_s` 是“加到原始时间上的修正量”。正值表示把 IMU 事件时间向后移动；负值表示向前修正。实际项目中必须根据时钟定义和测量方法确认符号，不能凭直觉填写。

## 本次示例

- 原始 IMU 时间单位：毫秒（ms）；
- 转换比例：`0.001`；
- 固定延迟修正：`+0.005 s`；
- 雷达 frame：5 个；
- IMU 样本：3 个；
- 最大 IMU 源间隔：0.05 s；
- 允许最大间隔：0.06 s；
- 时间覆盖：通过；
- 对齐质量门：通过。

## 输出文件

- `simulation/hardware/awr2944pev/v04_63_imu_ingest/synthetic_run/normalized_imu.csv`
- `simulation/hardware/awr2944pev/v04_63_imu_ingest/synthetic_run/alignment/aligned_imu.csv`
- `simulation/hardware/awr2944pev/v04_63_imu_ingest/synthetic_run/alignment/summary.json`
- `simulation/hardware/awr2944pev/v04_63_imu_ingest/synthetic_run/summary.json`
- `simulation/hardware/awr2944pev/v04_63_imu_ingest/synthetic_run/output_analysis.md`

## 如何分析

1. 先看 `normalized_imu.csv`，确认毫秒/微秒等单位已经转换成秒，并确认延迟修正后的首尾时间是否覆盖雷达 frame。
2. 再看 `alignment/aligned_imu.csv`，检查每个 frame 的插值区间和 `alignment_status`。
3. 最后看 `summary.json` 的 `coverage_ok`、`max_source_imu_gap_s`、`max_allowed_gap_s` 和 `quality_pass`。

任何一项不满足，都不应把姿态送入 V0.4.61 的海杂波补偿链路。

## 证据边界

当前数据仍是合成示例，`source_is_hardware_imu=false`。真实接入时需要保留原始时间戳、单位、设备时钟、雷达时钟、延迟测量证据以及坐标轴定义。
