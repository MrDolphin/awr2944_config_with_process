# 雷达—相机接收时钟模拟

`radar_camera_sync_scenarios.py` 为同步算法提供确定性的离线时间线测试。它不启动相机、FFmpeg、雷达串口、GPIO 或云台；仅构造带接收时间戳的合成 `CameraFrame`，并复用生产中的 `CameraFrameBuffer` 与 `match_camera_frame`。

## 模型边界

对每一条雷达时间戳，模拟器只会把**接收时间不晚于该雷达帧**的相机帧放入缓冲区。未来相机帧不会被用于改善历史雷达帧的匹配，因此它符合实时服务的可见性约束。

结果包含：

- 每个雷达帧的 `matched`、`degraded`、`stale` 或 `unavailable` 状态；
- 匹配帧 ID 与有符号 `camera_time - radar_time` 偏差；
- 各状态数量、匹配比例和仅针对 `matched` 帧的绝对偏差 p95；
- 在最后一个雷达帧时刻仍保留在有界相机缓冲中的帧 ID。

## 在测试中使用

```python
from radar_camera_sync_scenarios import run_timing_scenario

report = run_timing_scenario(
    camera_timestamps_ns=[0, 33_000_000, 66_000_000, 200_000_000],
    radar_timestamps_ns=[20_000_000, 53_000_000, 140_000_000, 220_000_000],
)
```

时间戳必须是严格递增的非负整数。`capacity`、`matched_limit_ms` 和 `stale_limit_ms` 可按明确的设计试验修改。

## 不可替代的实机门禁

模拟器只能验证同步状态机、阈值和缓冲区语义。它不能证明树莓派的相机帧率、CPU 温度、内存趋势、FFmpeg 重启、雷达丢帧、网络传输时延、空间标定精度或云台安全行为。上述结果仍必须在 Task 11 的上电验收中独立记录。
