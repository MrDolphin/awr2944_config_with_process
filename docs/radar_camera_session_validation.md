# 同步记录包离线验收

`radar_camera_session_validation.py` 只读取已完成的同步记录目录。它不会启动雷达、相机、FFmpeg、串口、GPIO 或云台，因此可在设备断电时开发和复核，也可在采集完成后从 PC 或树莓派运行。

## 运行

在包含记录目录的仓库检出中执行：

```bash
python3 radar_camera_session_validation.py /path/to/radar_camera_YYYYMMDD_HHMMSS
```

默认门限与 Task 11 的静态联合运行门限一致：

- 已匹配雷达帧比例不少于 `0.95`；
- 已匹配帧的绝对接收时钟偏差 p95 不大于 `50 ms`。

可针对一次明确记录的实验门限覆盖默认值：

```bash
python3 radar_camera_session_validation.py \
  /path/to/radar_camera_YYYYMMDD_HHMMSS \
  --min-matched-ratio 0.95 \
  --max-absolute-offset-p95-ms 50
```

输出是 JSON，可直接保存为验收证据：

```bash
python3 radar_camera_session_validation.py /path/to/radar_camera_YYYYMMDD_HHMMSS \
  > session-validation.json
```

退出码含义：

- `0`：记录包完整，且两项同步门限通过；
- `1`：记录包完整，但匹配比例或 p95 时延未达门限；
- `2`：记录包不完整或不一致，例如引用的 JPEG、雷达行、索引列或必要元数据缺失。

## 检查范围

验证器要求并交叉检查：

- `session_metadata.json` 中的完成状态、时钟基准、Git、雷达配置、相机配置和同步门限；
- `radar_frames.jsonl` 与 `fusion_index.csv` 的行数及雷达帧号对应关系；
- 每一个 `camera_frame_id` 所引用的 `camera_frames/<id>.jpg` 是否存在；
- `matched` 行的匹配比例和绝对 `time_offset_ms` p95。

该结果只证明已保存文件的完整性和树莓派接收时间轴上的同步统计。它不证明相机实际帧率、雷达丢帧、空间标定精度、目标识别结果或云台行为；这些仍需按 Task 11 在上电且操作员在场的条件下单独记录。
