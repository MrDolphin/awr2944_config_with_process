# 树莓派部署与实机联调清单

部署前先在项目目录执行：

```bash
python3 tools/deployment_preflight.py --config Config/<profile>.cfg
```

硬件已接入后增加 `--require-ports`，它会把缺少的 `/dev/ttyACM*` 端口视为失败。预检只读检查依赖、配置、采集目录、磁盘空间、串口路径与 WebSocket 端口；不会改网络或服务。

实机联调顺序：

1. 固定雷达与电源，确认天线视场无船体遮挡。
2. 接入 USB，执行预检并记录输出。
3. 启动服务，网页确认“数据流正常”；无帧时应显示“等待雷达帧”或“数据帧超时”。
4. 录制至少 60 秒空场与已知目标场景，确认 `dropped_frames=0`、`writer_error` 为空。
5. 用离线回放检查空帧、目标帧和配置快照；保留 capture 目录作为试验留档。

若串口掉线，先查看 HUD 最近错误和 `journalctl -u radar -f`；服务会对 I/O 错误尝试重连，但电源、USB 线和端口映射仍需现场确认。
