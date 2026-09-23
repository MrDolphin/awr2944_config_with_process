# 雷达—相机融合现场验收记录：相机启动与帧同步

**时间：** 2026-09-23 11:56–11:58 CST
**状态：** 相机采集、HTTP 帧服务与雷达帧匹配通过；浏览器主观画质、持续运行与空间标定仍待验收。

## 启动范围

操作员明确要求启动摄像头服务并验收同步画面。服务以以下显式参数运行：

```text
--enable-camera
--camera-config /home/pi/camera_web_fusion/tools/camera/camera_config.cfg
--camera-http-host 0.0.0.0
--camera-http-port 8081
--camera-public-base-url http://172.20.10.10:8081
```

未下发雷达配置；未启动 GPIO、云台、电机或点云—图像空间投影。

## 设备与服务证据

| 判定项 | 实测结果 | 结论 |
| --- | --- | --- |
| UVC 设备 | 稳定路径 `/dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0` | 通过 |
| 采集模式 | MJPEG `1280×720 @ 30 fps` | 通过 |
| 雷达服务 | `radar.service` 为 `active (running)`，端口 `8765` 监听 | 通过 |
| 相机 HTTP | 端口 `8081` 监听；FFmpeg 以 MJPEG 输入、`1280×720@30` 运行 | 通过 |
| Pi 本机 JPEG | `GET /camera/frame/latest.jpg` 返回 `200`、`99,701 B` JPEG、`Cache-Control: no-store` 与帧时间戳头 | 通过 |
| Windows 访问 JPEG | `GET http://172.20.10.10:8081/camera/frame/latest.jpg` 返回 `200`、`99,708 B`、`Access-Control-Allow-Origin: *` | 通过 |
| 相机运行状态 | `frame_count=2,839`，`fps_observed=32.007`，无运行错误 | 通过 |

## 雷达—相机时间匹配

从 Pi 本机 WebSocket `ws://127.0.0.1:8765` 连续采集 12 个雷达帧：

| 项目 | 实测结果 |
| --- | --- |
| 雷达帧号 | `35250` 至 `35261`，连续递增 |
| `camera_sync.status` | 12/12 为 `matched` |
| 相机帧 ID | `2849` 至 `2882`，持续递增 |
| 雷达—相机偏差 | 最小 `-24.629 ms`；中位数 `10.177 ms`；最大 `14.085 ms` |
| 匹配门槛 | `matched ≤ 50 ms` |
| 对外逐帧 URL | `http://172.20.10.10:8081/camera/frame/<id>.jpg` |

该结果证明软件接收时间轴上的帧匹配、端到端 JPEG 可访问性和跨源请求头正常；不等同于硬件触发同步。

## 尚未完成的门禁

1. 在浏览器页面确认相机画面无明显模糊、冻结或可见延迟，并观察至少 10 分钟。
2. 在相同运行期间记录温度、`vcgencmd get_throttled` 与相机 `last_error`，确认无新增欠压/降频或采集异常。
3. 使用真实标定板完成内外参标定后，才可启用并验收点云投影；当前 `calibration_not_loaded` 是预期保护状态。
4. 完成现场静止/移动目标的同步误差验证；当前只验证了系统时间戳匹配，不宣称运动场景空间一致性。
