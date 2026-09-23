# 雷达—相机融合现场验收记录：雷达单独门禁

**时间：** 2026-09-23 09:52–09:54 CST  
**状态：** 阻塞；相机与云台未启用。  
**目的：** 将受管检出目录切换到新的雷达单独服务，确认迁移本身不启动相机，并验证点云 WebSocket 基线。

## 身份与部署证据

| 项目 | 实测结果 |
| --- | --- |
| Pi 主机 | `raspberrypi`，通过 `pi@172.20.10.10` 连接 |
| 受管检出目录 | `/home/pi/camera_web_fusion` |
| 分支 / Git commit | `codex/radar-camera-web-fusion` / `3ac3b69b33d50dbbdd9c744727b56aba82f44f18` |
| 原服务基线 | `/home/pi/radar_server.py --cfg_port /dev/ttyACM0 --data_port /dev/ttyACM1 --cfg_file /home/pi/profile-2944.cfg --ws_port 8765` |
| 安装 | `sudo env APP_DIR=/home/pi/camera_web_fusion bash ./deploy/setup_rpi.sh`；安装脚本启用但未自行启动服务 |
| 新服务 | `WorkingDirectory=/home/pi/camera_web_fusion`；`ExecStart=/usr/bin/python3 radar_server.py --ws_port 8765 $RADAR_CAMERA_ARGS` |
| systemd 静态验证 | `systemd-analyze verify /etc/systemd/system/radar.service` 成功 |
| 相机默认策略 | `/etc/default/radar-camera` 中 `RADAR_CAMERA_ARGS=` |

## 雷达单独运行结果

`sudo systemctl restart radar.service` 后，服务进程为 `/usr/bin/python3 radar_server.py --ws_port 8765`。检查到：

| 判定项 | 实测结果 | 结论 |
| --- | --- | --- |
| 服务状态 | `active`，WebSocket `0.0.0.0:8765` 监听 | 通过 |
| 串口映射 | 配置 `/dev/ttyACM0`；数据 `/dev/ttyACM1`；两者已连接 | 通过 |
| 相机/FFmpeg | 未监听 `8081`；无 FFmpeg 进程；相机参数为空 | 通过 |
| 云台动作 | 未发送 GPIO、伺服或云台命令 | 未执行，符合本门禁范围 |
| WebSocket 接收 | 可连接；返回初始 `frame_num: 0` | 部分通过 |
| 数据流 | `data_status=waiting_for_frame`、`bytes_received=0`、`frames_parsed=0`、`active_config.name=null` | **未通过** |

环境记录：温度 `51.6°C`，内存可用约 `1.6 GiB`，根分区可用约 `39 GiB`。`vcgencmd get_throttled` 为 `0x50000`，表示设备曾记录欠压和降频事件；本次不据此判定当前存在故障，但相机性能测试前后必须再次记录并确认没有新增事件。

## 阻塞与下一步

服务不会后台自动下发雷达配置，因此迁移后不应通过重启自动恢复输出。要让点云重新进入 WebSocket，操作员需要明确确认要在网页端下发的配置文件及其现场安全性；下发后再验证 `bytes_received > 0`、`frames_parsed > 0` 和页面点云渲染。

相机、同步 JPEG、点云投影和云台验收均保持未运行。只有雷达单独数据流恢复并稳定后，才能进入相机单独 30 分钟门禁。
