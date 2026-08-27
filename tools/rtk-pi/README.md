# rtk-pi

树莓派 4B 版 UM982/WTRTK-982 NTRIP 移动站采集程序。独立于 PC 端 `tools/rtk`。

## 功能

- 支持 `SINGLE`、`DGPS`、`RTK` 三种目标模式。
- 启动时安全探测 Linux 串口，默认跳过雷达占用的 `/dev/ttyACM0` 和 `/dev/ttyACM1`。
- 探测到 UM982 后把设备路径写回 `config.ini` 的 `[SERIAL].port`，再开始采集。
- 支持手动固定串口：`auto_on_start=false`，或运行时使用 `--port /dev/ttyUSB0 --save`。
- 连接千寻 NTRIP，自动注入 RTCM，并周期回发最新 GGA。
- 输出 `gnss_raw.log`、`position.csv`、`status.log` 和带完成状态的 `manifest.json`。
- 处理 `SIGTERM`，可直接交给 systemd 管理。

## 安装

```bash
cd awr2944_config_with_process/tools/rtk-pi
python3 -m pip install -r requirements.txt
sudo usermod -aG dialout $USER
```

重新登录后让 `dialout` 组生效。

## 配置

复制模板后修改：

```bash
cp config.example.ini config.ini
nano config.ini
```

当前目录中的 `config.ini` 已被 `.gitignore` 排除，适合放千寻账号等本机信息。

串口策略：

```ini
[SERIAL]
port = auto
auto_on_start = true
save_detected_port = true
exclude_ports = /dev/ttyACM0, /dev/ttyACM1
```

手动固定串口：

```ini
port = /dev/ttyUSB0
auto_on_start = false
```

## 运行

无硬件检查：

```bash
python3 rtk_pi.py --check
```

列出候选串口：

```bash
python3 rtk_pi.py --list-ports
```

自动探测并持续采集：

```bash
python3 rtk_pi.py
```

临时指定模式或时长：

```bash
python3 rtk_pi.py --mode SINGLE --duration 20
python3 rtk_pi.py --mode DGPS --duration 60
python3 rtk_pi.py --mode RTK --duration 300
```

手动指定并保存：

```bash
python3 rtk_pi.py --port /dev/ttyUSB0 --save
```

## systemd 示例

```ini
[Unit]
Description=UM982 RTK Pi collector
After=network-online.target
Wants=network-online.target

[Service]
User=pi
WorkingDirectory=/home/pi/awr2944_config_with_process/tools/rtk-pi
ExecStart=/usr/bin/python3 /home/pi/awr2944_config_with_process/tools/rtk-pi/rtk_pi.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

## SINGLE/DGPS/RTK 状态说明

`config.ini` 中的 `MODE.type=RTK` 是目标模式和接收机初始化策略：

- `SINGLE`：不连接 NTRIP，关闭 DGPS/RTK 引擎。
- `DGPS`：连接 NTRIP 并注入改正数，开启 DGPS 引擎。
- `RTK`：连接 NTRIP 并注入改正数，开启 DGPS 和 RTK 引擎。

程序打印 `Mode: RTK` 不表示当前定位质量已经是 RTK。实际状态以 GGA quality 为准。如果长时间停在 `SINGLE`，先看日志中的 NTRIP 计数：

```text
[ntrip] first correction payload
[state] ... rtcm=N
```

若一直是 `rtcm=0`，说明 caster 没有下发差分数据。常见原因包括 GGA 未被接受、账号无该接入点权限、挂载点不可用或网络运营商拦截。当前版本会使用 CRLF 结尾回发最新 GGA，并在 HTTP 响应头后保留可能已经到达的首段 RTCM 载荷。
