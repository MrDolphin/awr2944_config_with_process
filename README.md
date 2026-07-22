# AWR2944 船载雷达点云项目

本仓库用于 AWR2944 雷达在树莓派上的 WebSocket 服务、网页 PPI 点云显示、离线回放分析和后续船载避障/泊船验证。

## 当前开发分支

当前主要开发分支：

```bash
feat/codex
```

第一次接收代码时建议先确认分支：

```bash
git clone https://github.com/MrDolphin/awr2944_config_with_process.git
cd awr2944_config_with_process
git checkout feat/codex
git status --short
```

说明：

- 本分支包含网页端 `radar_app.html`、树莓派服务端 `radar_server.py`、离线回放分析、目标簇稳定性和最近表面距离显示等功能。
- 当前仓库是开发源。树莓派上可以不安装 Git，通过 `scp` 从电脑同步文件。
- 不要在未确认硬件安全边界前启动电机、修改 GPIO 极性或更改运动安全参数。

## 目录要点

```text
radar_app.html                 # 网页端 PPI/离线回放界面
radar_server.py                # 树莓派 WebSocket + 雷达串口服务
radar_replay.py                # 离线采集列表、帧读取和分析
radar_runtime.py               # 运行时配置、采集记录等基础能力
radar_control.py               # 雷达配置保存/读取/下发控制
radar_health.py                # 服务健康状态
Config/                        # 雷达 cfg 配置文件
captures/pointcloud_logs/      # 默认点云采集目录
deploy/radar.service           # systemd 服务模板
deploy/setup_rpi.sh            # 树莓派依赖和服务安装脚本
tools/deployment_preflight.py  # 只读部署预检脚本
test/                          # 自动化测试
```

## 网页端部署

网页端是单文件 `radar_app.html`。如果只修改了网页界面，不需要重启树莓派服务，只需覆盖 HTML 并强制刷新浏览器。

当前已知树莓派地址：

```text
http://172.20.10.10:8765
```

从 Windows 开发机同步网页文件到当前直部署目录 `/home/pi`：

```powershell
scp "D:\hp-laptop\USV\awr2944_config_and_process_with_trace_codex\radar_app.html" pi@172.20.10.10:/home/pi/radar_app.html
```

浏览器打开或强制刷新：

```text
http://172.20.10.10:8765
```

如果树莓派使用推荐项目目录 `/home/pi/awr2944_config_with_process`，则改为：

```powershell
scp "D:\hp-laptop\USV\awr2944_config_and_process_with_trace_codex\radar_app.html" pi@172.20.10.10:/home/pi/awr2944_config_with_process/radar_app.html
```

## 树莓派设备端部署

### 1. 准备部署目录

如果当前继续使用 `/home/pi` 直部署模式，只同步必要文件即可：

```powershell
scp "D:\hp-laptop\USV\awr2944_config_and_process_with_trace_codex\radar_server.py" pi@172.20.10.10:/home/pi/radar_server.py
scp "D:\hp-laptop\USV\awr2944_config_and_process_with_trace_codex\radar_app.html" pi@172.20.10.10:/home/pi/radar_app.html
scp "D:\hp-laptop\USV\awr2944_config_and_process_with_trace_codex\radar_replay.py" pi@172.20.10.10:/home/pi/radar_replay.py
scp "D:\hp-laptop\USV\awr2944_config_and_process_with_trace_codex\radar_runtime.py" pi@172.20.10.10:/home/pi/radar_runtime.py
scp "D:\hp-laptop\USV\awr2944_config_and_process_with_trace_codex\radar_control.py" pi@172.20.10.10:/home/pi/radar_control.py
scp "D:\hp-laptop\USV\awr2944_config_and_process_with_trace_codex\radar_health.py" pi@172.20.10.10:/home/pi/radar_health.py
```

如果是新机器，推荐使用独立目录，避免把开发文件和用户主目录混在一起：

```bash
mkdir -p /home/pi/awr2944_config_with_process
```

然后把仓库文件同步到该目录，并在安装服务时设置：

```bash
sudo APP_DIR=/home/pi/awr2944_config_with_process ./deploy/setup_rpi.sh
```

如果当前服务已经配置为 `/home/pi`，安装脚本应使用：

```bash
sudo APP_DIR=/home/pi ./deploy/setup_rpi.sh
```

### 2. 安装运行依赖和 systemd 服务

在树莓派项目目录下执行：

```bash
sudo APP_DIR=/home/pi ./deploy/setup_rpi.sh
```

脚本会做以下事情：

- 安装 `python3`、`python3-serial`、`python3-websockets`
- 安装 `deploy/radar.service` 到 `/etc/systemd/system/radar.service`
- 设置服务开机可用
- 不会修改 Wi-Fi、IP、网关等网络配置

### 3. 部署前只读预检

硬件未接入时：

```bash
python3 tools/deployment_preflight.py --config Config/<profile>.cfg
```

雷达 USB 已接入后：

```bash
python3 tools/deployment_preflight.py --config Config/<profile>.cfg --require-ports
```

默认检查项包括：

- Python 依赖：`serial`、`websockets`
- 配置文件是否存在且为 `.cfg`
- 采集目录是否可写
- 剩余磁盘空间是否足够
- `/dev/ttyACM0` 和 `/dev/ttyACM1` 是否存在
- WebSocket 端口 `8765` 是否可绑定

### 4. 启动和检查服务

启动服务：

```bash
sudo systemctl start radar.service
```

查看状态：

```bash
sudo systemctl status radar.service --no-pager
```

查看日志：

```bash
journalctl -u radar.service -n 80 --no-pager
```

持续查看日志：

```bash
journalctl -u radar.service -f
```

重启服务：

```bash
sudo systemctl restart radar.service
```

## 当前直部署模式说明

已有现场部署曾直接放在：

```text
/home/pi
```

常见文件包括：

```text
/home/pi/radar_server.py
/home/pi/radar_app.html
/home/pi/captures/
/home/pi/record/
/home/pi/tools/
```

如果 `systemctl status radar.service` 显示服务正在从 `/home/pi/radar_server.py` 启动，则不要只把文件传到 `/home/pi/awr2944_config_with_process`，否则不会影响当前运行服务。

确认服务实际命令：

```bash
ps -ef | grep radar_server
systemctl cat radar.service
```

## 常用访问和排错

SSH：

```bash
ssh pi@172.20.10.10
```

网页：

```text
http://172.20.10.10:8765
```

查看服务是否占用端口：

```bash
ss -lntp | grep 8765
```

查看雷达 USB：

```bash
ls -l /dev/ttyACM*
```

查看采集目录空间：

```bash
du -xhd1 /home/pi | sort -h
df -h
```

## 开发和验证

本地运行测试：

```bash
python -m unittest discover -s test -v
```

网页端单文件变更通常只需要：

1. 本地检查 JavaScript 语法或运行测试。
2. `scp radar_app.html` 到树莓派。
3. 浏览器强制刷新。

服务端 Python 变更通常需要：

1. 同步相关 `.py` 文件。
2. `sudo systemctl restart radar.service`
3. 查看 `journalctl -u radar.service -n 80 --no-pager`

## 安全边界

- 不要在无人看守时启动电机扫描。
- 不要在未标定前假设编码器计数、转向极性、安全端点或线束余量。
- 不要把离线回放中显示的候选簇直接当成避障真值；应优先使用稳定簇、最近表面距离和现场实测距离做交叉验证。
- 任何实际泊船/避障策略上线前，应保留人工接管和物理急停方案。
