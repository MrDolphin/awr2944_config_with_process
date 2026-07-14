#!/bin/bash

# =================================================================
# Raspberry Pi 4 Radar Server 一键配置脚本 (v3.0)
# 功能: 基础依赖安装、静态IP配置 (nmcli)、Systemd服务部署
# 适用系统: Raspberry Pi OS Lite (64-bit) / Debian Bookworm
# =================================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}>>> 开始配置树莓派雷达服务器环境...${NC}"

# 1. 检查 root 权限
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}错误: 请使用 sudo 运行此脚本${NC}"
    exit 1
fi

# 2. 系统更新与基础依赖安装
echo -e "${YELLOW}[1/4] 正在安装系统依赖...${NC}"
apt update
apt install -y python3-pip python3-serial network-manager

# 3. 安装 Python 库 (针对 Debian 12/Bookworm 的处理)
echo -e "${YELLOW}[2/4] 正在安装 Python 库 (websockets)...${NC}"
pip3 install websockets --break-system-packages

# 4. 静态 IP 配置 (通过 nmcli)
# 默认配置: IP 172.20.10.10, 网关 172.20.63.254, DNS 8.8.8.8
echo -e "${YELLOW}[3/4] 正在配置 WiFi 静态 IP...${NC}"

# 获取当前激活的 WiFi 连接名称
WIFI_CONN=$(nmcli -t -f NAME,TYPE connection show --active | grep wifi | cut -d: -f1)

if [ -z "$WIFI_CONN" ]; then
    echo -e "${RED}警告: 未发现激活的 WiFi 连接，跳过静态 IP 配置。${NC}"
    echo -e "请在连接 WiFi 后手动执行: nmcli connection modify '连接名' ipv4.addresses 172.20.10.10/18 ipv4.method manual"
else
    echo -e "发现 WiFi 连接: ${GREEN}$WIFI_CONN${NC}"
    nmcli connection modify "$WIFI_CONN" \
        ipv4.addresses 172.20.10.10/18 \
        ipv4.gateway 172.20.63.254 \
        ipv4.dns "8.8.8.8 114.114.114.114" \
        ipv4.method manual
    
    echo -e "静态 IP (172.20.10.10) 已配置。${YELLOW}注意: 重启后生效。${NC}"
fi

# 5. 部署 Systemd 服务
echo -e "${YELLOW}[4/4] 正在部署 radar.service 自动化服务...${NC}"

SERVICE_PATH="/etc/systemd/system/radar.service"
cat > $SERVICE_PATH << 'EOF'
[Unit]
Description=Radar WebSocket Server
After=network.target dev-ttyACM0.device dev-ttyACM1.device
BindsTo=dev-ttyACM0.device

[Service]
ExecStartPre=/bin/sleep 3
ExecStart=/usr/bin/python3 /home/pi/radar_server.py --cfg_port /dev/ttyACM0 --data_port /dev/ttyACM1 --cfg_file /home/pi/profile-2944.cfg --ws_port 8765
WorkingDirectory=/home/pi
User=pi
Environment=HOME=/home/pi
Environment=PYTHONPATH=/home/pi/.local/lib/python3.13/site-packages
StandardOutput=journal
StandardError=journal
Restart=on-failure
RestartSec=5

[Install]
WantedBy=dev-ttyACM0.device
EOF

systemctl daemon-reload
systemctl enable radar

echo -e "${GREEN}>>> 所有配置已完成!${NC}"
echo -e "${YELLOW}--------------------------------------------------${NC}"
echo -e "1. 请确保 /home/pi/ 目录下已有 radar_server.py 和 profile-2944.cfg"
echo -e "2. 插入雷达板后，输入 'sudo systemctl status radar' 查看状态"
echo -e "3. 推荐执行 'sudo reboot' 重启以激活静态 IP"
echo -e "${YELLOW}--------------------------------------------------${NC}"
