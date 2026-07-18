#!/usr/bin/env bash
set -euo pipefail

# Install only the runtime prerequisites. Network configuration is deliberately
# not changed here: shipboard Wi-Fi/IP addressing must be confirmed on site.
APP_DIR="${APP_DIR:-/home/pi/awr2944_config_with_process}"
SERVICE_NAME="radar.service"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo. Example: sudo APP_DIR=/home/pi/awr2944_config_with_process ./deploy/setup_rpi.sh"
  exit 1
fi
if [[ ! -f "${APP_DIR}/radar_server.py" ]]; then
  echo "APP_DIR does not contain radar_server.py: ${APP_DIR}"
  exit 2
fi

apt-get update
apt-get install -y python3 python3-serial python3-websockets

install -m 0644 "${SCRIPT_DIR}/radar.service" "/etc/systemd/system/${SERVICE_NAME}"
sed -i "s|^WorkingDirectory=.*|WorkingDirectory=${APP_DIR}|" "/etc/systemd/system/${SERVICE_NAME}"

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

echo "Installed ${SERVICE_NAME}. Before starting, run:"
echo "  cd ${APP_DIR}"
echo "  python3 tools/deployment_preflight.py --config Config/<profile>.cfg --require-ports"
echo "Then start with: sudo systemctl start ${SERVICE_NAME}"
