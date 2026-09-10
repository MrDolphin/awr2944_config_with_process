#!/usr/bin/env bash
# Switch only the Raspberry Pi management Wi-Fi.  The DCA1000 interface eth0
# remains untouched at 192.168.33.30/24 throughout every mode.
set -euo pipefail

WLAN_DEVICE="wlan0"
HOTSPOT_PROFILE="radar-pi-ap"
DCA_DEVICE="eth0"
DCA_SUBNET="192.168.33.0/24"

usage() {
  cat <<'EOF'
Usage: sudo tools/pi_network_mode.sh <command> [profile]

Commands:
  status                 Show the active Wi-Fi profile and read-only DCA route checks.
  hotspot                Activate the existing radar-pi-ap profile on wlan0.
  wifi <saved-profile>   Activate an existing saved Wi-Fi profile on wlan0.

Examples:
  tools/pi_network_mode.sh status
  sudo tools/pi_network_mode.sh hotspot
  sudo tools/pi_network_mode.sh wifi WLAN-DHSYS
  sudo tools/pi_network_mode.sh wifi oneplus-hotspot

The hotspot command intentionally disconnects the current SSH session. Reconnect
the Windows PC to the Pi hotspot and use the address reported by the script
(normally 10.42.0.1). The script never changes eth0, DCA1000 addressing, radar
configuration, or capture state.
EOF
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "[FAIL] Required command is unavailable: $1" >&2
    exit 127
  }
}

require_root() {
  if [[ "${EUID}" -ne 0 ]]; then
    echo "[FAIL] This action changes wlan0. Re-run with sudo." >&2
    exit 77
  fi
}

require_connection_profile() {
  local profile="$1"
  if ! nmcli -g NAME connection show | grep -Fxq -- "$profile"; then
    echo "[FAIL] Connection profile does not exist: $profile" >&2
    exit 2
  fi
}

require_wifi_profile() {
  local profile="$1"
  local connection_type
  connection_type="$(nmcli -g connection.type connection show "$profile")"
  if [[ "$connection_type" != "802-11-wireless" ]]; then
    echo "[FAIL] Profile is not a Wi-Fi connection: $profile ($connection_type)" >&2
    exit 2
  fi
}

show_status() {
  echo "[STATUS] NetworkManager devices"
  nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
  echo
  echo "[STATUS] Active connections"
  nmcli connection show --active
  echo
  echo "[STATUS] IP addresses"
  ip -br addr
  echo
  echo "[STATUS] Routes"
  ip route
  echo
  echo "[DCA-GUARD] eth0 is read-only in this script"
  ip -4 addr show dev eth0 || true
  ip route show "$DCA_SUBNET" || true
  ip neigh show dev eth0 || true
}

disconnect_hotspot_if_active() {
  if nmcli -t -f NAME,DEVICE connection show --active | grep -Fxq "${HOTSPOT_PROFILE}:${WLAN_DEVICE}"; then
    echo "[ACTION] Deactivating ${HOTSPOT_PROFILE} on ${WLAN_DEVICE}"
    nmcli connection down "$HOTSPOT_PROFILE"
  fi
}

activate_hotspot() {
  require_root
  require_connection_profile "$HOTSPOT_PROFILE"
  require_wifi_profile "$HOTSPOT_PROFILE"

  echo "[WARN] Activating ${HOTSPOT_PROFILE} disconnects the current Wi-Fi/SSH session."
  echo "[ACTION] Activating ${HOTSPOT_PROFILE} on ${WLAN_DEVICE}"
  nmcli connection up "$HOTSPOT_PROFILE" ifname "$WLAN_DEVICE"
  echo "[DONE] Hotspot active. Reconnect using: ssh pi@10.42.0.1"
  show_status
}

activate_saved_wifi() {
  local profile="$1"
  require_root
  require_connection_profile "$profile"
  require_wifi_profile "$profile"

  echo "[WARN] Switching Wi-Fi disconnects the current hotspot/SSH session."
  disconnect_hotspot_if_active
  echo "[ACTION] Activating ${profile} on ${WLAN_DEVICE}"
  nmcli connection up "$profile" ifname "$WLAN_DEVICE"
  echo "[DONE] Saved Wi-Fi profile active: ${profile}"
  show_status
}

main() {
  local action="${1:-}"
  case "$action" in
    -h|--help|help)
      usage
      return 0
      ;;
  esac

  require_command nmcli
  require_command ip

  case "$action" in
    status)
      [[ "$#" -eq 1 ]] || { usage >&2; exit 2; }
      show_status
      ;;
    hotspot)
      [[ "$#" -eq 1 ]] || { usage >&2; exit 2; }
      activate_hotspot
      ;;
    wifi)
      [[ "$#" -eq 2 ]] || { usage >&2; exit 2; }
      activate_saved_wifi "$2"
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
}

main "$@"
