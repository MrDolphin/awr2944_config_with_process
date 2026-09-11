#!/usr/bin/env python3
"""Configure a DCA1000 from Linux/Windows without WaveStudio.

By default this command is a dry-run.  Use ``--apply`` only after verifying
the IP/interface values.  EEPROM/IP programming is intentionally opt-in via
``--write-eeprom`` because it changes persistent DCA1000 settings.
"""

from __future__ import annotations

import argparse
import json
import socket
from pathlib import Path

try:
    from tools.dca1000_protocol import (
        CMD_CONFIG_EEPROM,
        CMD_CONFIG_FPGA,
        CMD_CONFIG_PACKET_DATA,
        CMD_READ_FPGA_VERSION,
        CMD_RESET_FPGA,
        build_command,
        build_config_eeprom_payload,
        build_config_fpga_payload,
        build_packet_data_payload,
        parse_response,
    )
except ModuleNotFoundError:  # direct ``python tools/dca1000_configure.py``
    from dca1000_protocol import (
        CMD_CONFIG_EEPROM,
        CMD_CONFIG_FPGA,
        CMD_CONFIG_PACKET_DATA,
        CMD_READ_FPGA_VERSION,
        CMD_RESET_FPGA,
        build_command,
        build_config_eeprom_payload,
        build_config_fpga_payload,
        build_packet_data_payload,
        parse_response,
    )


def args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Configure DCA1000 over UDP 4096.")
    p.add_argument("--dca-ip", default="192.168.33.180")
    p.add_argument("--system-ip", default="192.168.33.30")
    p.add_argument("--mac", default="12.34.56.78.90.12")
    p.add_argument("--config-port", type=int, default=4096)
    p.add_argument("--packet-delay-us", type=int, default=25)
    p.add_argument(
        "--lvds-mode",
        type=int,
        choices=(1, 2),
        default=2,
        help="DCA1000 LVDS mode: 1 = 4 lane, 2 = 2 lane (must match SW2.3).",
    )
    p.add_argument("--timeout", type=float, default=1.0)
    p.add_argument("--apply", action="store_true", help="Send commands; otherwise print a dry-run plan.")
    p.add_argument("--write-eeprom", action="store_true", help="Also persist IP/MAC settings (requires --apply).")
    return p.parse_args()


def plan(ns: argparse.Namespace) -> list[tuple[str, int, bytes]]:
    commands: list[tuple[str, int, bytes]] = [
        ("reset_fpga", CMD_RESET_FPGA, b""),
    ]
    if ns.write_eeprom:
        commands.append(("configure_eeprom", CMD_CONFIG_EEPROM, build_config_eeprom_payload(ns.system_ip, ns.dca_ip, ns.mac)))
    commands.extend(
        [
            ("configure_fpga", CMD_CONFIG_FPGA, build_config_fpga_payload(lvds_mode=ns.lvds_mode)),
            ("configure_packet_data", CMD_CONFIG_PACKET_DATA, build_packet_data_payload(ns.packet_delay_us)),
            ("read_fpga_version", CMD_READ_FPGA_VERSION, b""),
        ]
    )
    return commands


def send(ns: argparse.Namespace, name: str, command: int, payload: bytes) -> dict[str, object]:
    packet = build_command(command, payload)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        # DCA1000 sends control responses back to the host's configured
        # command port (4096).  Binding the local endpoint is therefore
        # important; an ephemeral source port can make a valid response
        # invisible to this process.
        sock.settimeout(ns.timeout)
        sock.bind((ns.system_ip, ns.config_port))
        sock.sendto(packet, (ns.dca_ip, ns.config_port))
        response, peer = sock.recvfrom(2048)
    response_command, status = parse_response(response)
    return {
        "name": name,
        "command": command,
        "request_hex": packet.hex(" "),
        "response_hex": response.hex(" "),
        "response_command": response_command,
        "status_hex": status.hex(" "),
        "peer": f"{peer[0]}:{peer[1]}",
    }


def main() -> int:
    ns = args()
    commands = plan(ns)
    print(json.dumps({"apply": ns.apply, "dca_ip": ns.dca_ip, "config_port": ns.config_port, "commands": [{"name": n, "command": c, "payload_hex": p.hex(" "), "packet_hex": build_command(c, p).hex(" ")} for n, c, p in commands]}, indent=2))
    if not ns.apply:
        return 0
    for name, command, payload in commands:
        try:
            result = send(ns, name, command, payload)
        except (OSError, TimeoutError, ValueError) as exc:
            print(f"[FAIL] {name}: {exc}")
            return 2
        print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
