"""Small, dependency-free DCA1000 UDP control protocol implementation.

The command framing and payload layouts follow TI's DCA1000 protocol guide
and the AWR2944P captures kept with this project.  This module does not open
sockets; callers can unit-test and dry-run packets without hardware.
"""

from __future__ import annotations

import ipaddress
import struct
from typing import Tuple

HEADER = 0xA55A
FOOTER = 0xEEAA

CMD_RESET_FPGA = 0x0001
CMD_RESET_AR_DEVICE = 0x0002
CMD_CONFIG_FPGA = 0x0003
CMD_CONFIG_EEPROM = 0x0004
CMD_START_RECORD = 0x0005
CMD_STOP_RECORD = 0x0006
CMD_SYSTEM_ALIVENESS = 0x0009
CMD_CONFIG_PACKET_DATA = 0x000B
CMD_CONFIG_DATA_MODE = 0x000C
CMD_READ_FPGA_VERSION = 0x000E


def build_command(command: int, payload: bytes = b"") -> bytes:
    """Build a TI DCA1000 config-port command packet."""
    if not 0 <= command <= 0xFFFF:
        raise ValueError("command must fit uint16")
    if len(payload) > 504:
        raise ValueError("DCA1000 command payload exceeds 504 bytes")
    return struct.pack("<HHH", HEADER, command, len(payload)) + payload + struct.pack("<H", FOOTER)


def build_config_fpga_payload(
    logging_mode: int = 1,
    lvds_mode: int = 2,
    transfer_mode: int = 1,
    capture_mode: int = 2,
    data_format_mode: int = 3,
    timeout_s: int = 30,
) -> bytes:
    """Build the six-byte CONFIG_FPGA payload used by AWR2944P capture."""
    values = (logging_mode, lvds_mode, transfer_mode, capture_mode, data_format_mode, timeout_s)
    if any(not 0 <= value <= 0xFF for value in values):
        raise ValueError("CONFIG_FPGA fields must fit uint8")
    return bytes(values)


def build_packet_data_payload(packet_delay_us: int, packet_size: int = 1470) -> bytes:
    """Build CONFIG_PACKET_DATA payload.

    TI's packet-delay field is encoded in 8-ns ticks.  The AWR2944P captures
    show the equivalent integer relation ``delay_us * 125``.
    """
    if packet_size < 0 or packet_size > 0xFFFF:
        raise ValueError("packet_size must fit uint16")
    if packet_delay_us < 0 or packet_delay_us * 125 > 0xFFFF:
        raise ValueError("packet_delay_us is outside the uint16 tick range")
    delay_ticks = packet_delay_us * 125
    return struct.pack("<HHH", packet_size, delay_ticks, 0)


def _parse_octets(value: str, label: str) -> bytes:
    try:
        return ipaddress.ip_address(value).packed
    except ValueError as exc:
        raise ValueError(f"invalid {label} IPv4 address: {value}") from exc


def _parse_mac(value: str) -> bytes:
    dotted_decimal = "." in value and ":" not in value and "-" not in value
    parts = value.replace("-", ":").replace(".", ":").split(":")
    if len(parts) != 6:
        raise ValueError("MAC address must contain six octets")
    try:
        result = bytes(int(part, 10 if dotted_decimal else 16) for part in parts)
    except ValueError as exc:
        raise ValueError(f"invalid MAC address: {value}") from exc
    if len(result) != 6:
        raise ValueError("MAC address must contain six octets")
    return result


def build_config_eeprom_payload(system_ip: str, dca_ip: str, mac: str) -> bytes:
    """Build CONFIG_EEPROM payload: system IPv4, DCA IPv4, then MAC."""
    return _parse_octets(system_ip, "system") + _parse_octets(dca_ip, "DCA") + _parse_mac(mac)


def parse_response(packet: bytes) -> Tuple[int, bytes]:
    """Validate a DCA1000 response and return ``(command, status_bytes)``.

    Responses use the two bytes after the command as a status/value field,
    not as the request payload-length field.  For example, the FPGA version
    response captured from AWR2944P is ``5a a5 0e 00 82 04 aa ee``.
    """
    if len(packet) < 8:
        raise ValueError("DCA1000 response is shorter than 8 bytes")
    header, command = struct.unpack_from("<HH", packet, 0)
    if header != HEADER:
        raise ValueError("invalid DCA1000 response header")
    footer = struct.unpack_from("<H", packet, len(packet) - 2)[0]
    if footer != FOOTER:
        raise ValueError("invalid DCA1000 response footer")
    return command, packet[4:-2]
