import struct
import unittest

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


class Dca1000ProtocolTests(unittest.TestCase):
    def test_command_framing_matches_wave_capture(self):
        packet = build_command(CMD_CONFIG_FPGA, bytes([1, 2, 1, 2, 3, 30]))
        self.assertEqual(
            packet.hex(" "),
            "5a a5 03 00 06 00 01 02 01 02 03 1e aa ee",
        )

    def test_four_lane_fpga_payload_uses_lvds_mode_one(self):
        self.assertEqual(
            build_config_fpga_payload(lvds_mode=1).hex(" "),
            "01 01 01 02 03 1e",
        )

    def test_packet_delay_encoding_matches_25_50_and_75_us_captures(self):
        for delay, expected in ((25, "be 05 35 0c 00 00"), (50, "be 05 6a 18 00 00"), (75, "be 05 9f 24 00 00")):
            self.assertEqual(build_packet_data_payload(delay).hex(" "), expected)

    def test_eeprom_payload_uses_octets_then_mac(self):
        payload = build_config_eeprom_payload(
            "192.168.33.30", "192.168.33.180", "12.34.56.78.90.12"
        )
        self.assertEqual(
            payload,
            bytes([192, 168, 33, 30, 192, 168, 33, 180, 12, 34, 56, 78, 90, 12]),
        )

    def test_response_status_and_version(self):
        self.assertEqual(parse_response(bytes.fromhex("5a a5 03 00 00 00 aa ee")), (3, b"\x00\x00"))
        self.assertEqual(parse_response(bytes.fromhex("5a a5 0e 00 82 04 aa ee")), (14, b"\x82\x04"))

    def test_empty_commands(self):
        self.assertEqual(build_command(CMD_RESET_FPGA).hex(" "), "5a a5 01 00 00 00 aa ee")
        self.assertEqual(build_command(CMD_READ_FPGA_VERSION).hex(" "), "5a a5 0e 00 00 00 aa ee")


if __name__ == "__main__":
    unittest.main()
