#!/usr/bin/env python3
"""Hardware-free checks for the Raspberry Pi collector."""

import tempfile
import threading
from pathlib import Path

from config import load_config, save_serial_port
from nmea_parser import NmeaParser
from port_detect import detect_gnss_port
from rtk_state import RtkState
from session import NtripWorker, Rtcm3Framer, checksum_nmea


def make_config(text):
    handle = tempfile.NamedTemporaryFile(
        "w", suffix=".ini", delete=False, encoding="utf-8"
    )
    handle.write(text)
    handle.close()
    return Path(handle.name)


class FakeProbeSerial:
    def __init__(self, responses):
        self.responses = list(responses)

    def reset_input_buffer(self):
        pass

    def write(self, data):
        return len(data)

    def flush(self):
        pass

    def readline(self):
        return self.responses.pop(0) if self.responses else b""

    def close(self):
        pass


class FakeInjectedSerial:
    def __init__(self):
        self.written = b""

    def write(self, data):
        self.written += data
        return len(data)

    def flush(self):
        pass


def make_rtcm3(payload):
    length = len(payload)
    header = bytes([
        0xD3,
        (length >> 8) & 0x03,
        length & 0xFF,
    ])
    return header + payload + b"\x00\x00\x00"


def fake_probe_factory(responses):
    def probe(device, baudrate=115200, probe_timeout_s=1.2):
        # The detector passes a device path; emulate only the expected target.
        if device != "/dev/ttyUSB2":
            return False
        serial = FakeProbeSerial(responses)
        try:
            serial.write(b"VERSION\r\n")
            return bool(serial.readline())
        finally:
            serial.close()
    return probe


def test_mode_commands():
    ini = """
[MODE]
type = SINGLE
[SERIAL]
port = auto
auto_on_start = true
save_detected_port = true
exclude_ports = /dev/ttyACM0,/dev/ttyACM1
[RECEIVER]
rover_profile = UAV
gga_rate = 2
[NTRIP]
enabled = false
"""
    path = make_config(ini)
    cfg = load_config(path)
    assert cfg.work_mode == "SINGLE"
    commands = cfg.receiver_commands()
    assert commands[0] == "MODE ROVER UAV"
    assert "GPGGA 2" in commands
    assert "CONFIG DGPS TIMEOUT 0" in commands
    assert "CONFIG RTK TIMEOUT 0" in commands


def test_config_port_save_preserves_file():
    source = Path(__file__).with_name("config.example.ini")
    temporary = Path(tempfile.mkdtemp()) / "config.ini"
    temporary.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
    before = temporary.read_text(encoding="utf-8")
    save_serial_port(temporary, "/dev/ttyUSB2")
    after = temporary.read_text(encoding="utf-8")
    assert "port = /dev/ttyUSB2" in after
    assert "[NTRIP]" in after
    assert "host = 203.107.45.154" in after
    assert before.count("[RECEIVER]") == after.count("[RECEIVER]") == 1


def test_detection_skips_reserved_and_matches_um982():
    ini = """
[MODE]
type = RTK
[SERIAL]
port = auto
candidate_patterns = /dev/ttyUSB*
exclude_ports = /dev/ttyACM0,/dev/ttyACM1
[RECEIVER]
gga_rate = 1
"""
    path = make_config(ini)
    cfg = load_config(path)
    seen = []

    def audit(message):
        seen.append(message)

    detected = detect_gnss_port(
        cfg,
        audit=audit,
        candidates=[
            "/dev/ttyACM0",
            "/dev/ttyACM1",
            "/dev/ttyUSB9",
            "/dev/ttyUSB2",
        ],
        probe_fn=fake_probe_factory([b"$command,VERSION,response: OK*04\n"]),
    )
    assert detected == "/dev/ttyUSB2"
    assert any("/dev/ttyACM0" in message for message in seen)
    assert any("/dev/ttyACM1" in message for message in seen)


def test_parser_and_state():
    parser = NmeaParser()
    gga = (
        "$GNGGA,010000.00,2959.77910000,N,12209.32000000,E,"
        "4,20,0.9,31.2,M,14.5,M,1.0,0643*00"
    ).split("*")[0]
    record = parser.parse(gga)
    assert record["type"] == "GGA"
    assert record["quality"] == 4
    assert abs(record["latitude"] - 29.9963183333) < 1e-8

    state = RtkState(min_fixed_epochs=2, fix_timeout_s=10.0)
    state.update_quality(4, now=1.0)
    assert state.update_quality(4, now=2.0) == state.position
    assert state.position.value == "RTK FIXED"


def test_ntrip_gga_and_payload_handling():
    source = Path(__file__).with_name("config.example.ini")
    cfg = load_config(source)
    serial = FakeInjectedSerial()
    stop_event = threading.Event()
    worker = NtripWorker(serial, cfg, [""], stop_event)

    worker.latest_gga[0] = (
        "$GNGGA,010000.00,2959.77910000,N,12209.32000000,E,"
        "1,20,0.9,31.2,M,14.5,M,,"
    )
    packet = worker.gga_packet()
    assert packet.startswith(b"$GNGGA,")
    assert packet.endswith(b"\r\n")
    expected = checksum_nmea(worker.latest_gga[0]).encode("ascii") + b"\r\n"
    assert packet == expected

    rtcm_frame = make_rtcm3(b"\x3e\x00\xd0" + b"\x00" * 16)
    response = (
        b"HTTP/1.1 200 OK\r\n"
        b"Ntrip-Version: Ntrip/2.0\r\n"
        b"\r\n" + rtcm_frame
    )
    header, payload = worker.split_response(response)
    assert header.endswith(b"\r\n\r\n")
    assert payload == rtcm_frame

    framer = Rtcm3Framer()
    worker.receive_payload(framer, payload)
    assert serial.written == payload
    assert worker.bytes_received == len(payload)
    assert worker.bytes_injected == len(payload)
    assert worker.frames_received == 1


def main():
    test_mode_commands()
    test_config_port_save_preserves_file()
    test_detection_skips_reserved_and_matches_um982()
    test_parser_and_state()
    test_ntrip_gga_and_payload_handling()
    print("rtk-pi selftest: PASS")


if __name__ == "__main__":
    main()
