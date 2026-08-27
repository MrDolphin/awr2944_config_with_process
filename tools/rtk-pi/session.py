"""GNSS/NTRIP acquisition session for Raspberry Pi."""

import base64
import csv
import json
import socket
import threading
import time
from datetime import datetime
from pathlib import Path

from nmea_parser import NmeaParser
from rtk_state import PositionState, RtkState


QUALITY_NAMES = {
    0: "NO FIX",
    1: "SINGLE",
    2: "DGPS",
    4: "RTK FIXED",
    5: "RTK FLOAT",
}
RANKS = {
    "NO FIX": 0,
    "UNKNOWN": 0,
    "SINGLE": 1,
    "DGPS": 2,
    "RTK FLOAT": 3,
    "RTK FIXED": 4,
}


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.flush()
    temporary.replace(path)


class Rtcm3Framer:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 6:
            if self.buffer[0] != 0xD3:
                self.buffer.pop(0)
                continue
            length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
            total = length + 6
            if len(self.buffer) < total:
                break
            frames.append(bytes(self.buffer[:total]))
            del self.buffer[:total]
        return frames


def checksum_nmea(sentence):
    sentence = sentence.strip()
    if "*" in sentence:
        return sentence
    body = sentence[1:]
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return "{}*{:02X}".format(sentence, checksum)


class NtripWorker(threading.Thread):
    def __init__(self, serial_port, config, latest_gga, stop_event):
        super().__init__(daemon=True)
        self.serial_port = serial_port
        self.config = config
        self.latest_gga = latest_gga
        self.stop_event = stop_event
        self.frames_received = 0
        self.bytes_received = 0
        self.bytes_injected = 0
        self.connected = False
        self.first_payload_monotonic = None
        self.last_payload_monotonic = None
        self._no_data_warned_at = None

    def _request(self):
        token = base64.b64encode("{}:{}".format(
            self.config.ntrip_username,
            self.config.ntrip_password,
        ).encode("ascii")).decode("ascii")
        return (
            "GET /{} HTTP/1.1\r\n"
            "Host: {}:{}\r\n"
            "User-Agent: rtk-pi/1.0\r\n"
            "Authorization: Basic {}\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            "Accept: */*\r\n\r\n"
        ).format(
            self.config.ntrip_mountpoint,
            self.config.ntrip_host,
            self.config.ntrip_port,
            token,
        ).encode("ascii")

    def gga_packet(self):
        """Return an NMEA packet; NTRIP requires CRLF termination."""
        sentence = self.latest_gga[0].strip() or self.config.gga_fallback
        return checksum_nmea(sentence).encode("ascii") + b"\r\n"

    @staticmethod
    def split_response(data):
        """Split an NTRIP HTTP response into header and binary payload."""
        separator = data.find(b"\r\n\r\n")
        if separator < 0:
            return data, b""
        boundary = separator + len(b"\r\n\r\n")
        return data[:boundary], data[boundary:]

    def receive_payload(self, framer, data):
        if not data:
            return
        now = time.monotonic()
        if self.first_payload_monotonic is None:
            self.first_payload_monotonic = now
            print(
                "[ntrip] first correction payload: {} bytes".format(len(data))
            )
        self.bytes_received += len(data)
        self.frames_received += len(framer.feed(data))
        self.serial_port.write(data)
        self.serial_port.flush()
        self.bytes_injected += len(data)
        self.last_payload_monotonic = now

    def run(self):
        framer = Rtcm3Framer()
        while not self.stop_event.is_set():
            sock = None
            try:
                sock = socket.create_connection(
                    (self.config.ntrip_host, self.config.ntrip_port),
                    timeout=10.0,
                )
                sock.settimeout(2.0)
                sock.sendall(self._request())
                header = b""
                deadline = time.monotonic() + 10.0
                while time.monotonic() < deadline:
                    chunk = sock.recv(1024)
                    if not chunk:
                        break
                    header += chunk
                    if b"\r\n\r\n" in header:
                        break
                response_header, residual_payload = self.split_response(header)
                if b"200 OK" not in response_header:
                    print("[ntrip] authentication failed: {}".format(header[:120]))
                    time.sleep(5.0)
                    continue

                self.connected = True
                print("[ntrip] connected: {}/{}".format(
                    self.config.ntrip_host, self.config.ntrip_mountpoint
                ))
                # Prefer a position actually reported by the receiver.  The
                # configured fallback may be far from the current VRS area.
                wait_deadline = time.monotonic() + 3.0
                while (
                    not self.stop_event.is_set()
                    and not self.latest_gga[0].strip()
                    and time.monotonic() < wait_deadline
                ):
                    time.sleep(0.1)
                sock.sendall(self.gga_packet())
                if residual_payload:
                    self.receive_payload(framer, residual_payload)
                last_gga = time.monotonic()
                connection_started = last_gga

                while not self.stop_event.is_set():
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        data = b""
                    if data:
                        self.receive_payload(framer, data)

                    if time.monotonic() - last_gga >= 5.0:
                        sock.sendall(self.gga_packet())
                        last_gga = time.monotonic()

                    connected_for = time.monotonic() - connection_started
                    if self.first_payload_monotonic is None and connected_for >= 30:
                        since_warning = (
                            time.monotonic()
                            if self._no_data_warned_at is None
                            else time.monotonic() - self._no_data_warned_at
                        )
                        if since_warning >= 30:
                            self._no_data_warned_at = time.monotonic()
                            print(
                                "[ntrip] warning: connected {}s but no RTCM "
                                "payload; check GGA updates, account rights, "
                                "and mountpoint {}".format(
                                    int(connected_for),
                                    self.config.ntrip_mountpoint,
                                )
                            )
            except Exception as exc:
                print("[ntrip] retry after error: {}".format(exc))
                self.connected = False
                if not self.stop_event.wait(5.0):
                    continue
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass


class GnssSession:
    CSV_HEADER = [
        "pc_time", "gnss_time", "latitude", "longitude", "altitude",
        "quality", "satellites", "hdop", "speed", "course",
        "heading", "heading_available", "position_state",
    ]

    def __init__(self, serial_port, config, stop_event):
        self.port_name = serial_port
        self.cfg = config
        self.stop_event = stop_event
        self.parser = NmeaParser()
        self.state = RtkState(
            min_fixed_epochs=config.min_fixed_epochs,
            fix_timeout_s=config.fix_timeout_s,
        )
        self.latest_gga = [""]
        self.quality_counts = {}
        self.lines_written = 0
        self.last_status = None
        self.session_dir = None
        self.raw_handle = None
        self.csv_writer = None
        self.csv_file = None
        self.status_handle = None
        self._last_gga_sentence = None
        self._last_rmc = {}
        self._status_deadline = 0.0
        self.started_monotonic = None

    def _open_session(self):
        output_dir = Path(self.cfg.output_directory)
        if not output_dir.is_absolute():
            output_dir = Path(self.cfg.config_path).resolve().parent / output_dir
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = output_dir / stamp
        self.session_dir.mkdir(parents=True, exist_ok=True)

        marker = self.session_dir / ".incomplete"
        marker.touch()
        if self.cfg.raw_log:
            self.raw_handle = open(
                self.session_dir / "gnss_raw.log", "w", encoding="utf-8"
            )
        csv_path = self.session_dir / "position.csv"
        self.csv_file = open(csv_path, "w", newline="", encoding="utf-8-sig")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.CSV_HEADER)
        self.status_handle = open(
            self.session_dir / "status.log", "w", encoding="utf-8"
        )
        self.save_status("session started on {}".format(self.port_name))

    def save_status(self, text):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = "{} {}".format(timestamp, text)
        print(line)
        if self.status_handle is not None:
            self.status_handle.write(line + "\n")
            self.status_handle.flush()

    def _write_raw(self, text):
        if self.raw_handle is None:
            return
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.raw_handle.write("{} {}\n".format(timestamp, text))
        self.raw_handle.flush()
        self.lines_written += 1

    def _save_position(self, record):
        row = [record.get(key) for key in self.CSV_HEADER]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def _handle_gga(self, record, raw):
        # UM982 can mirror one output sentence; keep raw logs but count once.
        duplicate = raw == self._last_gga_sentence
        self._last_gga_sentence = raw
        self.latest_gga[0] = raw
        quality = int(record.get("quality", 0))
        status = QUALITY_NAMES.get(quality, "UNKNOWN")

        if not duplicate:
            self.quality_counts[status] = self.quality_counts.get(status, 0) + 1
        previous_state = self.state.position.value
        current_state = self.state.update_quality(quality).value

        if current_state != self.last_status and previous_state != current_state:
            self.save_status("POSITION STATUS: {}".format(current_state))
            self.last_status = current_state

        if quality > 0:
            merged = dict(record)
            merged["pc_time"] = datetime.now().strftime(
                "%Y-%m-%d %H:%M:%S.%f"
            )[:-3]
            merged["speed"] = self._last_rmc.get("speed")
            merged["course"] = self._last_rmc.get("course")
            merged["heading"] = (
                self.state.heading_deg if self.state.heading_available else None
            )
            merged["heading_available"] = self.state.heading_available
            merged["position_state"] = current_state
            self._save_position(merged)

    def run(self):
        import serial

        self._open_session()
        serial_port = serial.Serial(
            port=self.port_name,
            baudrate=self.cfg.serial_baudrate,
            timeout=self.cfg.serial_timeout,
        )
        worker = None
        try:
            print("[receiver] sending startup commands")
            for command in self.cfg.receiver_commands():
                print("[receiver] {}".format(command))
                serial_port.write((command + "\r\n").encode("ascii"))
                serial_port.flush()
                time.sleep(0.25)

            if self.cfg.work_mode != "SINGLE":
                worker = NtripWorker(
                    serial_port,
                    self.cfg,
                    self.latest_gga,
                    self.stop_event,
                )
                worker.start()

            started = time.monotonic()
            while not self.stop_event.is_set():
                if self.cfg.duration_s is not None:
                    remaining = started + self.cfg.duration_s - time.monotonic()
                    if remaining <= 0:
                        break

                raw = serial_port.readline()
                if not raw:
                    continue
                text = raw.decode("ascii", errors="ignore").strip()
                if not text:
                    continue
                self._write_raw(text)

                parsed = self.parser.parse(text)
                if not parsed:
                    continue
                if parsed["type"] == "GGA":
                    self._handle_gga(parsed, text)
                elif parsed["type"] == "RMC":
                    self._last_rmc = {
                        "speed": parsed.get("speed"),
                        "course": parsed.get("course"),
                    }
                elif parsed["type"] == "UNIHEADINGA":
                    self.state.update_heading(
                        parsed.get("sol_stat"), parsed.get("heading", 0.0)
                    )

                now = time.monotonic()
                if now >= self._status_deadline:
                    self._status_deadline = now + self.cfg.status_interval_s
                    print("[state] {} | lines={} | rtcm={}".format(
                        self.state.position.value,
                        self.lines_written,
                        worker.bytes_received if worker else 0,
                    ))
        finally:
            self.stop_event.set()
            if worker is not None:
                worker.join(timeout=3.0)
            serial_port.close()
            manifest_path = self.finalize(worker)
            print("Session directory:", self.session_dir.resolve())
            print("Manifest:", manifest_path.resolve())

    def achieved_target(self):
        counts = self.quality_counts
        if self.cfg.work_mode == "SINGLE":
            return counts.get("SINGLE", 0) > 0
        if self.cfg.work_mode == "DGPS":
            return any(counts.get(name, 0) > 0 for name in (
                "DGPS", "RTK FLOAT", "RTK FIXED"
            ))
        return any(counts.get(name, 0) > 0 for name in (
            "RTK FLOAT", "RTK FIXED"
        ))

    def finalize(self, worker=None):
        complete = self.achieved_target()
        reason = "" if complete else "requested mode not reached before stop"
        status = "complete" if complete else "partial"

        if self.status_handle is not None:
            self.save_status(
                "session finished: status={} counts={}".format(
                    status, self.quality_counts
                )
            )
            self.status_handle.close()
            self.status_handle = None
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        if self.raw_handle is not None:
            self.raw_handle.close()
            self.raw_handle = None
        (self.session_dir / ".incomplete").unlink(missing_ok=True)

        duration = 0.0
        if self.started_monotonic is not None:
            duration = max(time.monotonic() - self.started_monotonic, 0.0)
        manifest = {
            "session_id": self.session_dir.name,
            "platform": "raspberry_pi_4b",
            "status": status,
            "is_valid": complete,
            "reason": reason,
            "mode_requested": self.cfg.work_mode,
            "serial_port": self.port_name,
            "duration_actual_s": round(duration, 3),
            "duration_planned_s": self.cfg.duration_s,
            "raw_lines": self.lines_written,
            "quality_counts": self.quality_counts,
            "best_position": self.state.best_position.value,
            "heading_available": self.state.heading_available,
            "ntrip_enabled": self.cfg.work_mode != "SINGLE",
            "rtcm_bytes_injected": worker.bytes_injected if worker else 0,
            "rtcm_frames_received": worker.frames_received if worker else 0,
        }
        path = self.session_dir / "manifest.json"
        atomic_json(path, manifest)
        return path
