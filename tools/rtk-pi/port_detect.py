"""Safe Linux serial discovery for a Pi with several UART/USB devices."""

import fnmatch
import glob
import re
from pathlib import Path
import time

import serial


def _priority(device):
    name = device.rsplit("/", 1)[-1]
    if fnmatch.fnmatch(name, "ttyUSB*"):
        rank = 0
    elif fnmatch.fnmatch(name, "ttyACM*"):
        rank = 1
    elif fnmatch.fnmatch(name, "ttyAMA*"):
        rank = 2
    else:
        rank = 3
    match = re.search(r"(\d+)$", name)
    number = int(match.group(1)) if match else 999999
    return rank, number, device


def list_candidate_ports(patterns):
    devices = set()
    for pattern in patterns:
        for device in glob.glob(pattern):
            if device.startswith("/dev/serial/"):
                try:
                    resolved = str(Path(device).resolve())
                except OSError:
                    continue
                devices.add(resolved)
            else:
                devices.add(device)
    return sorted(devices, key=_priority)


def excluded(device, exclusions):
    return any(
        fnmatch.fnmatch(device, item) or device == item for item in exclusions
    )


def probe_port(
    port,
    baudrate=115200,
    probe_timeout_s=1.2,
    serial_module=None,
    serial_factory=None,
):
    """Return True only for an UM982-style command response."""
    serial_module = serial_module or serial
    port_handle = None
    try:
        if serial_factory is not None:
            port_handle = serial_factory(port)
        else:
            port_handle = serial_module.Serial(
                port=port, baudrate=baudrate, timeout=0.2
            )
        port_handle.reset_input_buffer()
        port_handle.write(b"VERSION\r\n")
        port_handle.flush()
        deadline = time.monotonic() + float(probe_timeout_s)
        saw_command_frame = False
        while time.monotonic() < deadline:
            raw = port_handle.readline()
            if not raw:
                continue
            text = raw.decode("ascii", errors="ignore").strip()
            if not text:
                continue
            upper = text.upper()
            if "UM982" in upper or "$COMMAND,VERSION" in upper:
                return True
            if upper.startswith("$COMMAND") and "OK" in upper:
                saw_command_frame = True
            # Some firmware emits the product string before the response frame.
            if saw_command_frame and ("OK" in upper or "UM" in upper):
                return True
        return False
    except (OSError, serial_module.SerialException) as exc:
        message = str(exc).lower()
        if "permission denied" in message or "errno 13" in message:
            print("[detect] permission denied: {}; add user to dialout".format(port))
        return False
    finally:
        if port_handle is not None:
            try:
                port_handle.close()
            except Exception:
                pass


def detect_gnss_port(cfg, audit=None, candidates=None, probe_fn=None):
    if candidates is None:
        candidates = list_candidate_ports(cfg.candidate_patterns)
    started = time.monotonic()

    for device in candidates:
        if time.monotonic() - started >= cfg.detect_timeout_s:
            break
        if excluded(device, cfg.exclude_ports):
            if audit:
                audit("skip reserved/ineligible serial port: {}".format(device))
            continue
        if audit:
            audit("probing {}".format(device))
        if (probe_fn or probe_port)(
            device,
            baudrate=cfg.serial_baudrate,
            probe_timeout_s=cfg.probe_timeout_s,
        ):
            return device

    return None
