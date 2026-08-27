#!/usr/bin/env python3
"""Raspberry Pi 4B GNSS/RTK collector entry point."""

import argparse
import signal
import sys
import threading
from pathlib import Path

from config import MODES, load_config, resolve_output_dir, save_serial_port
from port_detect import detect_gnss_port, list_candidate_ports


BASE_DIR = Path(__file__).resolve().parent


def list_ports(cfg):
    candidates = list_candidate_ports(cfg.candidate_patterns)
    for device in candidates:
        marker = " [reserved]" if device in cfg.exclude_ports else ""
        print(device + marker)


def validate_ntrip(cfg):
    if not cfg.ntrip_enabled:
        raise ValueError("MODE.type={} requires NTRIP.enabled=true".format(
            cfg.work_mode
        ))
    missing = []
    for name, value in (
        ("NTRIP.host", cfg.ntrip_host),
        ("NTRIP.mountpoint", cfg.ntrip_mountpoint),
        ("NTRIP.username", cfg.ntrip_username),
        ("NTRIP.password", cfg.ntrip_password),
    ):
        if not value:
            missing.append(name)
    if missing:
        raise ValueError("missing settings: {}".format(", ".join(missing)))


def resolve_serial_port(cfg, args):
    """Manual --port wins; otherwise auto-detect and optionally persist."""
    if args.port:
        cfg.serial_port = args.port
        print("[serial] manual override: {}".format(cfg.serial_port))
        if args.save:
            save_serial_port(args.config, cfg.serial_port)
            print("[serial] saved to config: {}".format(args.config))
        return cfg.serial_port

    # auto_on_start=true deliberately redetects on every startup, even if a
    # previous run persisted a device path. Set auto_on_start=false to lock it.
    if not cfg.auto_on_start:
        if not cfg.serial_port or cfg.serial_port.lower() == "auto":
            raise ValueError(
                "manual mode requires SERIAL.port when auto_on_start=false"
            )
        print("[serial] manual config: {}".format(cfg.serial_port))
        return cfg.serial_port

    def audit(message):
        print("[detect] {}".format(message))

    print("[detect] searching UM982 on candidate serial ports")
    detected = detect_gnss_port(
        cfg,
        audit=audit,
    )
    if not detected:
        raise RuntimeError(
            "no UM982 found; check power, wiring, dialout membership, "
            "and SERIAL.exclude_ports"
        )

    cfg.serial_port = detected
    print("[detect] UM982 found: {}".format(detected))
    if cfg.save_detected_port and not args.no_save:
        save_serial_port(args.config, detected)
        print("[config] persisted SERIAL.port={}".format(detected))
    return detected


def main():
    parser = argparse.ArgumentParser(
        description="UM982 RTK rover for Raspberry Pi 4B"
    )
    parser.add_argument(
        "--config",
        default=str(BASE_DIR / "config.ini"),
        help="INI configuration path (default: tools/rtk-pi/config.ini)",
    )
    parser.add_argument("--mode", choices=MODES, help="override MODE.type")
    parser.add_argument("--port", help="manual serial device and persist it")
    parser.add_argument(
        "--duration",
        type=float,
        help="seconds to collect; omit or 0 means run until SIGINT/SIGTERM",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="do not persist an automatically detected port",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="persist a manually supplied --port as well",
    )
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument(
        "--check",
        action="store_true",
        help="load/show configuration without opening hardware",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.mode:
        cfg.work_mode = args.mode.upper()
    if args.duration is not None:
        cfg.duration_s = args.duration if args.duration > 0 else None

    output_dir = resolve_output_dir(cfg)
    print("Config:", Path(args.config).resolve())
    print("Mode:", cfg.work_mode)
    print("Serial policy:", "auto" if cfg.auto_on_start else "manual")
    print("Output:", output_dir)

    if args.list_ports:
        list_ports(cfg)
        return 0

    if args.check:
        print("Startup commands:")
        for command in cfg.receiver_commands():
            print("  {}".format(command))
        print("Configuration check OK")
        return 0

    port = resolve_serial_port(cfg, args)
    if cfg.work_mode != "SINGLE":
        validate_ntrip(cfg)
        print("NTRIP: {}/{}".format(cfg.ntrip_host, cfg.ntrip_mountpoint))
    else:
        print("NTRIP: disabled")

    from session import GnssSession

    stop_event = threading.Event()

    def request_stop(signum=None, frame=None):
        print("\n[signal] stopping collection")
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    session = GnssSession(port, cfg, stop_event)
    session.run()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("Interrupted")
        sys.exit(130)
    except Exception as exc:
        print("ERROR: {}".format(exc))
        sys.exit(1)
