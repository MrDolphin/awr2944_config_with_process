#!/usr/bin/env python3
"""
Small CLI helper for configuring and starting/stopping an AWR2944 radar.

This script is intended for the router/Raspberry Pi side of the legacy
DCA1000 workflow. It replaces the interactive config_and_wait_key.py with
non-interactive commands that can be called from SSH.

Examples on the remote Linux host:
  python3 awr2944_cli_control.py configure --port /dev/ttyACM0 --cfg cfg/profile.cfg --no-start
  python3 awr2944_cli_control.py start --port /dev/ttyACM0
  python3 awr2944_cli_control.py stop --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError as exc:  # pragma: no cover - only runs on target host
    raise SystemExit("pyserial is required on the remote host: pip3 install pyserial") from exc


def open_serial(port: str, baud: int) -> serial.Serial:
    return serial.Serial(port, baud, timeout=1)


def send_line(cli: serial.Serial, line: str, delay: float) -> None:
    cli.write((line + "\n").encode("utf-8"))
    cli.flush()
    print(f">>> {line}")
    time.sleep(delay)


def read_response(cli: serial.Serial, timeout: float = 1.0) -> str:
    """Collect the immediate CLI reply without waiting forever for a prompt.

    ``sensorStart`` may leave the radar running without returning a prompt, but
    configuration rejection and firmware assertions are emitted immediately.
    Read until a short quiet period so those errors remain visible to the
    non-interactive capture launcher.
    """
    chunks: list[bytes] = []
    deadline = time.monotonic() + timeout
    quiet_deadline: float | None = None
    while time.monotonic() < deadline:
        waiting = cli.in_waiting
        if waiting:
            chunks.append(cli.read(waiting))
            quiet_deadline = time.monotonic() + 0.10
            continue
        if quiet_deadline is not None and time.monotonic() >= quiet_deadline:
            break
        time.sleep(0.02)
    return b"".join(chunks).decode("utf-8", errors="ignore").strip()


def print_and_validate_response(command: str, response: str) -> int:
    if response:
        print(f"[RADAR-CLI-RESPONSE] {response}")
    if "exception:" in response.lower() or "error" in response.lower():
        print(f"radar rejected {command}: {response}", file=sys.stderr)
        return 1
    return 0


def iter_cfg_commands(cfg_path: Path):
    for raw in cfg_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw.strip()
        if not line or line.startswith("%"):
            continue
        yield line


def configure(args: argparse.Namespace) -> int:
    cfg = Path(args.cfg)
    if not cfg.exists():
        print(f"cfg not found: {cfg}", file=sys.stderr)
        return 2

    with open_serial(args.port, args.baud) as cli:
        for line in iter_cfg_commands(cfg):
            if line.startswith("sensorStart"):
                if args.no_start:
                    print("... skip sensorStart from cfg")
                    continue
                if args.resume and line == "sensorStart":
                    line = "sensorStart 0"
            send_line(cli, line, args.delay)
            if args.read_echo:
                try:
                    echo = cli.read(cli.in_waiting or 1).decode("utf-8", errors="ignore").strip()
                    if echo:
                        print(echo)
                except Exception:
                    pass
    return 0


def start(args: argparse.Namespace) -> int:
    with open_serial(args.port, args.baud) as cli:
        command = "sensorStart 0" if args.resume else "sensorStart"
        send_line(cli, command, args.delay)
        return print_and_validate_response(command, read_response(cli))


def stop(args: argparse.Namespace) -> int:
    with open_serial(args.port, args.baud) as cli:
        command = "sensorStop"
        send_line(cli, command, args.delay)
        return print_and_validate_response(command, read_response(cli))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="AWR2944 non-interactive CLI control helper.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Radar CLI serial port.")
    parser.add_argument("--baud", type=int, default=115200, help="Radar CLI baud rate.")
    parser.add_argument("--delay", type=float, default=0.03, help="Delay after each command in seconds.")

    sub = parser.add_subparsers(dest="command", required=True)

    p_cfg = sub.add_parser("configure", help="Send cfg file commands.")
    p_cfg.add_argument("--cfg", required=True, help="Radar cfg file on the remote host.")
    p_cfg.add_argument("--no-start", action="store_true", help="Skip sensorStart lines in cfg.")
    p_cfg.add_argument("--resume", action="store_true", help="Use sensorStart 0 when starting.")
    p_cfg.add_argument("--read-echo", action="store_true", help="Print serial echo when available.")
    p_cfg.set_defaults(func=configure)

    p_start = sub.add_parser("start", help="Send sensorStart.")
    p_start.add_argument("--resume", action="store_true", help="Use sensorStart 0.")
    p_start.set_defaults(func=start)

    p_stop = sub.add_parser("stop", help="Send sensorStop.")
    p_stop.set_defaults(func=stop)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
