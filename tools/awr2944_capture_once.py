#!/usr/bin/env python3
"""One-command AWR2944P + DCA1000 raw-ADC capture on a Raspberry Pi.

The launcher preserves the safe sequencing established in the field test:
configure DCA1000, load radar CFG without starting frames, open UDP 4098
listener, start DCA recording, then start radar frames.  It always attempts
to stop both DCA recording and the radar CLI when the capture ends or fails.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

try:
    from tools import dca1000_configure
    from tools.dca1000_capture import DCA_CMD_RECORD_START, DCA_CMD_RECORD_STOP, send_dca_command
except ModuleNotFoundError:  # Direct ``python tools/awr2944_capture_once.py``.
    import dca1000_configure
    from dca1000_capture import DCA_CMD_RECORD_START, DCA_CMD_RECORD_STOP, send_dca_command


TOOLS_DIR = Path(__file__).resolve().parent


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cfg", required=True, help="Radar CFG with lvdsStreamCfg enabled.")
    parser.add_argument("--cli-port", default="/dev/ttyACM0", help="AWR2944P CLI serial device.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--dca-ip", default="192.168.33.180")
    parser.add_argument("--system-ip", default="192.168.33.30")
    parser.add_argument("--dca-mac", default="12.34.56.78.90.12")
    parser.add_argument("--config-port", type=int, default=4096)
    parser.add_argument("--data-port", type=int, default=4098)
    parser.add_argument("--packet-delay-us", type=int, default=25)
    parser.add_argument("--duration", type=float, default=60.0, help="Capture duration in seconds.")
    parser.add_argument("--output-dir", default="/home/pi/radar_runs/awr2944p")
    parser.add_argument("--prefix", default="adc_data")
    parser.add_argument("--min-free-gb", type=float, default=4.0)
    parser.add_argument("--socket-buffer-mb", type=int, default=64)
    parser.add_argument("--cli-delay", type=float, default=0.15)
    parser.add_argument("--dca-timeout", type=float, default=5.0)
    parser.add_argument("--listener-timeout", type=float, default=10.0)
    return parser.parse_args(argv)


def validate_cfg(path: Path) -> list[str]:
    if not path.is_file():
        return [f"cfg not found: {path}"]
    commands = [line.strip() for line in path.read_text(encoding="utf-8", errors="ignore").splitlines()]
    active = [line for line in commands if line and not line.startswith(("%", "#"))]
    if not any(line.startswith("lvdsStreamCfg") for line in active):
        return ["lvdsStreamCfg is required for DCA1000 raw ADC capture"]
    return []


def build_cli_configure_command(args: argparse.Namespace) -> list[str]:
    return [
        sys.executable, str(TOOLS_DIR / "awr2944_cli_control.py"),
        "--port", args.cli_port, "--baud", str(args.baud), "--delay", str(args.cli_delay),
        "configure", "--cfg", str(args.cfg), "--no-start", "--read-echo",
    ]


def build_cli_start_stop_command(args: argparse.Namespace, command: str) -> list[str]:
    if command not in {"start", "stop"}:
        raise ValueError("command must be start or stop")
    return [
        sys.executable, str(TOOLS_DIR / "awr2944_cli_control.py"),
        "--port", args.cli_port, "--baud", str(args.baud), command,
    ]


def build_capture_command(args: argparse.Namespace) -> list[str]:
    """Build listener-only capture command; this launcher controls record start."""
    return [
        # The launcher waits for the child's "[UDP] Listening" line before
        # sending record-start and sensorStart.  Force unbuffered output so a
        # piped stdout cannot hide that readiness signal until process exit.
        sys.executable, "-u", str(TOOLS_DIR / "dca1000_capture.py"),
        "--cfg", str(args.cfg), "--cf-json", "",
        "--listen-ip", args.system_ip,
        "--dca-ip", args.dca_ip,
        "--config-port", str(args.config_port),
        "--data-port", str(args.data_port),
        "--duration", str(args.duration),
        "--no-control",
        "--socket-buffer-mb", str(args.socket_buffer_mb),
        "--min-free-gb", str(args.min_free_gb),
        "--output-dir", str(args.output_dir),
        "--prefix", args.prefix,
    ]


def configure_dca(args: argparse.Namespace) -> None:
    config_args = argparse.Namespace(
        dca_ip=args.dca_ip,
        system_ip=args.system_ip,
        mac=args.dca_mac,
        config_port=args.config_port,
        packet_delay_us=args.packet_delay_us,
        timeout=args.dca_timeout,
        apply=True,
        write_eeprom=False,
    )
    for name, command, payload in dca1000_configure.plan(config_args):
        result = dca1000_configure.send(config_args, name, command, payload)
        print("[DCA-CONFIG] " + json.dumps(result, ensure_ascii=False))


def run_checked(command: list[str], label: str) -> None:
    print(f"[{label}] " + " ".join(command))
    result = subprocess.run(command, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"{label} failed with exit code {result.returncode}")


def start_listener(args: argparse.Namespace) -> subprocess.Popen[str]:
    command = build_capture_command(args)
    print("[CAPTURE] " + " ".join(command))
    process = subprocess.Popen(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
    )
    assert process.stdout is not None
    deadline = time.monotonic() + args.listener_timeout
    while time.monotonic() < deadline:
        line = process.stdout.readline()
        if line:
            print(line, end="")
            if "[UDP] Listening on" in line:
                return process
        elif process.poll() is not None:
            raise RuntimeError(f"capture listener exited with code {process.returncode}")
    process.terminate()
    raise RuntimeError("timed out waiting for UDP 4098 listener readiness")


def stream_capture_output(process: subprocess.Popen[str]) -> int:
    assert process.stdout is not None
    for line in process.stdout:
        print(line, end="")
    return process.wait()


def run(args: argparse.Namespace) -> int:
    cfg = Path(args.cfg)
    errors = validate_cfg(cfg)
    if errors:
        raise RuntimeError("; ".join(errors))
    if not Path(args.cli_port).exists():
        raise RuntimeError(f"CLI port not found: {args.cli_port}")
    if args.duration <= 0:
        raise RuntimeError("duration must be positive")

    listener: subprocess.Popen[str] | None = None
    radar_started = False
    record_started = False
    try:
        configure_dca(args)
        run_checked(build_cli_configure_command(args), "RADAR-CONFIG")
        listener = start_listener(args)
        send_dca_command(args.dca_ip, args.config_port, DCA_CMD_RECORD_START, "record_start")
        record_started = True
        run_checked(build_cli_start_stop_command(args, "start"), "RADAR-START")
        radar_started = True
        returncode = stream_capture_output(listener)
        if returncode != 0:
            raise RuntimeError(f"capture failed with exit code {returncode}")
        return 0
    finally:
        if record_started:
            send_dca_command(args.dca_ip, args.config_port, DCA_CMD_RECORD_STOP, "record_stop")
        if radar_started:
            try:
                run_checked(build_cli_start_stop_command(args, "stop"), "RADAR-STOP")
            except Exception as exc:
                print(f"[WARN] Could not stop radar: {exc}", file=sys.stderr)
        if listener is not None and listener.poll() is None:
            listener.terminate()
            listener.wait(timeout=3)


def main(argv: list[str] | None = None) -> int:
    try:
        return run(parse_args(argv))
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
