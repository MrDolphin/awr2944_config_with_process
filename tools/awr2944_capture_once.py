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
from typing import Any

try:
    from tools import dca1000_configure
    from tools.dca1000_capture import DCA_CMD_RECORD_START, DCA_CMD_RECORD_STOP, send_dca_command
except ModuleNotFoundError:  # Direct ``python tools/awr2944_capture_once.py``.
    import dca1000_configure
    from dca1000_capture import DCA_CMD_RECORD_START, DCA_CMD_RECORD_STOP, send_dca_command


TOOLS_DIR = Path(__file__).resolve().parent
DEFAULTS_PATH = Path("Config/awr2944_capture_defaults.json")

# Keep this list explicit so a typo in the local JSON cannot silently alter a
# hardware run.  CLI options always take precedence over these defaults.
DEFAULT_SETTING_NAMES = {
    "cfg",
    "cli_port",
    "baud",
    "dca_ip",
    "system_ip",
    "dca_mac",
    "config_port",
    "data_port",
    "packet_delay_us",
    "dca_lvds_mode",
    "duration",
    "output_dir",
    "prefix",
    "min_free_gb",
    "socket_buffer_mb",
    "cli_delay",
    "dca_timeout",
    "listener_timeout",
    "analyze_range",
    "post_analyze",
    "analysis_max_range_m",
}


def find_defaults_path(argv: list[str] | None) -> Path:
    """Read only ``--defaults`` before constructing the full CLI parser."""
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument("--defaults", default=str(DEFAULTS_PATH))
    known, _ = bootstrap.parse_known_args(argv)
    return Path(known.defaults)


def load_defaults(path: Path) -> dict[str, Any]:
    """Load one local JSON profile and reject unknown settings early."""
    if not path.is_file():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"defaults file must contain a JSON object: {path}")
    unknown = sorted(set(value) - DEFAULT_SETTING_NAMES)
    if unknown:
        raise ValueError(f"unknown defaults setting(s) in {path}: {', '.join(unknown)}")
    return value


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    defaults_path = find_defaults_path(argv)
    defaults = load_defaults(defaults_path)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--defaults",
        default=str(defaults_path),
        help=(
            "Local JSON settings file. Defaults to Config/awr2944_capture_defaults.json; "
            "copy the tracked .example file once, then CLI arguments override it."
        ),
    )
    parser.add_argument("--cfg", help="Radar CFG with lvdsStreamCfg enabled.")
    parser.add_argument("--cli-port", default="/dev/ttyACM0", help="AWR2944P CLI serial device.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--dca-ip", default="192.168.33.180")
    parser.add_argument("--system-ip", default="192.168.33.30")
    parser.add_argument("--dca-mac", default="12.34.56.78.90.12")
    parser.add_argument("--config-port", type=int, default=4096)
    parser.add_argument("--data-port", type=int, default=4098)
    parser.add_argument("--packet-delay-us", type=int, default=25)
    parser.add_argument(
        "--dca-lvds-mode",
        type=int,
        choices=(1, 2),
        default=2,
        help="DCA1000 LVDS mode: 1 = 4 lane, 2 = 2 lane; must match physical SW2.3.",
    )
    parser.add_argument("--duration", type=float, default=60.0, help="Capture duration in seconds.")
    parser.add_argument("--output-dir", default="/home/pi/radar_runs/awr2944p")
    parser.add_argument("--prefix", default="adc_data")
    parser.add_argument("--min-free-gb", type=float, default=4.0)
    parser.add_argument("--socket-buffer-mb", type=int, default=64)
    parser.add_argument("--cli-delay", type=float, default=0.15)
    parser.add_argument("--dca-timeout", type=float, default=5.0)
    parser.add_argument("--listener-timeout", type=float, default=10.0)
    parser.add_argument(
        "--analyze-range",
        action="store_true",
        help="After a successful capture and safe radar/DCA shutdown, generate range-domain analysis artifacts.",
    )
    parser.add_argument(
        "--post-analyze",
        action="store_true",
        help=(
            "After safe shutdown, generate range artifacts and a capture-level gate report. "
            "Unperformed absolute-range, phase, Doppler and AoA work is explicitly marked."
        ),
    )
    parser.add_argument("--analysis-max-range-m", type=float, default=15.0)
    parser.add_argument("--show-effective-config", action="store_true", help="Print merged defaults and CLI options, then exit without hardware I/O.")
    # Apply local settings after every action has declared its built-in
    # fallback.  Command-line arguments parsed below still override these.
    parser.set_defaults(**defaults)
    args = parser.parse_args(argv)
    if not args.cfg and not args.show_effective_config:
        parser.error("--cfg is required unless it is provided by --defaults")
    return args


def effective_config(args: argparse.Namespace) -> dict[str, Any]:
    """Return serializable settings actually selected for a future capture."""
    return {name: getattr(args, name) for name in sorted(DEFAULT_SETTING_NAMES)}


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


def build_range_analysis_command(args: argparse.Namespace, bin_path: Path) -> list[str]:
    """Build post-capture analysis; never runs until capture cleanup completed."""
    return [
        sys.executable,
        str(TOOLS_DIR / "analyze_adc_range.py"),
        "--bin", str(bin_path),
        "--cfg", str(args.cfg),
        "--output-dir", str(bin_path.parent / "range_analysis"),
        "--max-range-m", str(args.analysis_max_range_m),
        "--remove-mean",
    ]


def build_post_capture_analysis_command(args: argparse.Namespace, bin_path: Path) -> list[str]:
    """Build the capture-level report after range-domain artifacts are present."""
    return [
        sys.executable,
        str(TOOLS_DIR / "post_capture_analysis.py"),
        "--bin", str(bin_path),
        "--cfg", str(args.cfg),
        "--metadata", str(bin_path.with_suffix(".json")),
        "--range-analysis-dir", str(bin_path.parent / "range_analysis"),
        "--output-dir", str(bin_path.parent),
    ]


def should_run_range_analysis(args: argparse.Namespace) -> bool:
    """Post-analysis includes range analysis, so users need only one flag."""
    return bool(args.analyze_range or args.post_analyze)


def configure_dca(args: argparse.Namespace) -> None:
    config_args = argparse.Namespace(
        dca_ip=args.dca_ip,
        system_ip=args.system_ip,
        mac=args.dca_mac,
        config_port=args.config_port,
        packet_delay_us=args.packet_delay_us,
        lvds_mode=args.dca_lvds_mode,
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


def start_listener(args: argparse.Namespace) -> tuple[subprocess.Popen[str], Path]:
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
    bin_path: Path | None = None
    deadline = time.monotonic() + args.listener_timeout
    while time.monotonic() < deadline:
        line = process.stdout.readline()
        if line:
            print(line, end="")
            if line.startswith("[OUT] "):
                candidate = Path(line.removeprefix("[OUT] ").strip())
                if candidate.suffix.lower() == ".bin":
                    bin_path = candidate
            if "[UDP] Listening on" in line:
                if bin_path is None:
                    process.terminate()
                    raise RuntimeError("capture listener did not report its output BIN path")
                return process, bin_path
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
    if args.show_effective_config:
        print(json.dumps(effective_config(args), ensure_ascii=False, indent=2))
        return 0
    cfg = Path(args.cfg)
    errors = validate_cfg(cfg)
    if errors:
        raise RuntimeError("; ".join(errors))
    if not Path(args.cli_port).exists():
        raise RuntimeError(f"CLI port not found: {args.cli_port}")
    if args.duration <= 0:
        raise RuntimeError("duration must be positive")

    listener: subprocess.Popen[str] | None = None
    captured_bin: Path | None = None
    radar_started = False
    record_started = False
    capture_succeeded = False
    try:
        configure_dca(args)
        run_checked(build_cli_configure_command(args), "RADAR-CONFIG")
        listener, captured_bin = start_listener(args)
        send_dca_command(args.dca_ip, args.config_port, DCA_CMD_RECORD_START, "record_start")
        record_started = True
        run_checked(build_cli_start_stop_command(args, "start"), "RADAR-START")
        radar_started = True
        returncode = stream_capture_output(listener)
        if returncode != 0:
            raise RuntimeError(f"capture failed with exit code {returncode}")
        capture_succeeded = True
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

    if capture_succeeded and should_run_range_analysis(args):
        if captured_bin is None:
            raise RuntimeError("capture completed without a reported BIN path")
        run_checked(build_range_analysis_command(args, captured_bin), "RANGE-ANALYSIS")
    if capture_succeeded and args.post_analyze:
        if captured_bin is None:
            raise RuntimeError("capture completed without a reported BIN path")
        run_checked(build_post_capture_analysis_command(args, captured_bin), "POST-ANALYSIS")
    return 0


def main(argv: list[str] | None = None) -> int:
    try:
        return run(parse_args(argv))
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
