"""Command-line interface for the JY931 acquisition tool."""

import argparse
from datetime import datetime
import signal
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, TextIO

from .protocol import SAMPLE_TYPES, Sample
from .reader import SerialFrameReader, SerialOpenError
from .recorder import CsvRecorder


DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "output"


def nonnegative_float(value: str) -> float:
    number = float(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return number


def positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be > 0")
    return number


def parse_types(value: str):
    if value.strip().lower() == "all":
        return set(SAMPLE_TYPES)
    selected = {
        item.strip().lower() for item in value.split(",") if item.strip()
    }
    unknown = sorted(selected - SAMPLE_TYPES)
    if unknown:
        raise argparse.ArgumentTypeError(
            "unknown sample type(s): {}; choose from {}".format(
                ", ".join(unknown), ", ".join(sorted(SAMPLE_TYPES))
            )
        )
    return selected


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="JY931 IMU data recorder")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=positive_int, default=921600)
    parser.add_argument(
        "--output",
        "-o",
        help="CSV output path; defaults to a timestamped file in the package output directory",
    )
    parser.add_argument(
        "--rate",
        type=nonnegative_float,
        default=10.0,
        help="display rate in Hz; 0 prints every selected frame",
    )
    parser.add_argument(
        "--types",
        type=parse_types,
        default=parse_types("all"),
        help="display types: acc,gyro,angle,mag,quat or all",
    )
    parser.add_argument(
        "--flush-interval",
        type=nonnegative_float,
        default=1.0,
        help="CSV flush interval in seconds; 0 flushes every row",
    )
    parser.add_argument("--no-print", action="store_true")
    parser.add_argument(
        "--no-output",
        action="store_true",
        help="display data without saving a CSV file",
    )
    return parser


def resolve_output_path(args, moment: Optional[datetime] = None) -> Optional[Path]:
    """Return the explicit path, the automatic default, or None."""
    if getattr(args, "no_output", False):
        return None
    if args.output:
        return Path(args.output).expanduser()

    moment = moment or datetime.now()
    filename = "imu_{}.csv".format(moment.strftime("%Y%m%d_%H%M%S"))
    return DEFAULT_OUTPUT_DIR / filename


def format_sample(sample: Sample) -> str:
    if sample.type == "quat":
        q0, q1, q2, q3 = sample.values
        text = "quat  : q0={:+.4f} q1={:+.4f} q2={:+.4f} q3={:+.4f}".format(
            q0, q1, q2, q3
        )
        if sample.norm is not None and not 0.9 < sample.norm < 1.1:
            text += " norm={:.3f}".format(sample.norm)
        return text
    if sample.type == "mag":
        return "mag   : {:+7.0f} {:+7.0f} {:+7.0f} mG".format(*sample.values)
    return "{:6s}: {:+9.3f} {:+9.3f} {:+9.3f}".format(
        sample.type, *sample.values
    )


@dataclass
class RunStats:
    started_monotonic: float
    frames: int = 0
    invalid_frames: int = 0
    type_counts: Dict[str, int] = field(default_factory=lambda: defaultdict(int))
    elapsed: float = 0.0


def run(
    args,
    stop_event: Optional[threading.Event] = None,
    stdout: Optional[TextIO] = None,
    serial_factory=None,
) -> RunStats:
    stdout = stdout or sys.stdout
    stop_event = stop_event or threading.Event()
    stats = RunStats(started_monotonic=time.monotonic())
    output_path = resolve_output_path(args)
    reader = SerialFrameReader(
        port=args.port,
        baudrate=args.baud,
        serial_factory=serial_factory,
        audit=lambda message: print("[serial] {}".format(message), file=stdout),
    )

    reader.open()
    try:
        recorder = None
        if output_path is not None:
            recorder = CsvRecorder(
                output_path,
                flush_interval=args.flush_interval,
            )
    except Exception:
        reader.close()
        raise

    try:
        print(
            "serial: {} @ {} | display: {} Hz | types: {}".format(
                args.port,
                args.baud,
                args.rate,
                ", ".join(sorted(args.types)),
            ),
            file=stdout,
        )
        if recorder is not None:
            print("csv: {}".format(recorder.path), file=stdout)
        if args.no_print:
            print("terminal display disabled", file=stdout)
        print("Ctrl+C stops collection", file=stdout)

        print_interval = 1.0 / args.rate if args.rate > 0 else 0.0
        last_print = 0.0
        for sample in reader.frames(stop_event=stop_event):
            stats.frames += 1
            stats.type_counts[sample.type] += 1

            if recorder is not None:
                recorder.write(sample)

            if args.no_print or sample.type not in args.types:
                continue
            now = time.monotonic()
            if print_interval == 0.0 or now - last_print >= print_interval:
                last_print = now
                print("[{:6d}] {}".format(stats.frames, format_sample(sample)), file=stdout)
    finally:
        if recorder is not None:
            recorder.close()
        reader.close()
        stats.invalid_frames = reader.parser.invalid_frames
        stats.elapsed = time.monotonic() - stats.started_monotonic
        print_summary(
            stats,
            recorder.path if recorder is not None else None,
            stdout,
        )

    return stats


def print_summary(stats: RunStats, output: Optional[Path], stdout: TextIO):
    print("\n" + "=" * 50, file=stdout)
    print(
        "duration: {:.1f}s | frames: {} | dropped: {}".format(
            stats.elapsed, stats.frames, stats.invalid_frames
        ),
        file=stdout,
    )
    if stats.elapsed > 0:
        print("rate: {:.1f} fps".format(stats.frames / stats.elapsed), file=stdout)
    for sample_type in sorted(stats.type_counts):
        count = stats.type_counts[sample_type]
        rate = count / stats.elapsed if stats.elapsed else 0.0
        print(
            "  {:6s}: {:6d} ({:.1f} Hz)".format(sample_type, count, rate),
            file=stdout,
        )
    print("=" * 50, file=stdout)
    if output:
        print("csv saved: {}".format(output), file=stdout)


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    stop_event = threading.Event()

    def request_stop(signum=None, frame=None):
        stop_event.set()

    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGINT, request_stop)
        if hasattr(signal, "SIGTERM"):
            signal.signal(signal.SIGTERM, request_stop)

    try:
        run(args, stop_event=stop_event)
        return 0
    except SerialOpenError as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130
