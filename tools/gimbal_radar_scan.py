#!/usr/bin/env python3
"""
Two-axis bus-servo gimbal scanner for radar point-cloud capture.

Default plan:
  - ID001 stays at a fixed pitch angle.
  - ID002 sweeps yaw from -120 deg to +120 deg and back.
  - At each yaw stop, read actual servo position and collect radar frames from
    radar_server.py WebSocket.
  - Save raw synchronized frames and a simple stitched XY point CSV.

Run on Raspberry Pi, for example:
  python3 tools/gimbal_radar_scan.py --servo-port /dev/ttyUSB0 --ws ws://127.0.0.1:8765

Servo angle mapping assumes a 270-degree bus-servo mode:
  P1500 = 0 deg, P500 ~= -135 deg, P2500 ~= +135 deg
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import re
import signal
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import serial
import websockets


POS_RE = re.compile(rb"#(\d{3})P(\d{3,4})!")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def deg_to_pwm(deg: float, center_pwm: int, us_per_deg: float, min_pwm: int, max_pwm: int) -> int:
    return int(round(clamp(center_pwm + deg * us_per_deg, min_pwm, max_pwm)))


def pwm_to_deg(pwm: int, center_pwm: int, us_per_deg: float) -> float:
    return (pwm - center_pwm) / us_per_deg


def build_move_cmd(servo_id: int, pwm: int, move_ms: int) -> bytes:
    return f"#{servo_id:03d}P{pwm:04d}T{move_ms:04d}!".encode("ascii")


def read_positions(ser: serial.Serial, wait_s: float = 0.12) -> dict[int, int]:
    ser.reset_input_buffer()
    ser.write(b"#255PRAD!")
    time.sleep(wait_s)
    data = ser.read_all()
    return {int(sid): int(pos) for sid, pos in POS_RE.findall(data)}


def move_servo(ser: serial.Serial, servo_id: int, pwm: int, move_ms: int) -> None:
    ser.write(build_move_cmd(servo_id, pwm, move_ms))


def make_sweep_angles(min_deg: float, max_deg: float, step_deg: float) -> list[float]:
    if step_deg <= 0:
        raise ValueError("step_deg must be positive")

    forward: list[float] = []
    current = min_deg
    while current <= max_deg + 1e-6:
        forward.append(round(current, 6))
        current += step_deg
    if forward[-1] != max_deg:
        forward.append(max_deg)

    backward = list(reversed(forward[1:-1]))
    return forward + backward


def transform_point_by_yaw(point: dict[str, Any], yaw_deg: float) -> dict[str, Any]:
    """Rotate radar-local x/y point into scan/global frame using yaw angle."""
    yaw = math.radians(yaw_deg)
    x = float(point.get("x", 0.0))
    y = float(point.get("y", 0.0))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    return {
        "x_global_m": cos_yaw * x - sin_yaw * y,
        "y_global_m": sin_yaw * x + cos_yaw * y,
        "x_local_m": x,
        "y_local_m": y,
        "z_m": point.get("z", 0.0),
        "v_mps": point.get("v", 0.0),
        "snr_db": point.get("snr"),
        "noise_db": point.get("noise"),
    }


async def collect_frames(ws_url: str, seconds: float, max_frames: int) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    deadline = time.monotonic() + seconds

    try:
        async with websockets.connect(ws_url, ping_interval=None) as ws:
            while time.monotonic() < deadline and len(frames) < max_frames:
                timeout = max(0.05, deadline - time.monotonic())
                try:
                    message = await asyncio.wait_for(ws.recv(), timeout=timeout)
                except asyncio.TimeoutError:
                    break

                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    continue

                if isinstance(data, dict) and "points" in data:
                    frames.append(data)
    except OSError as exc:
        print(f"WARNING: cannot connect to radar WebSocket {ws_url}: {exc}")

    return frames


async def run_scan(args: argparse.Namespace) -> None:
    run_dir = Path(args.out_dir) / datetime.now().strftime("gimbal_scan_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    raw_path = run_dir / "sync_frames.jsonl"
    point_path = run_dir / "stitched_points.csv"
    meta_path = run_dir / "meta.json"

    yaw_us_per_deg = args.yaw_us_per_degree
    pitch_us_per_deg = args.pitch_us_per_degree
    yaw_angles = make_sweep_angles(args.yaw_min_deg, args.yaw_max_deg, args.yaw_step_deg)
    stop_requested = False

    def request_stop(signum: int, frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True
        print(f"\nStop requested by signal {signum}; centering servos after current step...")

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    meta = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "servo_port": args.servo_port,
        "baud": args.baud,
        "pitch_id": args.pitch_id,
        "yaw_id": args.yaw_id,
        "pitch_fixed_deg": args.pitch_fixed_deg,
        "yaw_min_deg": args.yaw_min_deg,
        "yaw_max_deg": args.yaw_max_deg,
        "yaw_step_deg": args.yaw_step_deg,
        "move_ms": args.move_ms,
        "settle_s": args.settle_s,
        "capture_s": args.capture_s,
        "frames_per_angle": args.frames_per_angle,
        "ws": args.ws,
        "angle_mapping": {
            "assumption": "270-degree servo mode, P1500=0 deg",
            "yaw_center_pwm": args.yaw_center_pwm,
            "yaw_us_per_degree": yaw_us_per_deg,
            "pitch_center_pwm": args.pitch_center_pwm,
            "pitch_us_per_degree": pitch_us_per_deg,
            "min_pwm": args.min_pwm,
            "max_pwm": args.max_pwm,
        },
    }
    meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

    with serial.Serial(args.servo_port, args.baud, timeout=0.2) as ser, raw_path.open(
        "w", encoding="utf-8"
    ) as raw_file, point_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "timestamp_s",
                "cycle",
                "yaw_cmd_deg",
                "yaw_actual_deg",
                "yaw_pwm",
                "pitch_cmd_deg",
                "pitch_actual_deg",
                "pitch_pwm",
                "frame_num",
                "point_index",
                "x_global_m",
                "y_global_m",
                "x_local_m",
                "y_local_m",
                "z_m",
                "v_mps",
                "snr_db",
                "noise_db",
            ],
        )
        writer.writeheader()

        pitch_pwm = deg_to_pwm(
            args.pitch_fixed_deg, args.pitch_center_pwm, pitch_us_per_deg, args.min_pwm, args.max_pwm
        )
        print(f"Set pitch servo ID{args.pitch_id:03d}: {args.pitch_fixed_deg:.1f} deg -> P{pitch_pwm}")
        move_servo(ser, args.pitch_id, pitch_pwm, args.move_ms)
        time.sleep(args.move_ms / 1000 + args.settle_s)

        try:
            cycle = 0
            while not stop_requested and (args.cycles <= 0 or cycle < args.cycles):
                for yaw_cmd_deg in yaw_angles:
                    if stop_requested:
                        break

                    yaw_pwm_cmd = deg_to_pwm(
                        yaw_cmd_deg, args.yaw_center_pwm, yaw_us_per_deg, args.min_pwm, args.max_pwm
                    )
                    print(f"[cycle {cycle}] yaw {yaw_cmd_deg:+.1f} deg -> P{yaw_pwm_cmd}")
                    move_servo(ser, args.yaw_id, yaw_pwm_cmd, args.move_ms)
                    time.sleep(args.move_ms / 1000 + args.settle_s)

                    positions = read_positions(ser)
                    yaw_pwm_actual = positions.get(args.yaw_id, yaw_pwm_cmd)
                    pitch_pwm_actual = positions.get(args.pitch_id, pitch_pwm)
                    yaw_actual_deg = pwm_to_deg(yaw_pwm_actual, args.yaw_center_pwm, yaw_us_per_deg)
                    pitch_actual_deg = pwm_to_deg(pitch_pwm_actual, args.pitch_center_pwm, pitch_us_per_deg)

                    frames = await collect_frames(args.ws, args.capture_s, args.frames_per_angle)
                    print(
                        f"  actual yaw={yaw_actual_deg:+.1f} deg P{yaw_pwm_actual}, "
                        f"frames={len(frames)}"
                    )

                    for frame in frames:
                        record = {
                            "timestamp_s": time.time(),
                            "cycle": cycle,
                            "yaw_cmd_deg": yaw_cmd_deg,
                            "yaw_actual_deg": yaw_actual_deg,
                            "yaw_pwm": yaw_pwm_actual,
                            "pitch_cmd_deg": args.pitch_fixed_deg,
                            "pitch_actual_deg": pitch_actual_deg,
                            "pitch_pwm": pitch_pwm_actual,
                            "frame": frame,
                        }
                        raw_file.write(json.dumps(record, ensure_ascii=False) + "\n")

                        points = frame.get("points") or []
                        for point_index, point in enumerate(points):
                            transformed = transform_point_by_yaw(point, yaw_actual_deg)
                            writer.writerow(
                                {
                                    "timestamp_s": record["timestamp_s"],
                                    "cycle": cycle,
                                    "yaw_cmd_deg": yaw_cmd_deg,
                                    "yaw_actual_deg": yaw_actual_deg,
                                    "yaw_pwm": yaw_pwm_actual,
                                    "pitch_cmd_deg": args.pitch_fixed_deg,
                                    "pitch_actual_deg": pitch_actual_deg,
                                    "pitch_pwm": pitch_pwm_actual,
                                    "frame_num": frame.get("frame_num"),
                                    "point_index": point_index,
                                    **transformed,
                                }
                            )

                    raw_file.flush()
                    csv_file.flush()

                cycle += 1
        finally:
            print("Centering yaw and keeping pitch fixed...")
            move_servo(ser, args.yaw_id, args.yaw_center_pwm, args.move_ms)
            move_servo(ser, args.pitch_id, pitch_pwm, args.move_ms)
            time.sleep(args.move_ms / 1000 + 0.2)

    print(f"Saved raw frames: {raw_path}")
    print(f"Saved stitched points: {point_path}")
    print(f"Saved metadata: {meta_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Sweep yaw servo and synchronize radar point-cloud frames.")
    parser.add_argument("--servo-port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--ws", default="ws://127.0.0.1:8765", help="radar_server.py WebSocket URL")
    parser.add_argument("--out-dir", default="record/gimbal_scans")

    parser.add_argument("--pitch-id", type=int, default=1, help="Fixed pitch/z-axis servo ID")
    parser.add_argument("--yaw-id", type=int, default=2, help="Sweeping horizontal/yaw servo ID")
    parser.add_argument("--pitch-fixed-deg", type=float, default=0.0)

    parser.add_argument("--yaw-min-deg", type=float, default=-120.0)
    parser.add_argument("--yaw-max-deg", type=float, default=120.0)
    parser.add_argument("--yaw-step-deg", type=float, default=10.0)
    parser.add_argument("--cycles", type=int, default=0, help="0 means loop forever until Ctrl+C")

    parser.add_argument("--move-ms", type=int, default=800)
    parser.add_argument("--settle-s", type=float, default=0.25)
    parser.add_argument("--capture-s", type=float, default=0.35)
    parser.add_argument("--frames-per-angle", type=int, default=8)

    parser.add_argument("--pitch-center-pwm", type=int, default=1500)
    parser.add_argument("--yaw-center-pwm", type=int, default=1500)
    parser.add_argument("--min-pwm", type=int, default=500)
    parser.add_argument("--max-pwm", type=int, default=2500)
    parser.add_argument(
        "--pitch-us-per-degree",
        type=float,
        default=2000.0 / 270.0,
        help="PWM units per degree; default assumes P500-P2500 maps to 270 deg",
    )
    parser.add_argument(
        "--yaw-us-per-degree",
        type=float,
        default=2000.0 / 270.0,
        help="PWM units per degree; default assumes P500-P2500 maps to 270 deg",
    )

    args = parser.parse_args()
    asyncio.run(run_scan(args))


if __name__ == "__main__":
    main()
