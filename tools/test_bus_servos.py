#!/usr/bin/env python3
"""
Small safety test for two SP-20S-style bus servos through the STM32 controller.

Default behavior:
  1. Read all servo positions with broadcast PRAD.
  2. Move servo 001 a little and return to center.
  3. Move servo 002 a little and return to center.
  4. Move both servos together and return to center.

Example on Raspberry Pi:
  python3 tools/test_bus_servos.py --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import re
import time

import serial


POS_RE = re.compile(rb"#(\d{3})P(\d{3,4})!")


def build_move_cmd(servo_id: int, pwm: int, move_ms: int) -> bytes:
    return f"#{servo_id:03d}P{pwm:04d}T{move_ms:04d}!".encode("ascii")


def build_group_cmd(commands: list[tuple[int, int, int]]) -> bytes:
    body = b"".join(build_move_cmd(servo_id, pwm, move_ms) for servo_id, pwm, move_ms in commands)
    return b"{" + body + b"}"


def read_positions(ser: serial.Serial, wait_s: float = 0.35) -> dict[int, int]:
    ser.reset_input_buffer()
    ser.write(b"#255PRAD!")
    time.sleep(wait_s)
    data = ser.read_all()
    print(f"read raw: {data!r}")
    return {int(sid): int(pos) for sid, pos in POS_RE.findall(data)}


def send_and_wait(ser: serial.Serial, cmd: bytes, wait_s: float) -> None:
    print(f"send: {cmd!r}")
    ser.reset_input_buffer()
    ser.write(cmd)
    time.sleep(wait_s)
    echo = ser.read_all()
    if echo:
        print(f"echo/raw: {echo!r}")


def test_single_servo(
    ser: serial.Serial,
    servo_id: int,
    center_pwm: int,
    delta_pwm: int,
    move_ms: int,
) -> None:
    print(f"\n=== Test servo {servo_id:03d} ===")
    send_and_wait(ser, build_move_cmd(servo_id, center_pwm + delta_pwm, move_ms), move_ms / 1000 + 0.25)
    print("pos after +delta:", read_positions(ser))

    send_and_wait(ser, build_move_cmd(servo_id, center_pwm - delta_pwm, move_ms), move_ms / 1000 + 0.25)
    print("pos after -delta:", read_positions(ser))

    send_and_wait(ser, build_move_cmd(servo_id, center_pwm, move_ms), move_ms / 1000 + 0.25)
    print("pos after center:", read_positions(ser))


def main() -> None:
    parser = argparse.ArgumentParser(description="Test two bus servos through STM32 serial bridge.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--id1", type=int, default=1, help="First servo ID")
    parser.add_argument("--id2", type=int, default=2, help="Second servo ID")
    parser.add_argument("--center", type=int, default=1500, help="Center PWM position")
    parser.add_argument("--delta", type=int, default=80, help="Small test offset from center")
    parser.add_argument("--move-ms", type=int, default=1000, help="Move duration in milliseconds")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        print("Initial positions:", read_positions(ser))

        test_single_servo(ser, args.id1, args.center, args.delta, args.move_ms)
        test_single_servo(ser, args.id2, args.center, args.delta, args.move_ms)

        print("\n=== Test both servos together ===")
        group_cmd = build_group_cmd(
            [
                (args.id1, args.center + args.delta, args.move_ms),
                (args.id2, args.center - args.delta, args.move_ms),
            ]
        )
        send_and_wait(ser, group_cmd, args.move_ms / 1000 + 0.25)
        print("pos after group move:", read_positions(ser))

        center_cmd = build_group_cmd(
            [
                (args.id1, args.center, args.move_ms),
                (args.id2, args.center, args.move_ms),
            ]
        )
        send_and_wait(ser, center_cmd, args.move_ms / 1000 + 0.25)
        print("pos after group center:", read_positions(ser))


if __name__ == "__main__":
    main()
