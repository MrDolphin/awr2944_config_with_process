"""Read-only validation of the configured UVC MJPEG capture mode."""

import argparse
from pathlib import Path
import re
import subprocess
import sys

from tools.camera.camera_config import CameraConfig, load_camera_config


_FORMAT_PATTERN = re.compile(r"^\s*\[\d+\]:\s*'(?P<format>[A-Z0-9]{4})'")
_SIZE_PATTERN = re.compile(r"^\s*Size:\s*Discrete\s+(?P<width>\d+)x(?P<height>\d+)")
_FPS_PATTERN = re.compile(r"\((?P<fps>\d+(?:\.\d+)?)\s+fps\)")


def parse_mjpeg_modes(v4l2_output: str) -> set[tuple[int, int, int]]:
    """Return discrete ``(width, height, fps)`` modes from MJPEG v4l2 output."""
    modes: set[tuple[int, int, int]] = set()
    is_mjpeg = False
    size: tuple[int, int] | None = None
    for line in v4l2_output.splitlines():
        format_match = _FORMAT_PATTERN.match(line)
        if format_match:
            is_mjpeg = format_match.group("format") == "MJPG"
            size = None
            continue
        if not is_mjpeg:
            continue
        size_match = _SIZE_PATTERN.match(line)
        if size_match:
            size = (int(size_match.group("width")), int(size_match.group("height")))
            continue
        fps_match = _FPS_PATTERN.search(line)
        if size and fps_match:
            modes.add((size[0], size[1], round(float(fps_match.group("fps")))))
    return modes


def probe_camera(config: CameraConfig) -> int:
    """Print supported MJPEG modes and return non-zero for an unsupported selection."""
    command = ["v4l2-ctl", "--list-formats-ext", "--device", config.device]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode:
        print(completed.stderr.strip() or "v4l2-ctl failed", file=sys.stderr)
        return completed.returncode

    selected_mode = (config.width, config.height, config.fps)
    supported_modes = parse_mjpeg_modes(completed.stdout)
    if selected_mode not in supported_modes:
        print(
            f"unsupported MJPEG mode: {config.width}x{config.height}@{config.fps}; "
            f"supported modes: {sorted(supported_modes)}",
            file=sys.stderr,
        )
        return 1

    print(f"supported MJPEG mode: {config.width}x{config.height}@{config.fps}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate a camera mode without opening a stream")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("camera_config.cfg"),
    )
    arguments = parser.parse_args()
    return probe_camera(load_camera_config(arguments.config))


if __name__ == "__main__":
    raise SystemExit(main())
