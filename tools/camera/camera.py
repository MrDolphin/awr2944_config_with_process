"""Small manual photo/video CLI using the validated camera capture contract."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import subprocess
import sys
import time

try:
    from tools.camera.camera_capture import build_v4l2_input_args
    from tools.camera.camera_config import CameraConfig, load_camera_config
except ModuleNotFoundError:  # Supports `python tools/camera/camera.py` on the Pi.
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from tools.camera.camera_capture import build_v4l2_input_args
    from tools.camera.camera_config import CameraConfig, load_camera_config


BASE_DIR = Path(__file__).resolve().parent
CONFIG_FILE = BASE_DIR / "camera_config.cfg"
OUTPUT_DIR = BASE_DIR / "output"

_video_process: subprocess.Popen[bytes] | None = None
_video_started_at: float | None = None


def _timestamped_path(kind: str, suffix: str) -> Path:
    directory = OUTPUT_DIR / kind
    directory.mkdir(parents=True, exist_ok=True)
    name = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return directory / f"{name}{suffix}"


def print_config(config: CameraConfig) -> None:
    print(f"Device: {config.device}")
    print(f"Capture: {config.width}x{config.height}@{config.fps} {config.input_format}")
    print("Audio: disabled")


def take_photo(config: CameraConfig) -> Path:
    output_file = _timestamped_path("photos", ".jpg")
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "warning",
        *build_v4l2_input_args(config),
        "-an", "-frames:v", "1", str(output_file),
    ]
    subprocess.run(command, check=True)
    print(f"Photo saved: {output_file}")
    return output_file


def start_video(config: CameraConfig) -> Path | None:
    global _video_process, _video_started_at
    if _video_process and _video_process.poll() is None:
        print("Video recording is already running")
        return None

    output_file = _timestamped_path("videos", ".mp4")
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "warning",
        *build_v4l2_input_args(config),
        "-an", "-c:v", "libx264", "-preset", "ultrafast",
        "-tune", "zerolatency", "-pix_fmt", "yuv420p", str(output_file),
    ]
    _video_process = subprocess.Popen(command, stdin=subprocess.PIPE)
    _video_started_at = time.monotonic()
    print(f"Recording video: {output_file}")
    return output_file


def stop_video() -> None:
    global _video_process, _video_started_at
    if not _video_process:
        print("No video recording is active")
        return
    if _video_process.poll() is None and _video_process.stdin:
        _video_process.stdin.write(b"q\n")
        _video_process.stdin.flush()
        try:
            _video_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            _video_process.kill()
            _video_process.wait(timeout=5)
    elapsed = time.monotonic() - _video_started_at if _video_started_at else 0
    print(f"Video stopped after {elapsed:.1f}s")
    _video_process = None
    _video_started_at = None


def main() -> None:
    config = load_camera_config(CONFIG_FILE)
    while True:
        print("\n1: photo\n2: start video\n3: stop video\n4: show config\nq: quit")
        choice = input("Choose: ").strip().lower()
        if choice == "1":
            take_photo(config)
        elif choice == "2":
            start_video(config)
        elif choice == "3":
            stop_video()
        elif choice == "4":
            print_config(config)
        elif choice == "q":
            stop_video()
            return
        else:
            print("Unknown choice")


if __name__ == "__main__":
    main()
