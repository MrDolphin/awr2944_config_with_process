"""One-owner FFmpeg camera capture with timestamped JPEG frame buffering."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import subprocess
import threading
import time
from typing import Any

from tools.camera.camera_config import CameraConfig


_JPEG_SOI = b"\xff\xd8"
_JPEG_EOI = b"\xff\xd9"


@dataclass(frozen=True)
class CameraFrame:
    frame_id: int
    host_monotonic_ns: int
    host_wall_time_ns: int
    width: int
    height: int
    jpeg: bytes


class JpegStreamParser:
    """Extract complete JPEGs while retaining incomplete marker fragments."""

    def __init__(self) -> None:
        self._pending = b""

    def feed(self, data: bytes) -> list[bytes]:
        pending = self._pending + data
        frames: list[bytes] = []
        while pending:
            start = pending.find(_JPEG_SOI)
            if start < 0:
                self._pending = b"\xff" if pending.endswith(b"\xff") else b""
                return frames
            pending = pending[start:]
            end = pending.find(_JPEG_EOI, len(_JPEG_SOI))
            if end < 0:
                self._pending = pending
                return frames
            frames.append(pending[: end + len(_JPEG_EOI)])
            pending = pending[end + len(_JPEG_EOI) :]
        self._pending = b""
        return frames


class CameraFrameBuffer:
    """A lock-protected, bounded store of immutable camera frames."""

    def __init__(self, capacity: int = 120) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self._capacity = capacity
        self._frames: deque[CameraFrame] = deque()
        self._by_id: dict[int, CameraFrame] = {}
        self._lock = threading.RLock()

    def append(self, frame: CameraFrame) -> None:
        with self._lock:
            self._frames.append(frame)
            self._by_id[frame.frame_id] = frame
            while len(self._frames) > self._capacity:
                removed = self._frames.popleft()
                self._by_id.pop(removed.frame_id, None)

    def latest(self) -> CameraFrame | None:
        with self._lock:
            return self._frames[-1] if self._frames else None

    def get(self, frame_id: int) -> CameraFrame | None:
        with self._lock:
            return self._by_id.get(frame_id)

    def nearest(self, host_monotonic_ns: int) -> CameraFrame | None:
        with self._lock:
            if not self._frames:
                return None
            return min(
                self._frames,
                key=lambda frame: abs(frame.host_monotonic_ns - host_monotonic_ns),
            )


def build_v4l2_input_args(config: CameraConfig) -> list[str]:
    """Build validated V4L2 input arguments for every camera command."""
    return [
        "-f",
        "v4l2",
        "-input_format",
        config.input_format,
        "-framerate",
        str(config.fps),
        "-video_size",
        f"{config.width}x{config.height}",
        "-i",
        config.device,
    ]


def build_ffmpeg_capture_command(config: CameraConfig) -> list[str]:
    """Build the single-owner MJPEG-to-JPEG-pipe command without a shell."""
    return [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "warning",
        *build_v4l2_input_args(config),
        "-an",
        "-c:v",
        "copy",
        "-f",
        "image2pipe",
        "pipe:1",
    ]


class CameraRuntime:
    """The only component allowed to own the live UVC capture process."""

    def __init__(
        self,
        config: CameraConfig,
        frame_buffer: CameraFrameBuffer,
        clock: Any = time,
    ) -> None:
        self._config = config
        self._frame_buffer = frame_buffer
        self._clock = clock
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._process: subprocess.Popen[bytes] | None = None
        self._reader_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._state = "stopped"
        self._last_error = ""
        self._stderr_lines: deque[str] = deque(maxlen=20)
        self._frame_count = 0
        self._next_frame_id = 1

    def start(self) -> None:
        with self._lock:
            if self._process and self._process.poll() is None:
                return
            self._stop_event.clear()
            self._state = "starting"
            self._last_error = ""
            try:
                self._process = subprocess.Popen(
                    build_ffmpeg_capture_command(self._config),
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=0,
                )
            except OSError as error:
                self._state = "unavailable"
                self._last_error = str(error)
                raise
            self._state = "running"
            self._reader_thread = threading.Thread(
                target=self._read_frames,
                name="camera-jpeg-reader",
                daemon=True,
            )
            self._stderr_thread = threading.Thread(
                target=self._read_stderr,
                name="camera-ffmpeg-stderr",
                daemon=True,
            )
            self._reader_thread.start()
            self._stderr_thread.start()

    def stop(self, timeout_s: float = 5.0) -> None:
        self._stop_event.set()
        with self._lock:
            process = self._process
        if process and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=timeout_s)
        for thread in (self._reader_thread, self._stderr_thread):
            if thread and thread is not threading.current_thread():
                thread.join(timeout=timeout_s)
        with self._lock:
            self._process = None
            self._state = "stopped"

    def status(self) -> dict[str, object]:
        latest = self._frame_buffer.latest()
        with self._lock:
            return {
                "state": self._state,
                "frame_count": self._frame_count,
                "latest_frame_id": latest.frame_id if latest else None,
                "latest_monotonic_ns": latest.host_monotonic_ns if latest else None,
                "last_error": self._last_error,
                "stderr_tail": list(self._stderr_lines),
                "clock_basis": "host_monotonic_ns",
            }

    def _read_frames(self) -> None:
        parser = JpegStreamParser()
        while not self._stop_event.is_set():
            with self._lock:
                process = self._process
            if not process or process.stdout is None:
                return
            chunk = process.stdout.read(64 * 1024)
            if not chunk:
                return_code = process.poll()
                if return_code is not None and not self._stop_event.is_set():
                    with self._lock:
                        self._state = "unavailable"
                        self._last_error = f"ffmpeg exited unexpectedly with code {return_code}"
                    return
                time.sleep(0.01)
                continue
            for jpeg in parser.feed(chunk):
                self._append_jpeg(jpeg)

    def _append_jpeg(self, jpeg: bytes) -> None:
        with self._lock:
            frame = CameraFrame(
                frame_id=self._next_frame_id,
                host_monotonic_ns=self._clock.monotonic_ns(),
                host_wall_time_ns=self._clock.time_ns(),
                width=self._config.width,
                height=self._config.height,
                jpeg=jpeg,
            )
            self._next_frame_id += 1
            self._frame_count += 1
        self._frame_buffer.append(frame)

    def _read_stderr(self) -> None:
        with self._lock:
            process = self._process
        if not process or process.stderr is None:
            return
        while not self._stop_event.is_set():
            line = process.stderr.readline()
            if not line:
                return
            with self._lock:
                self._stderr_lines.append(line.decode("utf-8", errors="replace").rstrip())
