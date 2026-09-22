from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import io
import threading
import time
import unittest
from unittest.mock import patch

from tools.camera.camera_capture import (
    CameraFrame,
    CameraFrameBuffer,
    CameraRuntime,
    JpegStreamParser,
    build_v4l2_input_args,
)
from tools.camera.camera_config import CameraConfig


def make_frame(frame_id: int, timestamp_ns: int) -> CameraFrame:
    return CameraFrame(
        frame_id=frame_id,
        host_monotonic_ns=timestamp_ns,
        host_wall_time_ns=timestamp_ns + 100,
        width=1280,
        height=720,
        jpeg=f"frame-{frame_id}".encode(),
    )


class _FakeClock:
    def __init__(self) -> None:
        self._monotonic_ns = 1_000_000_000
        self._wall_ns = 2_000_000_000

    def monotonic_ns(self) -> int:
        value = self._monotonic_ns
        self._monotonic_ns += 40_000_000
        return value

    def time_ns(self) -> int:
        value = self._wall_ns
        self._wall_ns += 40_000_000
        return value


class _FakeStream:
    def __init__(self, chunks: list[bytes], consumed: threading.Event) -> None:
        self._chunks = deque(chunks)
        self._consumed = consumed

    def read(self, _size: int) -> bytes:
        if self._chunks:
            chunk = self._chunks.popleft()
            if not self._chunks:
                self._consumed.set()
            return chunk
        return b""

    def readline(self) -> bytes:
        return b""


class _FakeProcess:
    def __init__(self, chunks: list[bytes], consumed: threading.Event) -> None:
        self.stdout = _FakeStream(chunks, consumed)
        self.stderr = _FakeStream([], threading.Event())
        self._return_code: int | None = None
        self.terminated = False
        self.killed = False

    def poll(self) -> int | None:
        return self._return_code

    def terminate(self) -> None:
        self.terminated = True
        self._return_code = -15

    def kill(self) -> None:
        self.killed = True
        self._return_code = -9

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        return self._return_code if self._return_code is not None else 0


class CameraCaptureTests(unittest.TestCase):
    def test_input_builder_preserves_validated_capture_contract(self):
        """A legacy CLI must not substitute an unstable device or fallback mode."""
        config = CameraConfig(
            device="/dev/v4l/by-id/camera-video-index0",
            width=1280,
            height=720,
            fps=30,
        )

        self.assertEqual(
            build_v4l2_input_args(config),
            [
                "-f", "v4l2", "-input_format", "mjpeg", "-framerate", "30",
                "-video_size", "1280x720", "-i", "/dev/v4l/by-id/camera-video-index0",
            ],
        )

    def test_extracts_fragmented_jpeg_frames(self):
        """Dropping a split SOI marker would corrupt the first received frame."""
        parser = JpegStreamParser()

        self.assertEqual(parser.feed(b"noise\xff"), [])
        self.assertEqual(parser.feed(b"\xd8abc\xff\xd9"), [b"\xff\xd8abc\xff\xd9"])

    def test_extracts_two_consecutive_jpeg_frames(self):
        """Stopping after one EOI marker would silently discard adjacent frames."""
        parser = JpegStreamParser()

        self.assertEqual(
            parser.feed(b"\xff\xd8first\xff\xd9\xff\xd8second\xff\xd9"),
            [b"\xff\xd8first\xff\xd9", b"\xff\xd8second\xff\xd9"],
        )

    def test_ring_buffer_evicts_old_frames_and_returns_nearest_frame(self):
        """An unbounded buffer would grow throughout a long capture session."""
        buffer = CameraFrameBuffer(capacity=3)
        buffer.append(make_frame(1, 1_000_000_000))
        buffer.append(make_frame(2, 1_040_000_000))
        buffer.append(make_frame(3, 1_080_000_000))
        buffer.append(make_frame(4, 1_120_000_000))

        self.assertIsNone(buffer.get(1))
        self.assertEqual(buffer.nearest(1_030_000_000).frame_id, 2)
        self.assertEqual(buffer.latest().frame_id, 4)

    def test_runtime_assigns_monotonic_ids_and_stops_owned_process(self):
        """Reusing frame IDs or leaving FFmpeg alive would break matching and device ownership."""
        consumed = threading.Event()
        process = _FakeProcess(
            [b"\xff\xd8one\xff\xd9", b"\xff\xd8two\xff\xd9"],
            consumed,
        )
        buffer = CameraFrameBuffer()
        runtime = CameraRuntime(
            CameraConfig(device="/dev/video0", width=1280, height=720, fps=30),
            buffer,
            clock=_FakeClock(),
        )

        with patch("tools.camera.camera_capture.subprocess.Popen", return_value=process):
            runtime.start()
            self.assertTrue(consumed.wait(timeout=1))
            deadline = time.monotonic() + 1
            while buffer.latest() is None or buffer.latest().frame_id != 2:
                if time.monotonic() >= deadline:
                    self.fail("camera runtime did not store both JPEG frames")
                time.sleep(0.01)
            runtime.stop()

        latest = buffer.latest()
        self.assertEqual(latest.frame_id, 2)
        self.assertEqual(latest.host_monotonic_ns, 1_040_000_000)
        self.assertTrue(process.terminated)
        self.assertEqual(runtime.status()["state"], "stopped")


if __name__ == "__main__":
    unittest.main()
