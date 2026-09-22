from __future__ import annotations

import json
from urllib.error import HTTPError
from urllib.request import urlopen
import unittest

from tools.camera.camera_capture import CameraFrame, CameraFrameBuffer
from tools.camera.camera_http import CameraHttpServer


class _Runtime:
    def status(self) -> dict[str, object]:
        return {"state": "running", "frame_count": 2, "last_error": ""}


def _frame(frame_id: int) -> CameraFrame:
    return CameraFrame(
        frame_id=frame_id,
        host_monotonic_ns=1_000_000_000 + frame_id,
        host_wall_time_ns=2_000_000_000 + frame_id,
        width=1280,
        height=720,
        jpeg=b"\xff\xd8frame-" + str(frame_id).encode() + b"\xff\xd9",
    )


class CameraHttpServerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.buffer = CameraFrameBuffer()
        self.buffer.append(_frame(17))
        self.buffer.append(_frame(18))
        self.server = CameraHttpServer("127.0.0.1", 0, _Runtime(), self.buffer)
        self.server.start()
        self.base_url = f"http://127.0.0.1:{self.server.bound_port}"

    def tearDown(self) -> None:
        self.server.stop()

    def test_status_returns_runtime_and_latest_frame_metadata(self):
        """Dropping latest-frame metadata would make browser synchronization impossible."""
        with urlopen(f"{self.base_url}/camera/status") as response:
            payload = json.loads(response.read())

        self.assertEqual(response.status, 200)
        self.assertEqual(payload["state"], "running")
        self.assertEqual(payload["latest_frame_id"], 18)
        self.assertIn("fps_observed", payload)

    def test_latest_frame_returns_jpeg_with_timestamp_headers(self):
        """Removing capture headers would leave the browser with image bytes but no match evidence."""
        with urlopen(f"{self.base_url}/camera/frame/latest.jpg") as response:
            body = response.read()
            headers = response.headers

        self.assertEqual(body, _frame(18).jpeg)
        self.assertEqual(headers["X-Camera-Frame-Id"], "18")
        self.assertEqual(headers["X-Capture-Monotonic-Ns"], "1000000018")
        self.assertEqual(headers["X-Capture-Wall-Time-Ns"], "2000000018")
        self.assertEqual(headers["Cache-Control"], "no-store")
        self.assertEqual(headers["Access-Control-Allow-Origin"], "*")
        self.assertEqual(
            headers["Access-Control-Expose-Headers"],
            "X-Camera-Frame-Id, X-Capture-Monotonic-Ns, X-Capture-Wall-Time-Ns",
        )

    def test_retained_frame_is_exact_and_missing_frame_is_json_404(self):
        """A stale requested frame must not be silently replaced by the latest frame."""
        with urlopen(f"{self.base_url}/camera/frame/17.jpg") as response:
            self.assertEqual(response.read(), _frame(17).jpeg)

        with self.assertRaises(HTTPError) as raised:
            urlopen(f"{self.base_url}/camera/frame/999.jpg")
        self.assertEqual(raised.exception.code, 404)
        self.assertEqual(json.loads(raised.exception.read())["error"], "camera frame not found")

    def test_unknown_path_returns_json_404(self):
        """An unrecognised endpoint must not leak a default HTML response contract."""
        with self.assertRaises(HTTPError) as raised:
            urlopen(f"{self.base_url}/unknown")
        self.assertEqual(raised.exception.code, 404)
        self.assertEqual(json.loads(raised.exception.read())["error"], "not found")


if __name__ == "__main__":
    unittest.main()
