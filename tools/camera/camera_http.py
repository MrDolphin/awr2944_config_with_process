"""Bounded HTTP access to timestamped frames from one CameraRuntime."""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import re
import threading
from typing import Any

from tools.camera.camera_capture import CameraFrame, CameraFrameBuffer, CameraRuntime


_FRAME_PATH = re.compile(r"^/camera/frame/(?P<frame_id>\d+)\.jpg$")


class CameraHttpServer:
    def __init__(
        self,
        host: str,
        port: int,
        runtime: CameraRuntime,
        frame_buffer: CameraFrameBuffer,
    ) -> None:
        self._runtime = runtime
        self._frame_buffer = frame_buffer
        self._httpd = ThreadingHTTPServer((host, port), self._make_handler())
        self._thread: threading.Thread | None = None

    @property
    def bound_port(self) -> int:
        return int(self._httpd.server_address[1])

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._httpd.serve_forever,
            name="camera-http-server",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread:
            self._thread.join(timeout=5)

    def _make_handler(self) -> type[BaseHTTPRequestHandler]:
        owner = self

        class CameraRequestHandler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802 - required by BaseHTTPRequestHandler
                if self.path == "/camera/status":
                    owner._send_status(self)
                    return
                if self.path == "/camera/frame/latest.jpg":
                    owner._send_frame(self, owner._frame_buffer.latest())
                    return
                match = _FRAME_PATH.fullmatch(self.path)
                if match:
                    owner._send_frame(self, owner._frame_buffer.get(int(match.group("frame_id"))))
                    return
                owner._send_json(self, HTTPStatus.NOT_FOUND, {"error": "not found"})

            def log_message(self, _format: str, *args: object) -> None:
                del args

        return CameraRequestHandler

    def _send_status(self, handler: BaseHTTPRequestHandler) -> None:
        payload = dict(self._runtime.status())
        latest = self._frame_buffer.latest()
        payload.update(
            {
                "latest_frame_id": latest.frame_id if latest else None,
                "latest_monotonic_ns": latest.host_monotonic_ns if latest else None,
                "fps_observed": self._frame_buffer.observed_fps(),
            }
        )
        self._send_json(handler, HTTPStatus.OK, payload)

    def _send_frame(self, handler: BaseHTTPRequestHandler, frame: CameraFrame | None) -> None:
        if frame is None:
            self._send_json(handler, HTTPStatus.NOT_FOUND, {"error": "camera frame not found"})
            return
        handler.send_response(HTTPStatus.OK)
        handler.send_header("Content-Type", "image/jpeg")
        handler.send_header("Content-Length", str(len(frame.jpeg)))
        handler.send_header("X-Camera-Frame-Id", str(frame.frame_id))
        handler.send_header("X-Capture-Monotonic-Ns", str(frame.host_monotonic_ns))
        handler.send_header("X-Capture-Wall-Time-Ns", str(frame.host_wall_time_ns))
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Access-Control-Allow-Origin", "*")
        handler.send_header(
            "Access-Control-Expose-Headers",
            "X-Camera-Frame-Id, X-Capture-Monotonic-Ns, X-Capture-Wall-Time-Ns",
        )
        handler.end_headers()
        handler.wfile.write(frame.jpeg)

    @staticmethod
    def _send_json(handler: BaseHTTPRequestHandler, status: HTTPStatus, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        handler.send_response(status)
        handler.send_header("Content-Type", "application/json; charset=utf-8")
        handler.send_header("Content-Length", str(len(body)))
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Access-Control-Allow-Origin", "*")
        handler.end_headers()
        handler.wfile.write(body)
