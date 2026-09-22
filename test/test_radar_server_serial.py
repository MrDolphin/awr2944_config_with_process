import importlib
import struct
import sys
import threading
import types
import unittest
from unittest.mock import patch

from tools.camera.camera_capture import CameraFrame
from sensor_pose import SensorPoseHistory
from tools.fusion.calibration import RadarCameraCalibration


class RadarSerialIntegrationTests(unittest.TestCase):
    def test_empty_packet_is_forwarded_with_header_time_and_raw_bytes(self):
        stop_event = threading.Event()
        packet = struct.pack(
            "<8sIIIIIIII",
            b"\x02\x01\x04\x03\x06\x05\x08\x07",
            0, 40, 0, 42, 1234, 0, 0, 0,
        )

        class FakeSerial:
            def __init__(self, *args, **kwargs):
                self.buffer = bytearray(packet)

            @property
            def in_waiting(self):
                return len(self.buffer)

            def read(self, _count):
                data = bytes(self.buffer)
                self.buffer.clear()
                stop_event.set()
                return data

            def reset_input_buffer(self):
                pass

        previous_serial = sys.modules.get("serial")
        previous_websockets = sys.modules.get("websockets")
        sys.modules["serial"] = types.SimpleNamespace(Serial=FakeSerial)
        sys.modules["websockets"] = types.SimpleNamespace(
            exceptions=types.SimpleNamespace(ConnectionClosed=RuntimeError)
        )
        sys.modules.pop("radar_server", None)
        try:
            server = importlib.import_module("radar_server")
            captured = []
            server.record_pointcloud_frame = lambda frame, raw_packet: captured.append((frame, raw_packet))
            previous_disabled = server.logger.disabled
            server.logger.disabled = True
            try:
                server.radar_serial_thread("fake", 3125000, stop_event=stop_event)
            finally:
                server.logger.disabled = previous_disabled
        finally:
            sys.modules.pop("radar_server", None)
            if previous_serial is None:
                sys.modules.pop("serial", None)
            else:
                sys.modules["serial"] = previous_serial
            if previous_websockets is None:
                sys.modules.pop("websockets", None)
            else:
                sys.modules["websockets"] = previous_websockets

        self.assertEqual(len(captured), 1)
        frame, raw_packet = captured[0]
        self.assertEqual(raw_packet, packet)
        self.assertEqual(frame["frame_num"], 42)
        self.assertEqual(frame["device_time_cpu_cycles"], 1234)
        self.assertEqual(frame["detected_object_count"], 0)
        self.assertEqual(frame["points"], [])


class CameraServiceLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.server = importlib.import_module("radar_server")
        self.server.stop_camera_services()
        self.server.camera_services.update({"public_base_url": "", "last_error": ""})

    def tearDown(self):
        self.server.stop_camera_services()

    def test_measured_pose_metadata_uses_receive_time_and_stale_threshold(self):
        self.server.sensor_pose_history = SensorPoseHistory()
        self.server.sensor_pose_history.append(
            self.server.SensorPose(1_000_000_000, 12.3, -1.4, 0.0, "encoder")
        )

        fresh = self.server.sensor_pose_metadata(1.040)
        stale = self.server.sensor_pose_metadata(1.051)

        self.assertEqual(fresh["status"], "fresh")
        self.assertEqual(fresh["yaw_deg"], 12.3)
        self.assertEqual(fresh["source"], "encoder")
        self.assertEqual(stale["status"], "stale")

    def test_projection_requires_matched_camera_and_fresh_pose_then_clips_points(self):
        self.server.active_camera_calibration = RadarCameraCalibration(
            1280, 720, 800.0, 800.0, 640.0, 360.0, (0, 0, 0, 0, 0),
            ((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0, 0, 0), "fixed_camera", 1.0,
        )
        self.server.active_calibration_id = "sha256:test"
        frame = {"points": [{"x": 0, "y": 0, "z": 5, "snr": 12}, {"x": 100, "y": 0, "z": 1}],
                 "camera_sync": {"status": "matched"},
                 "sensor_pose": {"status": "fresh", "yaw_deg": 0, "pitch_deg": 0, "roll_deg": 0, "pose_age_ms": 10}}
        valid = self.server.camera_projection_metadata(frame)
        self.assertEqual(valid["status"], "valid")
        self.assertEqual(valid["calibration_id"], "sha256:test")
        self.assertEqual(len(valid["points"]), 1)
        frame["sensor_pose"]["pose_age_ms"] = 101
        self.assertEqual(self.server.camera_projection_metadata(frame)["status"], "suppressed")
        frame["sensor_pose"]["pose_age_ms"] = 10; frame["camera_sync"]["status"] = "stale"
        self.assertEqual(self.server.camera_projection_metadata(frame)["reason"], "camera_sync_not_matched")

    def test_start_owns_runtime_and_http_server_then_stop_is_idempotent(self):
        events = []

        class FakeRuntime:
            def __init__(self, config, buffer):
                self.buffer = buffer

            def start(self):
                events.append("runtime.start")

            def stop(self):
                events.append("runtime.stop")

        class FakeHttpServer:
            def __init__(self, host, port, runtime, buffer):
                self.host = host
                self.port = port

            def start(self):
                events.append("http.start")

            def stop(self):
                events.append("http.stop")

        with (
            patch.object(self.server, "load_camera_config", return_value=object()),
            patch.object(self.server, "CameraRuntime", FakeRuntime),
            patch.object(self.server, "CameraHttpServer", FakeHttpServer),
        ):
            self.server.start_camera_services("camera.cfg", "127.0.0.1", 8081, "http://pi:8081")
            self.assertTrue(self.server.camera_services["enabled"])
            self.assertEqual(self.server.camera_services["public_base_url"], "http://pi:8081")
            self.server.stop_camera_services()
            self.server.stop_camera_services()

        self.assertEqual(events, ["runtime.start", "http.start", "http.stop", "runtime.stop"])

    def test_start_failure_releases_runtime_and_leaves_camera_unavailable(self):
        events = []

        class FakeRuntime:
            def __init__(self, config, buffer):
                pass

            def start(self):
                events.append("runtime.start")

            def stop(self):
                events.append("runtime.stop")

        class FailingHttpServer:
            def __init__(self, host, port, runtime, buffer):
                pass

            def start(self):
                events.append("http.start")
                raise RuntimeError("bind failed")

            def stop(self):
                events.append("http.stop")

        with (
            patch.object(self.server, "load_camera_config", return_value=object()),
            patch.object(self.server, "CameraRuntime", FakeRuntime),
            patch.object(self.server, "CameraHttpServer", FailingHttpServer),
        ):
            self.server.start_camera_services("camera.cfg", "127.0.0.1", 8081, "http://pi:8081")

        self.assertFalse(self.server.camera_services["enabled"])
        self.assertIsNone(self.server.camera_services["runtime"])
        self.assertEqual(events, ["runtime.start", "http.start", "runtime.stop"])
        self.assertEqual(self.server.camera_services["last_error"], "bind failed")

    def test_camera_enable_requires_browser_reachable_base_url(self):
        with patch.object(self.server, "load_camera_config") as load_config:
            self.server.start_camera_services("camera.cfg", "0.0.0.0", 8081, "")

        load_config.assert_not_called()
        self.assertFalse(self.server.camera_services["enabled"])
        self.assertIn("public base URL", self.server.camera_services["last_error"])

    def test_metadata_is_unavailable_until_camera_is_enabled(self):
        metadata = self.server.camera_sync_metadata(1.0)
        self.assertEqual(metadata["status"], "unavailable")
        self.assertEqual(metadata["clock_basis"], "pi_receive_monotonic")
        self.assertIsNone(metadata["frame_url"])

    def test_metadata_matches_camera_frame_with_receive_time_clock(self):
        buffer = self.server.CameraFrameBuffer()
        buffer.append(CameraFrame(3, 1_020_000_000, 1_020_000_001, 1280, 720, b"jpeg"))
        self.server.camera_services.update({
            "enabled": True,
            "buffer": buffer,
            "public_base_url": "http://pi:8081",
        })

        metadata = self.server.camera_sync_metadata(1.0)

        self.assertEqual(metadata["status"], "matched")
        self.assertEqual(metadata["frame_id"], 3)
        self.assertEqual(metadata["time_offset_ms"], 20.0)
        self.assertEqual(metadata["frame_url"], "http://pi:8081/camera/frame/3.jpg")
        self.assertEqual(metadata["clock_basis"], "pi_receive_monotonic")

    def test_keyboard_interrupt_cleanup_stops_camera_services(self):
        events = []

        class FakeRuntime:
            def stop(self):
                events.append("runtime.stop")

        class FakeHttpServer:
            def stop(self):
                events.append("http.stop")

        self.server.camera_services.update({
            "enabled": True,
            "runtime": FakeRuntime(),
            "http": FakeHttpServer(),
        })

        with (
            patch.object(self.server, "main_ws_server", new=lambda _port: object()),
            patch.object(self.server.asyncio, "run", side_effect=KeyboardInterrupt),
        ):
            self.server.run_websocket_server(8765)

        self.assertEqual(events, ["http.stop", "runtime.stop"])
        self.assertFalse(self.server.camera_services["enabled"])


if __name__ == "__main__":
    unittest.main()
