import json
import tempfile
import time
import unittest
from pathlib import Path

from radar_runtime import ConfigPathError, PointCloudRecorder, config_snapshot, resolve_config_path


class ConfigPathTests(unittest.TestCase):
    def test_resolves_only_simple_cfg_name_under_root(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "Config"
            path = resolve_config_path(root, "docking.cfg")
            self.assertEqual(path, root.resolve() / "docking.cfg")

    def test_rejects_path_traversal_and_non_cfg_name(self):
        with tempfile.TemporaryDirectory() as directory:
            for filename in ("../outside.cfg", "nested/profile.cfg", "profile.txt", "profile.CFG", ""):
                with self.assertRaises(ConfigPathError):
                    resolve_config_path(directory, filename)

    def test_config_snapshot_hashes_actual_content(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.cfg"
            path.write_text("sensorStop\n", encoding="utf-8")
            snapshot = config_snapshot(path)
            self.assertEqual(snapshot["name"], "profile.cfg")
            self.assertEqual(len(snapshot["sha256"]), 64)
            self.assertEqual(snapshot["content"], "sensorStop\n")


class RecorderTests(unittest.TestCase):
    def test_capture_metadata_records_requested_registration_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = PointCloudRecorder(directory, clock=lambda: 1_700_000_000.0)
            capture_dir = Path(recorder.start(registration={
                "enabled": True, "direction": 1, "zero_deg": 0.0,
                "mounting_yaw_deg": 0.0, "mounting_roll_deg": 90.0,
                "source": "recording_request", "motor_active": False,
            }))
            recorder.stop()
            metadata = json.loads((capture_dir / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["registration"]["mounting_roll_deg"], 90.0)
            self.assertFalse(metadata["registration"]["motor_active"])

    def test_records_motor_samples_and_registration_events_in_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = PointCloudRecorder(directory, clock=lambda: 1_700_000_000.0)
            capture_dir = Path(recorder.start())
            try:
                self.assertTrue(recorder.record_motor_sample({
                    "query_started_monotonic_s": 10.0,
                    "query_completed_monotonic_s": 10.012,
                    "angle_deg": 359.0,
                    "speed_rpm": 30.0,
                    "enabled": True,
                }))
                self.assertTrue(recorder.record_registration_event({
                    "frame_num": 17,
                    "host_monotonic_s": 10.025,
                    "status": "registered",
                    "yaw_deg": 12.5,
                    "rf_time_calibrated": False,
                    "pose_compensated": False,
                }))
            finally:
                status = recorder.stop()
            samples = [json.loads(line) for line in
                       (capture_dir / "motor_samples.jsonl").read_text(encoding="utf-8").splitlines()]
            events = [json.loads(line) for line in
                      (capture_dir / "live_registration.jsonl").read_text(encoding="utf-8").splitlines()]
            metadata = json.loads((capture_dir / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(samples[0]["angle_deg"], 359.0)
            self.assertEqual(samples[0]["time_basis"], "host_monotonic_query_bracket")
            self.assertEqual(events[0]["frame_num"], 17)
            self.assertEqual(events[0]["status"], "registered")
            self.assertEqual(status["motor_samples"], 1)
            self.assertEqual(status["registration_events"], 1)
            self.assertEqual(metadata["schema_version"], 4)
            self.assertEqual(metadata["files"]["motor_samples"], "motor_samples.jsonl")
            self.assertEqual(metadata["files"]["live_registration"], "live_registration.jsonl")

    def test_rejects_invalid_motor_and_registration_times(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = PointCloudRecorder(directory)
            capture_dir = Path(recorder.start())
            try:
                with self.assertRaises(ValueError):
                    recorder.record_motor_sample({
                        "query_started_monotonic_s": 2.0,
                        "query_completed_monotonic_s": 1.0,
                        "angle_deg": 20.0,
                    })
                with self.assertRaises(ValueError):
                    recorder.record_registration_event({
                        "frame_num": 1, "host_monotonic_s": float("nan"), "status": "registered"
                    })
            finally:
                recorder.stop()
            self.assertEqual((capture_dir / "motor_samples.jsonl").read_text(encoding="utf-8"), "")
            self.assertEqual((capture_dir / "live_registration.jsonl").read_text(encoding="utf-8"), "")

    def test_records_empty_and_populated_frames_with_raw_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = PointCloudRecorder(directory, clock=lambda: 1_700_000_000.0, monotonic=lambda: 99.0)
            capture_dir = Path(recorder.start({"name": "dock.cfg", "sha256": "abc"}))
            self.assertTrue(recorder.record_frame({"frame_num": 7, "points": []}, b"raw-empty"))
            self.assertTrue(recorder.record_frame({
                "frame_num": 8,
                "host_time_s": 12.5,
                "points": [{"x": 1.0, "y": 2.0, "z": 0.0, "v": -0.2, "snr": 12.0, "noise": 8.0}],
            }, b"raw-point"))
            self.assertFalse(recorder.record_frame({"frame_num": 8, "points": []}))
            status = recorder.stop()

            self.assertEqual(status["frames"], 2)
            self.assertEqual(status["points"], 1)
            self.assertEqual((capture_dir / "frames.tlv").read_bytes(), b"raw-emptyraw-point")
            records = [json.loads(line) for line in (capture_dir / "frames.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertEqual([record["point_count"] for record in records], [0, 1])
            self.assertEqual([record["raw_length"] for record in records], [9, 9])
            metadata = json.loads((capture_dir / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["active_config"]["name"], "dock.cfg")
            self.assertEqual(metadata["recording_policy"]["sync_every_frames"], 1)
            self.assertEqual(metadata["recording_status"], "completed")
            self.assertEqual(metadata["frames"], 2)

    def test_writer_error_stops_accepting_frames_without_blocking_stop(self):
        class BrokenRecorder(PointCloudRecorder):
            def _write_frame(self, frame, raw_packet):
                raise OSError("disk unavailable")

        with tempfile.TemporaryDirectory() as directory:
            recorder = BrokenRecorder(directory)
            recorder.start()
            self.assertTrue(recorder.record_frame({"frame_num": 1, "points": []}))
            deadline = time.monotonic() + 1
            while recorder.status()["writer_error"] is None and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(recorder.status()["writer_error"], "OSError: disk unavailable")
            self.assertFalse(recorder.record_frame({"frame_num": 2, "points": []}))
            self.assertEqual(recorder.stop()["frames"], 0)


if __name__ == "__main__":
    unittest.main()
