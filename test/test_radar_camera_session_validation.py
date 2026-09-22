import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from radar_camera_recording import RadarCameraSessionWriter
from radar_camera_session_validation import validate_session
from tools.camera.camera_capture import CameraFrame


class RadarCameraSessionValidationTests(unittest.TestCase):
    def _write_session(self, root: Path, *, include_camera: bool = True) -> Path:
        session = root / "radar_camera_20260922_120000"
        (session / "camera_frames").mkdir(parents=True)
        (session / "session_metadata.json").write_text(
            json.dumps(
                {
                    "recording_status": "completed",
                    "clock_basis": "pi_receive_monotonic",
                    "git": {"commit": "abc123", "dirty": False},
                    "radar_config": {"name": "dock.cfg", "sha256": "radar-hash"},
                    "camera_config": {"device": "/dev/v4l/by-id/camera", "width": 1280, "height": 720},
                    "sync_thresholds_ms": {"matched": 50, "stale": 100},
                }
            ),
            encoding="utf-8",
        )
        (session / "radar_frames.jsonl").write_text(
            "\n".join(
                [
                    json.dumps({"frame_num": 1, "points": []}),
                    json.dumps({"frame_num": 2, "points": []}),
                    json.dumps({"frame_num": 3, "points": []}),
                    json.dumps({"frame_num": 4, "points": []}),
                ]
            )
            + "\n",
            encoding="utf-8",
        )
        fieldnames = [
            "radar_frame_num",
            "radar_monotonic_ns",
            "camera_frame_id",
            "camera_monotonic_ns",
            "time_offset_ms",
            "sync_status",
            "yaw_deg",
            "pitch_deg",
            "pose_age_ms",
        ]
        rows = [
            {"radar_frame_num": "1", "camera_frame_id": "7", "time_offset_ms": "-10", "sync_status": "matched"},
            {"radar_frame_num": "2", "camera_frame_id": "7", "time_offset_ms": "20", "sync_status": "matched"},
            {"radar_frame_num": "3", "camera_frame_id": "8", "time_offset_ms": "30", "sync_status": "matched"},
            {"radar_frame_num": "4", "camera_frame_id": "", "time_offset_ms": "", "sync_status": "unavailable"},
        ]
        with (session / "fusion_index.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        if include_camera:
            (session / "camera_frames" / "7.jpg").write_bytes(b"jpeg-7")
            (session / "camera_frames" / "8.jpg").write_bytes(b"jpeg-8")
        return session

    def test_valid_session_reports_integrity_ratio_and_p95(self):
        with tempfile.TemporaryDirectory() as directory:
            report = validate_session(self._write_session(Path(directory)))

        self.assertEqual(report["errors"], [])
        self.assertEqual(report["metrics"]["radar_rows"], 4)
        self.assertEqual(report["metrics"]["matched_rows"], 3)
        self.assertEqual(report["metrics"]["matched_ratio"], 0.75)
        self.assertEqual(report["metrics"]["absolute_offset_p95_ms"], 30.0)
        self.assertEqual(report["acceptance"]["matched_ratio"], "fail")
        self.assertEqual(report["acceptance"]["absolute_offset_p95_ms"], "pass")

    def test_missing_referenced_jpeg_is_an_integrity_error(self):
        with tempfile.TemporaryDirectory() as directory:
            report = validate_session(self._write_session(Path(directory), include_camera=False))

        self.assertEqual(len(report["errors"]), 2)
        self.assertIn("camera_frames/7.jpg", report["errors"][0])
        self.assertIn("camera_frames/8.jpg", report["errors"][1])

    def test_corrupt_text_file_is_reported_as_integrity_error(self):
        for filename, error_fragment in (
            ("session_metadata.json", "Cannot read session_metadata.json"),
            ("radar_frames.jsonl", "Cannot read radar_frames.jsonl"),
            ("fusion_index.csv", "Cannot read fusion_index.csv"),
        ):
            with self.subTest(filename=filename), tempfile.TemporaryDirectory() as directory:
                session = self._write_session(Path(directory))
                (session / filename).write_bytes(b"\xff")
                report = validate_session(session)
                self.assertTrue(any(error_fragment in error for error in report["errors"]))

    def test_validates_a_package_written_by_the_session_writer(self):
        metadata = {
            "git": {"commit": "abc123", "dirty": False},
            "radar_config": {"name": "dock.cfg", "sha256": "radar-hash"},
            "camera_config": {"device": "/dev/v4l/by-id/camera", "width": 1280, "height": 720},
            "sync_thresholds_ms": {"matched": 50, "stale": 100},
        }
        message = {
            "frame_num": 1,
            "host_monotonic_s": 2.0,
            "points": [],
            "camera_sync": {
                "status": "matched",
                "frame_id": 3,
                "capture_monotonic_ns": 2_000_000_000,
                "time_offset_ms": -15.0,
            },
        }
        frame = CameraFrame(3, 2_000_000_000, 1_700_000_000_000_000_000, 1280, 720, b"jpeg")
        with tempfile.TemporaryDirectory() as directory:
            writer = RadarCameraSessionWriter(clock=lambda: 1_700_000_000.0, monotonic_ns=lambda: 900)
            session = writer.start(Path(directory), metadata)
            writer.append(message, frame)
            writer.stop()
            report = validate_session(session)

        self.assertEqual(report["errors"], [])
        self.assertEqual(report["acceptance"], {"matched_ratio": "pass", "absolute_offset_p95_ms": "pass"})

    def test_rejects_invalid_acceptance_thresholds(self):
        with tempfile.TemporaryDirectory() as directory:
            session = self._write_session(Path(directory))
            with self.assertRaisesRegex(ValueError, "min_matched_ratio must be between 0 and 1"):
                validate_session(session, min_matched_ratio=1.01)
            with self.assertRaisesRegex(ValueError, "max_absolute_offset_p95_ms must be non-negative"):
                validate_session(session, max_absolute_offset_p95_ms=-0.1)
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(value=value):
                    with self.assertRaisesRegex(ValueError, "max_absolute_offset_p95_ms must be finite"):
                        validate_session(session, max_absolute_offset_p95_ms=value)

    def test_cli_rejects_invalid_threshold_before_reading_a_session(self):
        script = Path(__file__).resolve().parents[1] / "radar_camera_session_validation.py"
        completed = subprocess.run(
            [sys.executable, str(script), "missing-session", "--min-matched-ratio", "-0.1"],
            text=True,
            capture_output=True,
            check=False,
        )

        self.assertEqual(completed.returncode, 2)
        self.assertIn("min_matched_ratio must be between 0 and 1", completed.stderr)

    def test_replay_manifest_selects_ten_spread_matched_frames(self):
        metadata = {
            "git": {"commit": "abc123", "dirty": False},
            "radar_config": {"name": "dock.cfg", "sha256": "radar-hash"},
            "camera_config": {"device": "/dev/v4l/by-id/camera", "width": 1280, "height": 720},
            "sync_thresholds_ms": {"matched": 50, "stale": 100},
        }
        with tempfile.TemporaryDirectory() as directory:
            writer = RadarCameraSessionWriter(clock=lambda: 1_700_000_000.0, monotonic_ns=lambda: 900)
            session = writer.start(Path(directory), metadata)
            for number in range(1, 21):
                timestamp = number * 1_000_000_000
                frame = CameraFrame(number, timestamp, timestamp, 1280, 720, b"jpeg")
                writer.append({
                    "frame_num": number,
                    "host_monotonic_s": number,
                    "points": [],
                    "camera_sync": {
                        "status": "matched", "frame_id": number,
                        "capture_monotonic_ns": timestamp, "time_offset_ms": 0.0,
                    },
                }, frame)
            writer.stop()
            report = validate_session(session)

        self.assertEqual(report["errors"], [])
        samples = report["replay_samples"]
        self.assertEqual(len(samples), 10)
        self.assertEqual(samples[0]["radar_frame_num"], 1)
        self.assertEqual(samples[-1]["radar_frame_num"], 20)
        self.assertEqual(samples[0]["radar_jsonl_line"], 1)
        self.assertEqual(samples[-1]["camera_frame"], "camera_frames/20.jpg")
        self.assertEqual(len({sample["radar_frame_num"] for sample in samples}), 10)

    def test_replay_manifest_rejects_matched_row_without_camera_id(self):
        with tempfile.TemporaryDirectory() as directory:
            session = self._write_session(Path(directory))
            index = session / "fusion_index.csv"
            with index.open(newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                fields = reader.fieldnames
                rows = list(reader)
            rows[-1]["sync_status"] = "matched"
            with index.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
            report = validate_session(session)

        self.assertTrue(any("matched" in error and "camera_frame_id" in error for error in report["errors"]))


if __name__ == "__main__":
    unittest.main()
