import csv
import io
import json
import tempfile
import unittest
from pathlib import Path

from tools.camera.camera_capture import CameraFrame

from radar_camera_recording import RadarCameraSessionWriter


def camera_frame(frame_id: int = 9) -> CameraFrame:
    return CameraFrame(
        frame_id=frame_id,
        host_monotonic_ns=2_000_000_000,
        host_wall_time_ns=1_700_000_002_000_000_000,
        width=1280,
        height=720,
        jpeg=b"\xff\xd8camera\xff\xd9",
    )


def radar_message(frame_num: int, camera_id: int | None = 9) -> dict[str, object]:
    return {
        "frame_num": frame_num,
        "host_monotonic_s": 1.98,
        "points": [{"x": 1.0, "y": 2.0}],
        "camera_sync": {
            "status": "matched" if camera_id is not None else "unavailable",
            "frame_id": camera_id,
            "capture_monotonic_ns": 2_000_000_000 if camera_id is not None else None,
            "time_offset_ms": 20.0 if camera_id is not None else None,
        },
    }


class RadarCameraSessionWriterTests(unittest.TestCase):
    def test_writes_metadata_radar_index_and_one_jpeg_per_camera_frame(self):
        metadata = {
            "git": {"commit": "abc123", "dirty": False},
            "radar_config": {"name": "dock.cfg", "sha256": "hash"},
            "camera_config": {"device": "/dev/v4l/by-id/camera", "width": 1280, "height": 720},
            "sync_thresholds_ms": {"matched": 50, "stale": 100},
            "mount_mode": "co_rotating",
        }
        with tempfile.TemporaryDirectory() as directory:
            writer = RadarCameraSessionWriter(clock=lambda: 1_700_000_000.0, monotonic_ns=lambda: 900)
            session_dir = writer.start(Path(directory), metadata)
            writer.append(radar_message(1), camera_frame())
            writer.append(radar_message(2), camera_frame())
            writer.stop()

            self.assertTrue((session_dir / "session_metadata.json").is_file())
            self.assertTrue((session_dir / "radar_frames.jsonl").is_file())
            self.assertTrue((session_dir / "fusion_index.csv").is_file())
            self.assertEqual((session_dir / "camera_frames" / "9.jpg").read_bytes(), b"\xff\xd8camera\xff\xd9")
            self.assertEqual(len(list((session_dir / "camera_frames").glob("*.jpg"))), 1)

            rows = list(csv.DictReader(io.StringIO((session_dir / "fusion_index.csv").read_text(encoding="utf-8"))))
            self.assertEqual([row["radar_frame_num"] for row in rows], ["1", "2"])
            self.assertEqual(rows[0]["camera_frame_id"], "9")
            self.assertEqual(rows[0]["time_offset_ms"], "20.0")
            self.assertEqual(rows[0]["yaw_deg"], "")
            persisted = json.loads((session_dir / "session_metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(persisted["git"]["commit"], "abc123")
            self.assertEqual(persisted["recording_status"], "completed")
            self.assertEqual(persisted["clock_basis"], "pi_receive_monotonic")

    def test_partial_session_closes_with_empty_camera_and_pose_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = RadarCameraSessionWriter(clock=lambda: 1_700_000_000.0, monotonic_ns=lambda: 900)
            session_dir = writer.start(Path(directory), {"git": {"commit": "abc", "dirty": True}})
            writer.append(radar_message(4, None), None, None)
            writer.stop()

            rows = list(csv.DictReader(io.StringIO((session_dir / "fusion_index.csv").read_text(encoding="utf-8"))))
            self.assertEqual(rows[0]["camera_frame_id"], "")
            self.assertEqual(rows[0]["camera_monotonic_ns"], "")
            self.assertEqual(rows[0]["yaw_deg"], "")
            self.assertEqual(rows[0]["pose_age_ms"], "")
            frames = [json.loads(line) for line in (session_dir / "radar_frames.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertEqual(frames[0]["frame_num"], 4)
            persisted = json.loads((session_dir / "session_metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(persisted["recording_status"], "completed")
            self.assertIn("ended_wall_time_s", persisted)


if __name__ == "__main__":
    unittest.main()
