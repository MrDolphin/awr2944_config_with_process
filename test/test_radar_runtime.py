import json
import tempfile
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
            for filename in ("../outside.cfg", "nested/profile.cfg", "profile.txt", ""):
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


if __name__ == "__main__":
    unittest.main()
