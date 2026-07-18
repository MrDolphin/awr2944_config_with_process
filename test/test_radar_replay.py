import tempfile
import unittest
from pathlib import Path

from radar_replay import CaptureAccessError, CaptureCatalog
from radar_runtime import PointCloudRecorder


class CaptureCatalogTests(unittest.TestCase):
    def setUp(self):
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self._temporary_directory.name)
        recorder = PointCloudRecorder(self.root, clock=lambda: 1_700_000_000.0)
        self.capture_id = Path(recorder.start({"name": "dock.cfg"})).name
        recorder.record_frame({"frame_num": 1, "points": []}, b"empty")
        recorder.record_frame({
            "frame_num": 2,
            "points": [{"x": 1.0, "y": 2.0, "z": 0.5, "v": -0.3, "snr": 9.0, "noise": 4.0}],
        }, b"point")
        recorder.record_frame({
            "frame_num": 1,
            "points": [{"x": 3.0, "y": 4.0, "z": 0.5, "v": 0.1}],
        }, b"reset")
        recorder.stop()
        self.catalog = CaptureCatalog(self.root)

    def tearDown(self):
        self._temporary_directory.cleanup()

    def test_lists_capture_with_metadata_and_frame_count(self):
        captures = self.catalog.list_captures()
        self.assertEqual(captures[0]["id"], self.capture_id)
        self.assertEqual(captures[0]["frames"], 3)
        self.assertEqual(captures[0]["active_config"]["name"], "dock.cfg")
        self.assertNotIn("content", captures[0]["active_config"])
        self.assertEqual(captures[0]["recording_status"], "completed")

    def test_reads_empty_and_populated_frames_for_display(self):
        empty = self.catalog.get_frame(self.capture_id, 0)
        populated = self.catalog.get_frame(self.capture_id, 1)
        reset_frame = self.catalog.get_frame(self.capture_id, 2)
        self.assertEqual(empty["points"], [])
        self.assertEqual(populated["point_count"], 1)
        self.assertEqual(populated["points"][0]["x"], 1.0)
        self.assertEqual(populated["raw_length"], 5)
        self.assertEqual(reset_frame["frame_num"], 1)
        self.assertEqual(reset_frame["points"][0]["x"], 3.0)

    def test_rejects_path_traversal_and_missing_frame(self):
        with self.assertRaises(CaptureAccessError):
            self.catalog.get_frame("../outside", 1)
        with self.assertRaises(CaptureAccessError):
            self.catalog.get_frame(self.capture_id, 99)

    def test_frame_summary_pagination_has_safe_limit(self):
        summaries = list(self.catalog.iter_frame_summaries(self.capture_id, offset=1, limit=999))
        self.assertEqual([frame["record_index"] for frame in summaries], [1, 2])
        with self.assertRaises(CaptureAccessError):
            list(self.catalog.iter_frame_summaries(self.capture_id, limit="many"))


if __name__ == "__main__":
    unittest.main()
