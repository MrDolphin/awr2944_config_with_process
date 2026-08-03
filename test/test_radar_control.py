import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from radar_control import RadarConfigManager


class RadarConfigManagerTests(unittest.TestCase):
    def test_save_list_and_read_only_cfg_profiles(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = RadarConfigManager(Path(directory) / "Config")
            self.assertTrue(manager.save("dock.cfg", "sensorStop\n")["success"])
            self.assertEqual(manager.list_configs()["files"], ["dock.cfg"])
            self.assertEqual(manager.read("dock.cfg")["content"], "sensorStop\n")
            self.assertFalse(manager.save("dock.CFG", "x")["success"])
            self.assertFalse(manager.read("../outside.cfg")["success"])

    def test_apply_uses_runtime_detected_cfg_port_and_returns_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = RadarConfigManager(Path(directory) / "Config")
            calls = []

            def fake_sender(port, path):
                calls.append((port, Path(path)))
                return True

            result = manager.apply("dock.cfg", "sensorStart\n", "/dev/ttyACM7", fake_sender)
            self.assertTrue(result["success"])
            self.assertEqual(calls[0][0], "/dev/ttyACM7")
            self.assertEqual(calls[0][1].name, "dock.cfg")
            self.assertEqual(result["active_config"]["content"], "sensorStart\n")

    def test_apply_returns_failure_without_snapshot_when_radar_rejects(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = RadarConfigManager(Path(directory) / "Config")
            result = manager.apply("dock.cfg", "sensorStart\n", "/dev/ttyACM7", lambda *_: False)
            self.assertEqual(result, {"success": False, "filename": "dock.cfg"})

    def test_apply_rejects_blank_content_and_sender_exceptions(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = RadarConfigManager(Path(directory) / "Config")
            blank_result = manager.apply("dock.cfg", " \n", "/dev/ttyACM7", lambda *_: True)
            self.assertFalse(blank_result["success"])

            result = manager.apply(
                "dock.cfg", "sensorStart\n", "/dev/ttyACM7", lambda *_: (_ for _ in ()).throw(OSError("offline"))
            )
            self.assertFalse(result["success"])
            self.assertIn("offline", result["message"])

    def test_failed_atomic_save_preserves_previous_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = RadarConfigManager(Path(directory) / "Config")
            self.assertTrue(manager.save("dock.cfg", "old\n")["success"])
            with patch("radar_control.os.replace", side_effect=OSError("disk full")):
                result = manager.save("dock.cfg", "new\n")
            self.assertFalse(result["success"])
            self.assertEqual(manager.read("dock.cfg")["content"], "old\n")


if __name__ == "__main__":
    unittest.main()
