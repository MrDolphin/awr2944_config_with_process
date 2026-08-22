import tempfile
import unittest
from pathlib import Path


class CaptureEnvironmentPreflightV137Test(unittest.TestCase):
    def test_preflight_is_non_invasive_and_reports_missing_lvds(self):
        from simulation.run_v04_137_capture_environment_preflight import run

        with tempfile.TemporaryDirectory() as directory:
            cfg = Path(directory) / "test.cfg"; cfg.write_text("sensorStart\n", encoding="utf-8")
            result = run(cfg, Path(directory) / "out", Path(directory) / "tools")
        self.assertFalse(result["capture_started"])
        self.assertFalse(result["checks"]["lvds_stream_config_present"])


if __name__ == "__main__":
    unittest.main()
