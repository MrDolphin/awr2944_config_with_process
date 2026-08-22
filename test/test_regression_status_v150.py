import json
import unittest
from pathlib import Path


class RegressionStatusV150Test(unittest.TestCase):
    def test_latest_regression_status_is_current(self):
        root = Path(__file__).parents[1]
        summary = json.loads((root / "simulation/hardware/awr2944pev/v04_150_regression_status/summary.json").read_text(encoding="utf-8"))
        self.assertEqual(summary["tests"], 225)
        self.assertEqual(summary["status"], "full_regression_passed")
        self.assertFalse(summary["hardware_capture_present"])


if __name__ == "__main__":
    unittest.main()
