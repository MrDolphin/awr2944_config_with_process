import json
import unittest
from pathlib import Path


class ProjectStatusV147Test(unittest.TestCase):
    def test_status_snapshot_preserves_hardware_boundary(self):
        root = Path(__file__).parents[1]
        summary = json.loads((root / "simulation/hardware/awr2944pev/v04_147_project_status/summary.json").read_text(encoding="utf-8"))
        report = (root / "simulation/hardware/awr2944pev/v04_147_project_status/output_analysis.md").read_text(encoding="utf-8")
        self.assertEqual(summary["full_regression_tests"], 220)
        self.assertEqual(summary["full_regression_status"], "passed")
        self.assertFalse(summary["hardware_capture_present"])
        self.assertFalse(summary["hardware_aoa_validated"])
        self.assertIn("尚不能宣称真实硬件 AoA", report)


if __name__ == "__main__":
    unittest.main()
