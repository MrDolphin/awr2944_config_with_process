import csv
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_93_known_angle_plan import build_plan
from simulation.run_v04_94_capture_completeness import run


class CaptureCompletenessTest(unittest.TestCase):
    def test_empty_capture_root_reports_zero_coverage(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            plan_path = root / "plan.csv"
            rows = build_plan()[:2]
            with plan_path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
            summary = run(plan_path, root / "captures", root / "output")
            self.assertEqual(summary["planned_count"], 2)
            self.assertEqual(summary["complete_basic_capture_count"], 0)
            self.assertEqual(summary["coverage_fraction"], 0.0)
            self.assertTrue((root / "output" / "capture_completeness.png").is_file())


if __name__ == "__main__":
    unittest.main()
