import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_84_point_cloud_report import run


class PointCloudReportTest(unittest.TestCase):
    def test_report_contract(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            input_dir = root / "input"
            output_dir = root / "output"
            input_dir.mkdir()
            with (input_dir / "detector_point_cloud.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=[
                    "case_id", "detector", "range_m", "velocity_mps", "azimuth_deg",
                    "elevation_deg", "x_m", "y_m", "z_m",
                ])
                writer.writeheader()
                writer.writerow({"case_id": "ss0_flat", "detector": "ca", "range_m": 10,
                                 "velocity_mps": 0.1, "azimuth_deg": 2, "elevation_deg": 1,
                                 "x_m": 9.9, "y_m": 0.3, "z_m": 0.2})
            with (input_dir / "detector_point_cloud_summary.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=["case_id", "detector", "detection_count", "stored_point_count"])
                writer.writeheader()
                writer.writerow({"case_id": "ss0_flat", "detector": "ca", "detection_count": 1, "stored_point_count": 1})

            result = run(input_dir, output_dir)
            self.assertEqual(result["case_count"], 1)
            self.assertEqual(result["point_count"], 1)
            self.assertTrue((output_dir / "output_analysis.md").is_file())
            self.assertTrue((output_dir / "sea_state_detector_statistics.csv").is_file())
            self.assertEqual(len(list((output_dir / "figures").glob("*.png"))), 2)
            self.assertEqual(json.loads((output_dir / "summary.json").read_text(encoding="utf-8"))["status"], "completed_point_cloud_report")


if __name__ == "__main__":
    unittest.main()
