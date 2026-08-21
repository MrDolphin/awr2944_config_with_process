import csv
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_sea_state_interpretation import run


class SeaStateInterpretationTests(unittest.TestCase):
    def test_flags_relative_increases_without_classifying_sea_state(self):
        fields = ["label", "point_count", "velocity_std_mps", "azimuth_std_deg", "elevation_std_deg"]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); stats = root / "stats.csv"
            with stats.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader()
                writer.writerow({"label": "ss0", "point_count": 10, "velocity_std_mps": 1, "azimuth_std_deg": 1, "elevation_std_deg": 1})
                writer.writerow({"label": "ss3", "point_count": 20, "velocity_std_mps": 2, "azimuth_std_deg": 1, "elevation_std_deg": 1})
            report = run(stats, root / "out")
            text = report.read_text(encoding="utf-8")
            self.assertIn("more_cfar_detections_than_baseline", text)
            self.assertIn("wider_velocity_spread_than_baseline", text)
            self.assertTrue((root / "out" / "interpretation.json").exists())


if __name__ == "__main__":
    unittest.main()
