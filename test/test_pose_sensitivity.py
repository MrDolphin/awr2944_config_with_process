import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_60_pose_sensitivity import run


class PoseSensitivityTest(unittest.TestCase):
    def test_scan_has_nominal_zero_displacement_and_nonzero_perturbation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            regions = root / "regions.csv"
            regions.write_text(
                "antenna,center_x_relative_to_origin_mm,center_y_relative_to_origin_mm\nTX1,10,20\nTX2,20,20\n",
                encoding="utf-8",
            )
            pose = root / "pose.json"
            pose.write_text(json.dumps({"roll_deg": 90, "pitch_deg": 0, "yaw_deg": 0, "translation_mm": [0, 0, 0]}), encoding="utf-8")
            output = root / "output"
            summary = run(regions, pose, output, (-1.0, 0.0, 1.0), (0.0,), (0.0,))
            self.assertEqual(summary["scenario_count"], 3)
            with (output / "pose_sensitivity.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            nominal = next(row for row in rows if float(row["roll_offset_deg"]) == 0.0)
            self.assertAlmostEqual(float(nominal["max_region_displacement_mm"]), 0.0)
            self.assertGreater(max(float(row["max_region_displacement_mm"]) for row in rows), 0.0)


if __name__ == "__main__":
    unittest.main()
