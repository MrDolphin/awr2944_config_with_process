import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_36_pose_transform import run


class PoseTransformTest(unittest.TestCase):
    def test_candidate_pose_transform_records_vertical_angle_without_confirming_installation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            regions = root / "regions.csv"
            regions.write_text("antenna,center_x_relative_to_origin_mm,center_y_relative_to_origin_mm\nTX1,10,20\n", encoding="utf-8")
            output = root / "output"
            summary = run(regions, output, (1.0, 2.0, 3.0))
            self.assertEqual(summary["pose_count"], 6)
            self.assertFalse(summary["installation_pose_confirmed"])
            with (output / "pose_candidates.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            identity = next(row for row in rows if row["pose_name"] == "identity")
            self.assertAlmostEqual(float(identity["normal_to_vertical_abs_angle_deg"]), 0.0)
            with (output / "rf_regions_pose_candidates.csv").open(encoding="utf-8", newline="") as handle:
                transformed = list(csv.DictReader(handle))
            self.assertEqual(len(transformed), 6)
            schema = json.loads((output / "pose_transform_schema.json").read_text(encoding="utf-8"))
            self.assertTrue(schema["pose_measurement_required"])


if __name__ == "__main__":
    unittest.main()
