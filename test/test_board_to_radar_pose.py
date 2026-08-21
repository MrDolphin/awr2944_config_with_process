import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_59_board_to_radar_pose import run


class BoardToRadarPoseTest(unittest.TestCase):
    def test_vertical_candidate_maps_board_y_to_radar_z_and_keeps_pose_unconfirmed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            regions = root / "regions.csv"
            regions.write_text(
                "antenna,center_x_relative_to_origin_mm,center_y_relative_to_origin_mm\nTX1,10,20\n",
                encoding="utf-8",
            )
            outline = root / "outline.csv"
            outline.write_text(
                "vertex_index,x_relative_to_origin_mm,y_relative_to_origin_mm\n0,0,0\n1,10,20\n",
                encoding="utf-8",
            )
            pose = root / "pose.json"
            pose.write_text(
                json.dumps({"roll_deg": 90, "pitch_deg": 0, "yaw_deg": 0, "translation_mm": [1, 2, 3]}),
                encoding="utf-8",
            )
            output = root / "output"
            summary = run(regions, outline, pose, output)
            self.assertAlmostEqual(summary["board_normal_to_vertical_abs_angle_deg"], 90.0)
            self.assertFalse(summary["installation_pose_confirmed"])
            with (output / "rf_regions_radar_coordinates.csv").open(encoding="utf-8", newline="") as handle:
                row = next(csv.DictReader(handle))
            self.assertAlmostEqual(float(row["radar_x_mm"]), 11.0)
            self.assertAlmostEqual(float(row["radar_y_mm"]), 2.0)
            self.assertAlmostEqual(float(row["radar_z_mm"]), 23.0)
            self.assertTrue((output / "array_baseline_metrics.csv").exists())


if __name__ == "__main__":
    unittest.main()
