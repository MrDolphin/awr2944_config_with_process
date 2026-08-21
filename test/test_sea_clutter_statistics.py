import csv
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_66_sea_clutter_statistics import run


class SeaClutterStatisticsTest(unittest.TestCase):
    def test_aggregates_case_and_delay_statistics(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            delay = root / "delay_p0_005000s" / "compensated"
            delay.mkdir(parents=True)
            (root / "latency_pointcloud_metrics.csv").write_text("latency_correction_s,quality_pass,max_position_change_m,max_abs_azimuth_change_deg,max_abs_elevation_change_deg,point_count\n0.005,True,0.2,2,1,2\n", encoding="utf-8")
            (delay / "point_cloud_pose_comparison.csv").write_text("case_id,frame,fixed_azimuth_deg,fixed_elevation_deg,compensated_azimuth_deg,compensated_elevation_deg,azimuth_change_deg,elevation_change_deg,position_change_m\nss3_nominal,1,10,2,11,2.5,1,0.5,0.1\nss3_nominal,2,12,3,11,2.5,-1,-0.5,0.2\n", encoding="utf-8")
            output = root / "output"
            summary = run(root, output)
            self.assertEqual(summary["case_count"], 1)
            with (output / "sea_state_delay_statistics.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 1)
            self.assertAlmostEqual(float(rows[0]["mean_position_change_m"]), 0.15)


if __name__ == "__main__":
    unittest.main()
