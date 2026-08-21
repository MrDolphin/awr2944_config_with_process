import csv
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_65_latency_pointcloud_propagation import run


class LatencyPointCloudPropagationTest(unittest.TestCase):
    def test_passing_latency_reaches_point_cloud_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = root / "points.csv"
            points.write_text("case_id,frame,x_m,y_m,z_m\nss3,25,1,8,1\nss3,29,2,8,1\n", encoding="utf-8")
            radar = root / "radar.csv"
            radar.write_text("frame,time_s\n25,0.01\n29,0.09\n", encoding="utf-8")
            imu = root / "imu.csv"
            imu.write_text("timestamp,roll_deg,pitch_deg,yaw_deg\n0,90,0,0\n50,91,1,2\n100,92,2,4\n", encoding="utf-8")
            output = root / "output"
            summary = run(points, radar, imu, output, delays_s=(0.0,), timestamp_unit="ms")
            self.assertEqual(summary["passing_delay_count"], 1)
            with (output / "latency_pointcloud_metrics.csv").open(encoding="utf-8", newline="") as handle:
                row = next(csv.DictReader(handle))
            self.assertEqual(row["quality_pass"], "True")
            self.assertGreater(float(row["max_abs_azimuth_change_deg"]), 0.0)


if __name__ == "__main__":
    unittest.main()
