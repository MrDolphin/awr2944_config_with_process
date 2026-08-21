import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_61_imu_pose_compensation import run


class ImuPoseCompensationTest(unittest.TestCase):
    def test_time_varying_pose_changes_projection_and_preserves_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            point_cloud = root / "points.csv"
            point_cloud.write_text(
                "case_id,frame,x_m,y_m,z_m\nss3,0,0,10,1\nss3,1,1,10,1\n",
                encoding="utf-8",
            )
            imu = root / "imu.csv"
            imu.write_text(
                "frame,time_s,roll_deg,pitch_deg,yaw_deg\n0,0,90,0,0\n1,0.1,91,0,0\n",
                encoding="utf-8",
            )
            pose = root / "pose.json"
            pose.write_text(json.dumps({"roll_deg": 90, "pitch_deg": 0, "yaw_deg": 0}), encoding="utf-8")
            output = root / "output"
            summary = run(point_cloud, imu, output, pose)
            self.assertEqual(summary["point_count"], 2)
            self.assertGreater(summary["max_position_change_m"], 0.0)
            with (output / "point_cloud_pose_comparison.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertAlmostEqual(float(rows[0]["position_change_m"]), 0.0, places=8)


if __name__ == "__main__":
    unittest.main()
