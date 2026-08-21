import csv
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_64_latency_scan import run


class LatencyScanTest(unittest.TestCase):
    def test_scan_reports_passing_window_and_pose_difference(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            radar = root / "radar.csv"
            radar.write_text("frame,time_s\n0,0.01\n1,0.05\n2,0.09\n", encoding="utf-8")
            imu = root / "imu.csv"
            imu.write_text("timestamp,roll_deg,pitch_deg,yaw_deg\n0,90,0,0\n50,91,1,2\n100,92,2,4\n", encoding="utf-8")
            summary = run(radar, imu, root / "output", delays_s=(-0.01, 0.0, 0.01), timestamp_unit="ms", max_gap_s=0.06)
            self.assertGreaterEqual(summary["passing_delay_count"], 1)
            with (root / "output" / "latency_scan.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 3)
            self.assertTrue((root / "output" / "latency_pose_difference.csv").exists())


if __name__ == "__main__":
    unittest.main()
