import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_62_imu_time_alignment import run


class ImuTimeAlignmentTest(unittest.TestCase):
    def test_interpolates_pose_and_passes_coverage_gap_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            radar = root / "radar.csv"
            radar.write_text("frame,time_s\n0,0.0\n1,0.05\n2,0.1\n", encoding="utf-8")
            imu = root / "imu.csv"
            imu.write_text("time_s,roll_deg,pitch_deg,yaw_deg\n0.0,90,0,0\n0.1,92,2,4\n", encoding="utf-8")
            output = root / "output"
            summary = run(radar, imu, output, max_gap_s=0.2)
            self.assertTrue(summary["quality_pass"])
            with (output / "aligned_imu.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertAlmostEqual(float(rows[1]["roll_deg"]), 91.0)
            self.assertAlmostEqual(float(rows[1]["pitch_deg"]), 1.0)

    def test_out_of_coverage_fails_quality_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            radar = root / "radar.csv"
            radar.write_text("frame,time_s\n0,0.2\n", encoding="utf-8")
            imu = root / "imu.csv"
            imu.write_text("time_s,roll_deg,pitch_deg,yaw_deg\n0.0,90,0,0\n0.1,92,2,4\n", encoding="utf-8")
            summary = run(radar, imu, root / "output", max_gap_s=0.2)
            self.assertFalse(summary["quality_pass"])
            self.assertFalse(summary["coverage_ok"])


if __name__ == "__main__":
    unittest.main()
