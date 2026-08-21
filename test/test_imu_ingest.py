import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_63_imu_ingest import run


class ImuIngestTest(unittest.TestCase):
    def test_millisecond_input_and_latency_are_normalized_before_alignment(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            radar = root / "radar.csv"
            radar.write_text("frame,time_s\n0,0.005\n1,0.055\n2,0.105\n", encoding="utf-8")
            imu = root / "imu.csv"
            imu.write_text("timestamp,roll_deg,pitch_deg,yaw_deg\n0,90,0,0\n50,91,1,2\n100,92,2,4\n", encoding="utf-8")
            output = root / "output"
            summary = run(radar, imu, output, timestamp_unit="ms", latency_s=0.005, max_gap_s=0.06)
            self.assertTrue(summary["alignment"]["quality_pass"])
            with (output / "normalized_imu.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertAlmostEqual(float(rows[1]["time_s"]), 0.055)
            self.assertAlmostEqual(float(rows[1]["yaw_deg"]), 2.0)

    def test_unknown_unit_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            radar = root / "radar.csv"; radar.write_text("frame,time_s\n0,0\n", encoding="utf-8")
            imu = root / "imu.csv"; imu.write_text("timestamp,roll_deg,pitch_deg,yaw_deg\n0,90,0,0\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                run(radar, imu, root / "output", timestamp_unit="minute")


if __name__ == "__main__":
    unittest.main()
