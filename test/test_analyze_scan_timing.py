import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from tools.analyze_scan_timing import analyze_records


class ScanTimingTests(unittest.TestCase):
    def test_cli_writes_summary_without_hardware_access(self):
        with tempfile.TemporaryDirectory() as directory:
            frames = Path(directory) / "frames.jsonl"
            output = Path(directory) / "summary.json"
            frames.write_text(
                '{"frame_num":1,"host_monotonic_s":1.0}\n'
                '{"frame_num":2,"host_monotonic_s":1.1}\n',
                encoding="utf-8",
            )
            subprocess.run(
                [sys.executable, "tools/analyze_scan_timing.py", str(frames),
                 "--output", str(output)],
                cwd=Path(__file__).resolve().parents[1], check=True,
                capture_output=True, text=True,
            )
            self.assertEqual(json.loads(output.read_text(encoding="utf-8"))["frame_count"], 2)

    def test_reports_observed_intervals_and_frame_counter_gaps(self):
        records = [
            {"frame_num": 1, "host_monotonic_s": 10.0},
            {"frame_num": 2, "host_monotonic_s": 10.1},
            {"frame_num": 3, "host_monotonic_s": 10.2},
            {"frame_num": 5, "host_monotonic_s": 10.4},
        ]
        result = analyze_records(records)
        self.assertEqual(result["frame_count"], 4)
        self.assertEqual(result["missing_frame_numbers"], 1)
        self.assertAlmostEqual(result["median_interval_ms"], 100.0)
        self.assertAlmostEqual(result["p95_interval_ms"], 200.0)
        self.assertAlmostEqual(result["effective_received_hz"], 7.5)
        self.assertEqual(result["time_basis"], "host_packet_receive_monotonic")

    def test_requires_monotonic_times_and_sufficient_frames(self):
        with self.assertRaises(ValueError):
            analyze_records([{"frame_num": 1, "host_monotonic_s": 1.0}])
        with self.assertRaises(ValueError):
            analyze_records([
                {"frame_num": 1, "host_monotonic_s": 1.0},
                {"frame_num": 2, "host_monotonic_s": 1.0},
            ])

    def test_counter_wrap_is_not_reported_as_massive_loss(self):
        result = analyze_records([
            {"frame_num": 2**32 - 1, "host_monotonic_s": 1.0},
            {"frame_num": 0, "host_monotonic_s": 1.1},
        ])
        self.assertEqual(result["missing_frame_numbers"], 0)


if __name__ == "__main__":
    unittest.main()
