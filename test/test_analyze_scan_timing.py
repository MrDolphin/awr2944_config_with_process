import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from tools.analyze_scan_timing import (
    analyze_records, compare_frame_timebases, frame_period_from_metadata,
)


class ScanTimingTests(unittest.TestCase):
    def test_cli_compares_capture_cfg_without_overwriting_raw_data(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            frames = root / "frames.jsonl"
            metadata = root / "metadata.json"
            frames.write_text(
                '{"frame_num":1,"host_monotonic_s":1.0}\n'
                '{"frame_num":2,"host_monotonic_s":1.01}\n', encoding="utf-8")
            metadata.write_text(json.dumps({"active_config": {"content":
                "frameCfg 0 0 16 0 656 10 1 0"}}), encoding="utf-8")
            completed = subprocess.run(
                [sys.executable, "tools/analyze_scan_timing.py", str(frames),
                 "--metadata", str(metadata), "--rpm", "30"],
                cwd=Path(__file__).resolve().parents[1], check=True,
                capture_output=True, text=True,
            )
            self.assertEqual(json.loads(completed.stdout)["timebase_comparison"]
                             ["estimated_max_angle_step_deg"], 1.8)

    def test_uses_only_active_framecfg_from_capture_metadata(self):
        metadata = {"active_config": {"content": (
            "%frameCfg 0 3 16 0 656 100 1 0\n"
            "frameCfg 0 0 16 0 656 10 1 0\n"
        )}}
        self.assertEqual(frame_period_from_metadata(metadata), 10)
        metadata["active_config"]["content"] += "frameCfg 0 0 16 0 656 15 1 0\n"
        with self.assertRaises(ValueError):
            frame_period_from_metadata(metadata)

    def test_frame_counter_timebase_separates_parse_burst_from_missing_frames(self):
        frames = [
            {"frame_num": 100 + i, "host_monotonic_s": 10 + i * .01 +
             (.025 if i >= 4 else 0)} for i in range(8)
        ]
        result = compare_frame_timebases(frames, frame_period_ms=10, rpm=30)
        self.assertEqual(result["host_gaps_over_5_deg"], 1)
        self.assertEqual(result["estimated_gaps_over_5_deg"], 0)
        self.assertAlmostEqual(result["estimated_max_angle_step_deg"], 1.8)
        self.assertEqual(result["missing_frame_numbers"], 0)
        self.assertFalse(result["rf_time_calibrated"])

    def test_frame_counter_timebase_preserves_real_counter_gaps(self):
        frames = [
            {"frame_num": 100, "host_monotonic_s": 10.0},
            {"frame_num": 101, "host_monotonic_s": 10.01},
            {"frame_num": 104, "host_monotonic_s": 10.04},
        ]
        result = compare_frame_timebases(frames, frame_period_ms=10, rpm=30)
        self.assertEqual(result["missing_frame_numbers"], 2)
        self.assertAlmostEqual(result["estimated_max_angle_step_deg"], 5.4)
        self.assertEqual(result["estimated_gaps_over_5_deg"], 1)

    def test_frame_counter_timebase_rejects_invalid_config_and_backward_counter(self):
        frames = [
            {"frame_num": 2, "host_monotonic_s": 10.0},
            {"frame_num": 1, "host_monotonic_s": 10.01},
        ]
        with self.assertRaises(ValueError):
            compare_frame_timebases(frames, frame_period_ms=10, rpm=30)
        frames[1]["frame_num"] = 3
        with self.assertRaises(ValueError):
            compare_frame_timebases(frames, frame_period_ms=0, rpm=30)

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
        self.assertEqual(result["time_basis"], "host_packet_parse_monotonic")

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
