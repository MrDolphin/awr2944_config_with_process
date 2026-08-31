import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np


def load_module():
    path = Path(__file__).resolve().parents[1] / "tools" / "analyze_adc_range.py"
    spec = importlib.util.spec_from_file_location("analyze_adc_range", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class AnalyzeAdcRangeTests(unittest.TestCase):
    def setUp(self):
        self.module = load_module()
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.cfg = self.root / "test.cfg"
        self.cfg.write_text(
            "channelCfg 3 1 0 0 0\n"
            "profileCfg 0 77 1 1 10 0 0 70 1 8 1000 0 0 0\n"
            "chirpCfg 0 0 0 0 0 0 0 1\n"
            "frameCfg 0 0 2 0 8 100 1 0\n"
            "lvdsStreamCfg -1 0 1 0\n",
            encoding="utf-8",
        )

    def tearDown(self):
        self.temp.cleanup()

    def test_reshape_uses_frame_chirp_rx_sample_order(self):
        raw = np.arange(3 * 2 * 2 * 8, dtype=np.int16)
        path = self.root / "capture.bin"
        raw.tofile(path)
        cube, trailing, cfg = self.module.load_cube(path, self.cfg)
        self.assertEqual(cube.shape, (3, 2, 2, 8))
        self.assertEqual(int(cube[1, 0, 0, 0]), 32)
        self.assertEqual(trailing, 0)
        self.assertEqual(cfg["estimated_payload_bytes_per_frame"], 64)

    def test_range_axis_and_peak_estimation(self):
        samples = 8
        tone = (2000 * np.cos(2 * np.pi * 2 * np.arange(samples) / samples)).astype(np.int16)
        cube = np.tile(tone, (2, 2, 2, 1))
        range_m = self.module.range_axis_m(samples, 1000.0, 70.0)
        power = self.module.range_power(cube)
        self.assertEqual(len(range_m), samples // 2 + 1)
        self.assertEqual(int(np.argmax(power.mean(axis=(0, 1, 2)))), 2)

    def test_markdown_report_contains_evidence_boundary(self):
        report = self.module.markdown_report(
            {
                "full_frames": 3,
                "trailing_bytes_ignored": 0,
                "cube_shape": [3, 2, 2, 8],
                "range_bin_spacing_m": 0.1,
                "max_range_plotted_m": 1.0,
                "remove_time_domain_mean": False,
                "strongest_mean_range_peak": {"range_bin": 2, "range_m": 0.2},
                "candidate_static_peaks": [{"range_m": 0.5, "relative_power_db": -3.0, "temporal_std_db": 0.2}],
            },
            {"num_adc_samples": 8, "num_rx": 2, "num_chirps_per_frame": 2, "frame_period_ms": 100.0},
        )
        self.assertIn("RX/TX 虚拟阵列重排", report)
        self.assertIn("0.200", report)

    def test_candidate_peaks_skip_direct_leakage_guard_range(self):
        ranges = np.arange(10, dtype=float) * 0.1
        mean_power = np.array([100, 10, 20, 8, 5, 30, 4, 3, 2, 1], dtype=float)
        range_time = np.tile(mean_power, (4, 1))
        peaks = self.module.candidate_static_peaks(ranges, mean_power, range_time, minimum_range_m=0.3)
        self.assertEqual(peaks[0]["range_bin"], 5)


if __name__ == "__main__":
    unittest.main()
