import importlib.util
import json
import struct
import tempfile
import unittest
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "tools" / "post_capture_analysis.py"
    spec = importlib.util.spec_from_file_location("post_capture_analysis", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PostCaptureAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.module = load_module()
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.cfg = self.root / "test.cfg"
        self.cfg.write_text(
            "channelCfg 3 1 0 0 0\n"
            "profileCfg 0 77 1 1 10 0 0 70 1 4 1000 0 0 0\n"
            "frameCfg 0 0 2 0 4 100 1 0\n"
            "lvdsStreamCfg -1 0 1 0\n",
            encoding="utf-8",
        )
        self.bin_path = self.root / "capture.bin"
        self.bin_path.write_bytes(struct.pack("<16h", *range(16)))
        self.metadata = self.root / "capture.json"
        self.metadata.write_text(
            json.dumps({"duration_s": 2.0, "packet_count": 12, "dropped_packets_estimate": 0, "total_saved_bytes": 32}),
            encoding="utf-8",
        )
        self.range_dir = self.root / "range_analysis"
        self.range_dir.mkdir()
        (self.range_dir / "range_fft_analysis.json").write_text(
            json.dumps(
                {
                    "full_frames": 2,
                    "trailing_bytes_ignored": 0,
                    "strongest_mean_range_peak": {"range_bin": 1, "range_m": 0.535, "relative_power_db": 0.0},
                    "candidate_static_peaks": [{"range_bin": 2, "range_m": 1.07, "relative_power_db": -4.0, "temporal_std_db": 0.1}],
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self):
        self.temp.cleanup()

    def test_report_separates_verified_capture_from_unrun_calibration_gates(self):
        report = self.module.analyze(self.bin_path, self.cfg, self.metadata, self.range_dir)
        self.assertEqual(report["gates"]["capture_integrity"]["status"], "PASS")
        self.assertEqual(report["gates"]["range_domain"]["status"], "PASS")
        self.assertEqual(report["gates"]["absolute_range_calibration"]["status"], "NOT_RUN")
        self.assertEqual(report["gates"]["rx_tx_phase_calibration"]["status"], "NOT_RUN")
        self.assertEqual(report["gates"]["range_doppler"]["status"], "BLOCKED")
        self.assertEqual(report["gates"]["aoa_point_cloud_accuracy"]["status"], "BLOCKED")

    def test_write_outputs_saves_capture_level_report_and_marks_packet_loss(self):
        self.metadata.write_text(json.dumps({"packet_count": 12, "dropped_packets_estimate": 2}), encoding="utf-8")
        report = self.module.analyze(self.bin_path, self.cfg, self.metadata, self.range_dir)
        json_path, markdown_path = self.module.write_outputs(report, self.root)
        self.assertEqual(report["gates"]["capture_integrity"]["status"], "FAIL")
        self.assertTrue(json_path.is_file())
        self.assertTrue(markdown_path.is_file())
        text = markdown_path.read_text(encoding="utf-8")
        self.assertIn("绝对距离标定", text)
        self.assertIn("未执行", text)


if __name__ == "__main__":
    unittest.main()
