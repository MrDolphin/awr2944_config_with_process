import importlib.util
import json
import struct
import tempfile
import unittest
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "tools" / "analyze_adc_capture.py"
    spec = importlib.util.spec_from_file_location("analyze_adc_capture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class AnalyzeAdcCaptureTests(unittest.TestCase):
    def setUp(self):
        self.module = load_module()
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.cfg = self.root / "test.cfg"
        self.cfg.write_text(
            "channelCfg 3 1 0 0 0\nprofileCfg 0 77 1 1 10 0 0 70 1 4 1000 0 0 0\nframeCfg 0 0 2 0 4 100 1 0\nlvdsStreamCfg -1 0 1 0\n",
            encoding="utf-8",
        )
        self.bin_path = self.root / "capture.bin"
        self.bin_path.write_bytes(struct.pack("<8h", -32768, -2, 0, 2, 32767, 10, -10, 4))
        self.meta_path = self.root / "capture.json"
        self.meta_path.write_text(json.dumps({"duration_s": 1.0}), encoding="utf-8")

    def tearDown(self):
        self.temp.cleanup()

    def test_report_contains_word_stats_and_candidate_frames(self):
        report = self.module.analyze(self.bin_path, self.cfg, self.meta_path)
        self.assertEqual(report["file"]["bytes"], 16)
        self.assertEqual(report["int16_words"]["count"], 8)
        self.assertEqual(report["int16_words"]["minimum"], -32768)
        self.assertEqual(report["int16_words"]["maximum"], 32767)
        self.assertGreater(report["int16_words"]["saturation_count"], 0)
        self.assertEqual(report["frame_candidates"][0]["bytes_per_frame"], 32)
        self.assertEqual(report["frame_candidates"][0]["full_frames"], 0)
        self.assertEqual(report["format_assessment"]["validated_bytes_per_frame"], 32)

    def test_write_outputs_creates_json_and_markdown_next_to_capture(self):
        report = self.module.analyze(self.bin_path, self.cfg, self.meta_path)
        json_path, markdown_path = self.module.write_outputs(report, self.bin_path.parent)
        self.assertTrue(json_path.is_file())
        self.assertTrue(markdown_path.is_file())
        self.assertIn("初步原始 ADC 采集分析", markdown_path.read_text(encoding="utf-8"))

    def test_word_statistics_does_not_depend_on_a_global_enumerate_binding(self):
        """Only values are needed; an index must not be unpacked or retained."""
        self.module.enumerate = lambda values: values
        try:
            stats = self.module._word_statistics(self.bin_path)
        finally:
            del self.module.enumerate

        self.assertEqual(stats["count"], 8)
        self.assertEqual(stats["minimum"], -32768)
        self.assertEqual(stats["maximum"], 32767)

    def test_word_statistics_streams_a_chunk_larger_than_half_a_million_words(self):
        """Regression for Windows access violations from huge struct.unpack tuples."""
        pattern = struct.pack("<4h", -2, 0, 2, 32767)
        self.bin_path.write_bytes(pattern * (1024 * 1024 // len(pattern) + 1))

        stats = self.module._word_statistics(self.bin_path)

        self.assertGreater(stats["count"], 512000)
        self.assertEqual(stats["minimum"], -2)
        self.assertEqual(stats["maximum"], 32767)
        self.assertGreater(stats["saturation_count"], 0)


if __name__ == "__main__":
    unittest.main()
