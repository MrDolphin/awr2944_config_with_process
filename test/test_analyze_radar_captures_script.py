import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "analyze_radar_captures.ps1"


class AnalyzeRadarCapturesScriptTests(unittest.TestCase):
    def test_dry_run_auto_discovers_capture_files_and_exact_cfg(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            capture_root = Path(temp_dir)
            run_dir = capture_root / "20260911_113827"
            run_dir.mkdir()
            (run_dir / "adc_data_20260911_113827.bin").write_bytes(b"adc")
            (run_dir / "adc_data_20260911_113827.json").write_text("{}", encoding="utf-8")
            (run_dir / "capture_config.cfg").write_text("sensorStop\n", encoding="utf-8")

            result = subprocess.run(
                [
                    "powershell",
                    "-NoProfile",
                    "-ExecutionPolicy",
                    "Bypass",
                    "-File",
                    str(SCRIPT),
                    "-CaptureRoot",
                    str(capture_root),
                    "-DryRun",
                ],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("adc_data_20260911_113827.bin", result.stdout)
            self.assertIn("adc_data_20260911_113827.json", result.stdout)
            self.assertIn("capture_config.cfg", result.stdout)
            self.assertIn("pc_analysis", result.stdout)

    def test_dry_run_uses_recovered_cfg_for_legacy_capture(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            capture_root = Path(temp_dir)
            run_dir = capture_root / "20260910_154213"
            run_dir.mkdir()
            (run_dir / "adc_data_20260910_154213.bin").write_bytes(b"adc")
            (run_dir / "adc_data_20260910_154213.json").write_text("{}", encoding="utf-8")
            (run_dir / "capture_config_recovered.cfg").write_text(
                "sensorStop\n", encoding="utf-8"
            )

            result = subprocess.run(
                [
                    "powershell",
                    "-NoProfile",
                    "-ExecutionPolicy",
                    "Bypass",
                    "-File",
                    str(SCRIPT),
                    "-CaptureRoot",
                    str(capture_root),
                    "-DryRun",
                ],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("capture_config_recovered.cfg", result.stdout)


if __name__ == "__main__":
    unittest.main()
