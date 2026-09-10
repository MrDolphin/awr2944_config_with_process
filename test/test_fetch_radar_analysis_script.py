import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class FetchRadarAnalysisScriptTests(unittest.TestCase):
    def test_fetch_script_keeps_bin_opt_in_and_requires_safe_run_id(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn("[switch]$IncludeBin", script)
        self.assertIn("[switch]$OpenDashboard", script)
        self.assertIn("Assert-SafeRunId", script)
        self.assertIn("range_analysis", script)
        self.assertIn("Invoke-Checked \"ssh\"", script)
        self.assertIn("Invoke-Checked \"scp\"", script)
        self.assertIn("diagnostic_dashboard.png", script)
        self.assertIn("adc_data_*.json", script)


if __name__ == "__main__":
    unittest.main()
