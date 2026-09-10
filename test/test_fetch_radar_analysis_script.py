import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class FetchRadarAnalysisScriptTests(unittest.TestCase):
    def test_fetch_script_supports_missing_run_sync_and_keeps_bin_opt_in(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn("[switch]$Hotspot", script)
        self.assertIn("[switch]$SyncMissing", script)
        self.assertIn("[switch]$IncludeBin", script)
        self.assertIn("[switch]$OpenDashboard", script)
        self.assertIn("Assert-SafeRunId", script)
        self.assertIn("range_analysis", script)
        self.assertIn("Invoke-Checked \"ssh\"", script)
        self.assertIn("Invoke-Checked \"scp\"", script)
        self.assertIn("diagnostic_dashboard.png", script)
        self.assertIn("adc_data_*.json", script)
        self.assertIn("No Pi capture directories are missing", script)
        self.assertIn("Join-Path $LocalCaptureRoot $_", script)

    def test_hotspot_mode_uses_the_pi_hotspot_address_for_ssh_and_scp(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn('"10.42.0.1"', script)
        self.assertIn("$effectivePiHost = if ($Hotspot)", script)
        self.assertIn("$remoteTarget = \"${PiUser}@${effectivePiHost}\"", script)
        self.assertIn("[MODE] Hotspot", script)

    def test_missing_analysis_directory_is_skipped_without_stopping_batch_sync(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn("if [ -d '$remoteAnalysis' ]; then printf yes; fi", script)
        self.assertIn("[SKIP] No range_analysis directory", script)
        self.assertIn("$fileName = if ($fileProbe)", script)


if __name__ == "__main__":
    unittest.main()
