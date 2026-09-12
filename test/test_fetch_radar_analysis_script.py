import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class FetchRadarAnalysisScriptTests(unittest.TestCase):
    def test_fetch_script_defaults_to_new_run_sync_and_keeps_bin_opt_in(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn("[switch]$IncludeBin", script)
        self.assertIn("[switch]$OpenDashboard", script)
        self.assertIn("Assert-SafeRunId", script)
        self.assertIn("range_analysis", script)
        self.assertIn("Invoke-Checked \"ssh\"", script)
        self.assertIn("Invoke-Checked \"scp\"", script)
        self.assertIn("diagnostic_dashboard.png", script)
        self.assertIn("adc_data_*.json", script)
        self.assertIn("Copy-RemoteFileIfMissing", script)
        self.assertIn("[SKIP] Already exists", script)
        self.assertIn('$RemoteRunsRoot = "/home/pi/radar_runs"', script)
        self.assertIn('$LocalRunsRoot = "D:\\radar_runs"', script)
        self.assertIn("Get-LocalRunPathSet", script)
        self.assertIn(".Path.TrimEnd('\\', '/')", script)
        self.assertIn(".TrimStart('\\', '/')", script)
        self.assertIn(".Replace('\\', '/')", script)
        self.assertNotIn(".TrimEnd('\\\\', '/')", script)
        self.assertIn("$localRunPathSet.ContainsKey($_)", script)
        self.assertIn("[SKIP] Local run already exists", script)
        self.assertIn("[PLAN] New Pi runs to transfer (newest first)", script)
        self.assertNotIn("Select-Object -Last 1", script)

    def test_default_sync_scans_every_capture_family_but_keeps_explicit_root_mode(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn("-mindepth 2 -maxdepth 2", script)
        self.assertIn("-printf '%P\\n'", script)
        self.assertIn("Sort-Object { ($_ -split '/')[-1] } -Descending", script)
        self.assertNotIn("awk -F/", script)
        self.assertIn("Specify both RemoteCaptureRoot and LocalCaptureRoot", script)
        self.assertIn("Copy-OneRun $remoteTarget $selectedRunPath $effectiveRemoteRoot $effectiveLocalRoot", script)

    def test_network_mode_uses_memorable_names_instead_of_ip_parameters(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn('[ValidateSet("lab", "hotspot", "phone")]', script)
        self.assertIn('[string]$NetworkMode = "lab"', script)
        self.assertIn('lab = "172.20.10.10"', script)
        self.assertIn('"10.42.0.1"', script)
        self.assertIn('phone = "192.168.43.36"', script)
        self.assertNotIn("[string]$PiHost", script)
        self.assertNotIn("[switch]$Hotspot", script)
        self.assertIn("$remoteTarget = \"${PiUser}@${effectivePiHost}\"", script)
        self.assertIn("[MODE] ${NetworkMode}", script)

    def test_missing_analysis_directory_is_skipped_without_stopping_batch_sync(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn("if [ -d '$remoteAnalysis' ]; then printf yes; fi", script)
        self.assertIn("[SKIP] No range_analysis directory", script)
        self.assertIn("$fileName = if ($fileProbe)", script)

    def test_partially_downloaded_analysis_directory_is_resumed_per_file(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn("Copy-RemoteTreeMissing", script)
        self.assertIn("-type f -printf '%P\\n' | sort", script)
        self.assertIn("Assert-SafeRelativePath", script)
        self.assertIn("New-Item -ItemType Directory -Force -Path $localParent", script)

    def test_capture_cfg_snapshot_is_downloaded_with_each_run(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn(
            'Copy-RemoteFileIfMissing $Target $remoteRun $localRun "capture_config.cfg"',
            script,
        )

    def test_legacy_run_recovers_cfg_from_pi_project_and_labels_it_recovered(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn(
            '[string]$RemoteProjectRoot = "/home/pi/awr2944_config_with_process_github"',
            script,
        )
        self.assertIn("Copy-LegacyCaptureCfg", script)
        self.assertIn("radar_cfg.cfg_path", script)
        self.assertIn("capture_config_recovered.cfg", script)
        self.assertIn("[WARN] Recovered current Pi CFG for legacy run", script)

    def test_pc_analysis_option_analyzes_only_the_newly_transferred_runs(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn("[switch]$AnalyzeOnPc", script)
        self.assertIn("$IncludeBin -or $AnalyzeOnPc", script)
        self.assertIn("analyze_radar_captures.ps1", script)
        self.assertIn('"-RunFolder", $localRunPath', script)
        self.assertIn("PC analysis failed for $selectedRunPath", script)
        self.assertNotIn('$pcAnalysisArguments += "-Recursive"', script)

    def test_pc_analysis_max_range_is_configurable_from_the_fetch_entrypoint(self):
        script = (ROOT / "tools" / "fetch_radar_analysis.ps1").read_text(encoding="utf-8")
        self.assertIn("[double]$MaxRangeM = 0.0", script)
        self.assertIn('"-MaxRangeM",', script)
        self.assertIn("$MaxRangeM", script)


if __name__ == "__main__":
    unittest.main()
