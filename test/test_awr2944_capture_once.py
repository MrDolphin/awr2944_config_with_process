import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "tools" / "awr2944_capture_once.py"
    spec = importlib.util.spec_from_file_location("awr2944_capture_once", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class OneClickCaptureTests(unittest.TestCase):
    def setUp(self):
        self.module = load_module()
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.cfg = self.root / "valid.cfg"
        self.cfg.write_text("sensorStop\nlvdsStreamCfg -1 0 1 0\nsensorStart\n", encoding="utf-8")

    def tearDown(self):
        self.temp.cleanup()

    def test_validate_cfg_requires_lvds_streaming(self):
        self.assertEqual(self.module.validate_cfg(self.cfg), [])
        no_lvds = self.root / "no_lvds.cfg"
        no_lvds.write_text("sensorStop\nsensorStart\n", encoding="utf-8")
        self.assertEqual(self.module.validate_cfg(no_lvds), ["lvdsStreamCfg is required for DCA1000 raw ADC capture"])

    def test_capture_command_uses_listener_before_radar_start(self):
        args = self.module.parse_args([
            "--cfg", str(self.cfg), "--duration", "60", "--output-dir", str(self.root / "out"),
        ])
        command = self.module.build_capture_command(args)
        self.assertEqual(command[1], "-u")
        self.assertIn("--no-control", command)
        self.assertNotIn("--start-dca", command)
        self.assertIn("--listen-ip", command)
        self.assertIn("192.168.33.30", command)

    def test_cli_commands_use_configure_without_start_then_start(self):
        args = self.module.parse_args(["--cfg", str(self.cfg)])
        configure = self.module.build_cli_configure_command(args)
        start = self.module.build_cli_start_stop_command(args, "start")
        self.assertIn("configure", configure)
        self.assertIn("--no-start", configure)
        self.assertIn("--delay", start)
        self.assertIn(str(args.cli_delay), start)
        self.assertEqual(start[-1], "start")

    def test_start_observe_window_is_forwarded_only_to_sensor_start(self):
        args = self.module.parse_args(
            ["--cfg", str(self.cfg), "--startup-observe-seconds", "8"]
        )
        start = self.module.build_cli_start_stop_command(args, "start")
        stop = self.module.build_cli_start_stop_command(args, "stop")
        self.assertIn("--observe-seconds", start)
        self.assertIn("8.0", start)
        self.assertNotIn("--observe-seconds", stop)

    def test_analysis_is_opt_in_and_uses_capture_directory(self):
        args = self.module.parse_args([
            "--cfg", str(self.cfg), "--analyze-range", "--analysis-max-range-m", "12.0",
        ])
        self.assertTrue(args.analyze_range)
        command = self.module.build_range_analysis_command(args, self.root / "run" / "adc.bin")
        self.assertIn("analyze_adc_range.py", command[1])
        self.assertIn("--remove-mean", command)
        self.assertIn("--max-range-m", command)
        self.assertIn("12.0", command)
        self.assertIn(str(self.root / "run" / "range_analysis"), command)

    def test_post_analysis_implies_range_analysis_and_writes_at_capture_level(self):
        args = self.module.parse_args(["--cfg", str(self.cfg), "--post-analyze"])
        self.assertFalse(args.analyze_range)
        self.assertTrue(args.post_analyze)
        self.assertTrue(self.module.should_run_range_analysis(args))
        command = self.module.build_post_capture_analysis_command(args, self.root / "run" / "adc.bin")
        self.assertIn("post_capture_analysis.py", command[1])
        self.assertIn(str(self.root / "run" / "adc.json"), command)
        self.assertIn(str(self.root / "run" / "range_analysis"), command)
        self.assertIn(str(self.root / "run"), command)

    def test_local_defaults_supply_stable_topology_and_cli_overrides_them(self):
        defaults = self.root / "capture_defaults.json"
        defaults.write_text(
            json.dumps(
                {
                    "cfg": str(self.cfg),
                    "cli_port": "/dev/ttyACM0",
                    "dca_ip": "192.168.33.180",
                    "system_ip": "192.168.33.30",
                    "duration": 20.0,
                    "output_dir": "/home/pi/radar_runs/awr2944p",
                    "post_analyze": True,
                }
            ),
            encoding="utf-8",
        )
        args = self.module.parse_args(["--defaults", str(defaults), "--duration", "7", "--output-dir", "/tmp/run"])
        self.assertEqual(args.cfg, str(self.cfg))
        self.assertEqual(args.dca_ip, "192.168.33.180")
        self.assertEqual(args.duration, 7.0)
        self.assertEqual(args.output_dir, "/tmp/run")
        self.assertTrue(args.post_analyze)

    def test_dca_lvds_mode_defaults_to_two_lane_and_can_request_four_lane(self):
        two_lane = self.module.parse_args(["--cfg", str(self.cfg)])
        four_lane = self.module.parse_args(["--cfg", str(self.cfg), "--dca-lvds-mode", "1"])
        self.assertEqual(two_lane.dca_lvds_mode, 2)
        self.assertEqual(four_lane.dca_lvds_mode, 1)
        self.assertEqual(self.module.effective_config(four_lane)["dca_lvds_mode"], 1)

    def test_unknown_default_setting_is_rejected(self):
        defaults = self.root / "bad_capture_defaults.json"
        defaults.write_text(json.dumps({"cfg": str(self.cfg), "unknown_setting": 1}), encoding="utf-8")
        with self.assertRaises(ValueError):
            self.module.parse_args(["--defaults", str(defaults)])


if __name__ == "__main__":
    unittest.main()
