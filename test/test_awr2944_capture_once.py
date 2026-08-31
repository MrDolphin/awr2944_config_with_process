import importlib.util
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
        self.assertEqual(start[-1], "start")


if __name__ == "__main__":
    unittest.main()
