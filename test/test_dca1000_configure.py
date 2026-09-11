import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


class Dca1000ConfigureDefaultsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        script = Path(__file__).resolve().parents[1] / "tools" / "dca1000_configure.py"
        spec = importlib.util.spec_from_file_location("dca1000_configure_under_test", script)
        assert spec and spec.loader
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_shared_capture_defaults_are_used_and_cli_can_override_lane_mode(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            defaults_path = Path(temporary_directory) / "capture_defaults.json"
            defaults_path.write_text(
                json.dumps(
                    {
                        "dca_ip": "192.168.33.180",
                        "system_ip": "192.168.33.30",
                        "dca_mac": "12.34.56.78.90.12",
                        "config_port": 4096,
                        "packet_delay_us": 25,
                        "dca_lvds_mode": 2,
                        "dca_timeout": 5.0,
                        "cfg": "Config/test_full.cfg",
                    }
                ),
                encoding="utf-8",
            )

            parsed = self.module.parse_args(
                ["--defaults", str(defaults_path), "--lvds-mode", "1", "--apply"]
            )

        self.assertEqual(parsed.dca_ip, "192.168.33.180")
        self.assertEqual(parsed.system_ip, "192.168.33.30")
        self.assertEqual(parsed.mac, "12.34.56.78.90.12")
        self.assertEqual(parsed.packet_delay_us, 25)
        self.assertEqual(parsed.timeout, 5.0)
        self.assertEqual(parsed.lvds_mode, 1)
        self.assertTrue(parsed.apply)


if __name__ == "__main__":
    unittest.main()
