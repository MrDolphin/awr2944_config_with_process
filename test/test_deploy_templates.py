import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class DeployTemplateTests(unittest.TestCase):
    def test_service_uses_dialout_and_no_stale_pythonpath(self):
        service = (ROOT / "deploy" / "radar.service").read_text(encoding="utf-8")
        self.assertIn("SupplementaryGroups=dialout", service)
        self.assertIn("After=network-online.target", service)
        self.assertNotIn("python3.13", service)
        self.assertNotIn("BindsTo=dev-ttyACM0.device", service)

    def test_setup_requires_explicit_app_and_does_not_change_network(self):
        script = (ROOT / "deploy" / "setup_rpi.sh").read_text(encoding="utf-8")
        self.assertIn("APP_DIR", script)
        self.assertIn("python3-serial python3-websockets", script)
        self.assertNotIn("nmcli connection modify", script)


if __name__ == "__main__":
    unittest.main()
