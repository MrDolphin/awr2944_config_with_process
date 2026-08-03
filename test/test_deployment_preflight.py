import argparse
import importlib.util
import tempfile
import unittest
from pathlib import Path


def load_preflight_module():
    path = Path(__file__).resolve().parents[1] / "tools" / "deployment_preflight.py"
    spec = importlib.util.spec_from_file_location("deployment_preflight", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DeploymentPreflightTests(unittest.TestCase):
    def setUp(self):
        self.module = load_preflight_module()
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.config = self.root / "dock.cfg"
        self.config.write_text("sensorStop\n", encoding="utf-8")

    def tearDown(self):
        self.temporary_directory.cleanup()

    def run_check(self, require_ports=False):
        args = argparse.Namespace(
            config=str(self.config), cfg_port=str(self.root / "missing-cli"), data_port=str(self.root / "missing-data"),
            capture_root=str(self.root / "captures"), ws_port=0, min_free_mb=1, require_ports=require_ports,
        )
        return {item["name"]: item for item in self.module.run_preflight(args)}

    def test_missing_ports_warn_before_hardware_is_connected(self):
        results = self.run_check()
        self.assertTrue(results["config"]["ok"])
        self.assertTrue(results["capture_root"]["ok"])
        self.assertEqual(results["cfg_port"]["level"], "warn")

    def test_missing_ports_fail_in_hardware_required_mode(self):
        results = self.run_check(require_ports=True)
        self.assertFalse(results["data_port"]["ok"])
        self.assertEqual(results["data_port"]["level"], "fail")


if __name__ == "__main__":
    unittest.main()
