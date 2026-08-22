import json
import tempfile
import unittest
from pathlib import Path


class OperatorCapturePackageV144Test(unittest.TestCase):
    def test_creates_review_only_package_without_execution(self):
        from simulation.run_v04_144_operator_capture_package import run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cfg = root / "candidate.cfg"; cfg.write_text("sensorStart\n", encoding="utf-8")
            dca = root / "cf.json"; dca.write_text(json.dumps({"DCA1000Config": {"ethernetConfig": {"DCA1000IPAddress": "192.168.1.2"}, "captureConfig": {"framesToCapture": 2}}}), encoding="utf-8")
            result = run(cfg, dca, root / "out")
            self.assertFalse(result["hardware_commands_executed"])
            self.assertFalse(result["authoritative_dca_config"])
            self.assertTrue((root / "out" / "output_analysis.md").exists())


if __name__ == "__main__":
    unittest.main()
