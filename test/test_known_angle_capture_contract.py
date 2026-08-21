import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from simulation.run_v04_90_known_angle_capture_contract import run, template, validate_manifest


class KnownAngleCaptureContractTest(unittest.TestCase):
    def test_template_is_explicitly_not_hardware_evidence(self):
        with TemporaryDirectory() as temp:
            output = Path(temp) / "output"
            result = run(output)
            self.assertEqual(result["status"], "template_created")
            payload = json.loads((output / "known_angle_capture_manifest.template.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["evidence_status"], "template_awaiting_capture")
            self.assertFalse(payload["capture"]["channel_order_verified"])

    def test_missing_capture_and_cfg_are_reported(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            manifest = root / "manifest.json"
            manifest.write_text(json.dumps(template(), ensure_ascii=False), encoding="utf-8")
            result = validate_manifest(manifest, root)
            self.assertEqual(result["status"], "awaiting_or_invalid_capture_evidence")
            self.assertIn("capture_file_missing", result["issues"])
            self.assertIn("cfg_file_missing", result["issues"])


if __name__ == "__main__":
    unittest.main()
