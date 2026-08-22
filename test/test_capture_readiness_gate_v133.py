import json
import tempfile
import unittest
from pathlib import Path


class CaptureReadinessGateV133Test(unittest.TestCase):
    def test_incomplete_manifest_is_not_admitted(self):
        from simulation.run_v04_133_capture_readiness_gate import run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "captures"; root.mkdir(); (root / "manifest.json").write_text(json.dumps({"capture_file": "capture.bin", "capture": {"channel_order_verified": False}, "target": {}, "radar_pose": {}, "calibration": {}}), encoding="utf-8")
            result = run(root, Path(directory) / "out")
        self.assertEqual(result["manifest_count"], 1)
        self.assertEqual(result["ready_count"], 0)
        self.assertFalse(result["real_dca_iq_admitted"])


if __name__ == "__main__":
    unittest.main()
