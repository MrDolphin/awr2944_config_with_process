import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_56_measured_aoa_package import check_manifest


class MeasuredAoaPackageTests(unittest.TestCase):
    def test_missing_evidence_does_not_pass_hardware_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); manifest = root / "manifest.json"
            manifest.write_text(json.dumps({"coverage": {"azimuth_deg": [0, 10], "elevation_deg": [0, 5], "range_m": [20, 30]}, "scenes": [{"scene_id": "s1", "capture": "missing.bin", "cfg": "missing.cfg", "truth": "missing.json"}]}), encoding="utf-8")
            result = check_manifest(manifest)
            self.assertFalse(result["measured_aoa_ready"])
            self.assertFalse(result["checks"]["scene_count_at_least_4"])


if __name__ == "__main__":
    unittest.main()
