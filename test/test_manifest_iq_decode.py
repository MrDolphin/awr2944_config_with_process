import json
import tempfile
import unittest
from pathlib import Path


class ManifestIqDecodeTest(unittest.TestCase):
    def test_missing_capture_is_a_blocked_input_not_a_decode_success(self):
        from simulation.run_v04_112_manifest_iq_decode import run
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"
            manifest.write_text(json.dumps({"capture_file": "capture.bin", "capture": {"samples_per_chirp": 656, "chirps": 64, "rx_count": 4, "tdm_tx_sequence": [0, 2, 3, 1]}}), encoding="utf-8")
            result = run(manifest, root / "out")
            self.assertEqual(result["status"], "awaiting_valid_dca_iq")
            self.assertFalse(result["decoded"])
            self.assertIn("capture_file_missing", result["issues"])


if __name__ == "__main__":
    unittest.main()
