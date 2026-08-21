import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_90_known_angle_capture_contract import template
from simulation.run_v04_96_finalize_manifests import run, sha256


class FinalizeManifestsTest(unittest.TestCase):
    def test_hashes_only_existing_files_and_preserves_truth_fields(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "captures"; case = root / "case01"; case.mkdir(parents=True)
            payload = template(); payload["capture_id"] = "case01"; payload["target"]["azimuth_deg"] = 17.0
            (case / "manifest.json").write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
            (case / "capture.bin").write_bytes(b"iq")
            (case / "profile.cfg").write_text("sensorStart\n", encoding="utf-8")
            summary = run(root, Path(temp) / "output")
            updated = json.loads((case / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["both_found_count"], 1)
            self.assertEqual(updated["target"]["azimuth_deg"], 17.0)
            self.assertEqual(updated["capture_sha256"], sha256(case / "capture.bin"))
            self.assertEqual(updated["capture"]["cfg_sha256"], sha256(case / "profile.cfg"))
            self.assertEqual(updated["evidence_status"], "captured_pending_validation")


if __name__ == "__main__":
    unittest.main()
