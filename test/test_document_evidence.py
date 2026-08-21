import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_30_document_evidence import run


class DocumentEvidenceTest(unittest.TestCase):
    def test_document_audit_preserves_source_and_does_not_confirm_phase_center(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "sources"
            root.mkdir()
            (root / "example.SchDoc").write_bytes(b"TX1 RX2 ANTENNA")
            (root / "broken.pdf").write_bytes(b"not a real pdf")
            output = Path(directory) / "audit"
            summary = run([root], output)
            self.assertEqual(summary["document_count"], 2)
            self.assertFalse(summary["phase_center_confirmed"])
            self.assertFalse(summary["mapping_confirmed"])
            payload = json.loads((output / "document_evidence.json").read_text(encoding="utf-8"))
            schdoc = next(item for item in payload["documents"] if item["format"] == "schdoc")
            self.assertEqual(schdoc["text_status"], "binary_token_scan")
            with (output / "document_rf_token_candidates.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[0]["evidence_status"], "binary_token_candidate")


if __name__ == "__main__":
    unittest.main()
