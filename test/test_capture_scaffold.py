import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_95_scaffold_capture_manifests import run


class CaptureScaffoldTest(unittest.TestCase):
    def test_scaffold_creates_planned_manifests_without_fake_iq(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            summary = run(root / "captures")
            self.assertEqual(summary["case_count"], 30)
            manifests = list((root / "captures").glob("ka*/manifest.json"))
            self.assertEqual(len(manifests), 30)
            payload = json.loads(manifests[0].read_text(encoding="utf-8"))
            self.assertEqual(payload["evidence_status"], "planned_awaiting_capture")
            self.assertFalse((manifests[0].parent / "capture.bin").exists())


if __name__ == "__main__":
    unittest.main()
