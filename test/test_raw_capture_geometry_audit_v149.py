import json
import tempfile
import unittest
from pathlib import Path


class RawCaptureGeometryAuditV149Test(unittest.TestCase):
    def _case(self, root: Path) -> Path:
        case = root / "case"; case.mkdir()
        cfg = case / "candidate.cfg"
        cfg.write_text("profileCfg 0 77 1 2 3 4 5 6 7 4 9 10 11\nchirpCfg 0 0 0 0 0 0 0 1\nframeCfg 0 0 1 4 100 1 0\nsensorStart\n", encoding="utf-8")
        manifest = {"capture_file": "capture.bin", "capture": {"cfg_file": cfg.name}}
        (case / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
        return case

    def test_missing_capture_is_not_ready(self):
        from simulation.run_v04_149_raw_capture_geometry_audit import run
        with tempfile.TemporaryDirectory() as directory:
            result = run(self._case(Path(directory)), Path(directory) / "out")
            self.assertFalse(result["ready_for_decode"]); self.assertIn("capture_missing", result["issues"])

    def test_complete_frame_is_ready_but_not_hardware_validated(self):
        from simulation.run_v04_149_raw_capture_geometry_audit import run
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); case = self._case(root)
            (case / "capture.bin").write_bytes(bytes(4 * 4 * 4 * 1))
            result = run(case, root / "out")
            self.assertTrue(result["ready_for_decode"]); self.assertEqual(result["complete_frames"], 1); self.assertFalse(result["hardware_aoa_validated"])


if __name__ == "__main__":
    unittest.main()
