import tempfile
import unittest
from pathlib import Path


class CfgLinkedCaptureScaffoldTest(unittest.TestCase):
    def test_generates_all_cases_with_cfg_consistent_manifest(self):
        from simulation.run_v04_113_cfg_linked_capture_scaffold import run
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "scaffold"
            result = run(Path("Config/profile_3d_3Azim_1ElevTx_awr2944P.cfg"), output)
            self.assertEqual(result["case_count"], 30)
            manifest = output / "ka001_az-30_el-10_r10" / "manifest.json"
            self.assertTrue(manifest.exists())
            text = manifest.read_text(encoding="utf-8")
            self.assertIn('"samples_per_chirp": 656', text)
            self.assertIn('"tdm_tx_sequence": [', text)
            self.assertFalse((output / "ka001_az-30_el-10_r10" / "capture.bin").exists())


if __name__ == "__main__":
    unittest.main()
