import tempfile
import unittest
from pathlib import Path


class KnownAngleManifestBatchV135Test(unittest.TestCase):
    def test_five_templates_are_cfg_linked(self):
        from simulation.run_v04_135_known_angle_manifest_batch import run

        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/hardware/awr2944pev/v04_113_cfg_linked_capture_scaffold/ka001_az-30_el-10_r10/profile_3d_3Azim_1ElevTx_awr2944P.cfg"), Path(directory) / "out")
            case = Path(directory) / "out" / "ka003_az+00_el-10_r10" / "manifest.json"
            text = case.read_text(encoding="utf-8")
        self.assertEqual(result["case_count"], 5)
        self.assertIn('"azimuth_deg": 0.0', text)
        self.assertFalse(result["real_capture_present"])


if __name__ == "__main__":
    unittest.main()
