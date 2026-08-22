import tempfile
import unittest
from pathlib import Path


class CfgLvdsComparisonV138Test(unittest.TestCase):
    def test_detects_lvds_difference_without_approving_replacement(self):
        from simulation.run_v04_138_cfg_lvds_comparison import run

        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "target.cfg"; reference = Path(directory) / "reference.cfg"
            target.write_text("profileCfg 0 77 1 2 3 4 5 6 7 656 8 9 10 11\n", encoding="utf-8")
            reference.write_text("profileCfg 0 77 1 2 3 4 5 6 7 256 8 9 10 11\nlvdsStreamCfg -1 0 1 0\n", encoding="utf-8")
            result = run(target, reference, Path(directory) / "out")
        self.assertFalse(result["target_lvds_present"])
        self.assertTrue(result["reference_lvds_present"])
        self.assertFalse(result["safe_to_replace_target_cfg"])


if __name__ == "__main__":
    unittest.main()
