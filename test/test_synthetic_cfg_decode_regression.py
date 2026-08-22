import tempfile
import unittest
from pathlib import Path


class SyntheticCfgDecodeRegressionTest(unittest.TestCase):
    def test_full_dimension_fixture_decodes_to_expected_shape(self):
        from simulation.run_v04_115_synthetic_cfg_decode_regression import run
        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("Config/profile_3d_3Azim_1ElevTx_awr2944P.cfg"), Path(directory) / "out")
            self.assertTrue(result["synthetic_only"])
            self.assertEqual(result["decoded"].get("decoded"), True)
            self.assertEqual(result["decoded_iq_shape"], [64, 656, 4])


if __name__ == "__main__":
    unittest.main()
