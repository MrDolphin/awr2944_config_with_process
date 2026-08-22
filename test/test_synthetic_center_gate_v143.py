import tempfile
import unittest
from pathlib import Path


class SyntheticCenterGateV143Test(unittest.TestCase):
    def test_synthetic_fixture_passes_gate_but_not_hardware_validation(self):
        from simulation.run_v04_143_synthetic_center_gate_regression import run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cfg = root / "candidate.cfg"
            cfg.write_text(
                "profileCfg 0 77 1 2 3 4 5 6 7 8 16 9 10 11\n"
                "chirpCfg 0 0 0 0 0 0 0 1\n"
                "frameCfg 0 0 1 0 16 100 1 0\n"
                "sensorStart\n",
                encoding="utf-8",
            )
            result = run(cfg, root / "out")
            self.assertTrue(result["synthetic_only"])
            self.assertTrue(result["decoded"])
            self.assertFalse(result["hardware_aoa_validated"])


if __name__ == "__main__":
    unittest.main()
