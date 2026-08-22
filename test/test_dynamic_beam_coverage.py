import unittest
from pathlib import Path


class DynamicBeamCoverageTest(unittest.TestCase):
    def test_surface_loader_and_run_contract(self):
        from simulation.run_v04_118_dynamic_beam_coverage import load_surface, run
        path = Path("simulation/stages/v02_dynamic_sea_truth/results/matlab/v02b_quick_seed101_hs1m/data/ss0_flat_seed101.h5")
        surface = load_surface(path)
        self.assertEqual(surface["case_id"], "ss0_flat")
        self.assertTrue(callable(run))


if __name__ == "__main__":
    unittest.main()
