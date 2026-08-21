import unittest
from pathlib import Path

from simulation.run_v04_41_sea_clutter_aoa import load_surface, surface_snapshot
import numpy as np


class SeaClutterAoATests(unittest.TestCase):
    def test_v02_surface_can_be_sampled_as_facets(self):
        path = Path("simulation/stages/v02_dynamic_sea_truth/results/matlab/v02b_forward_seed101_hs1m/data/ss2_normal_seed101.h5")
        surface = load_surface(path)
        facets, xyz, meta = surface_snapshot(surface, 0, np.random.default_rng(1), max_facets=16)
        self.assertEqual(facets.shape, (16, 3))
        self.assertEqual(xyz.shape, (16, 3))
        self.assertEqual(meta["facet_count"], 16)


if __name__ == "__main__":
    unittest.main()
