import unittest
from pathlib import Path

import numpy as np

from simulation.run_v04_42_physical_sea_microfacets import derive_frame, load_surface


class PhysicalSeaMicrofacetTests(unittest.TestCase):
    def test_height_truth_derives_velocity_normals_and_doppler(self):
        path = Path("simulation/stages/v02_dynamic_sea_truth/results/matlab/v02b_forward_seed101_hs1m/data/ss2_normal_seed101.h5")
        surface = load_surface(path)
        facets, stats = derive_frame(surface, 1, surface["height_m"][0], np.random.default_rng(2), max_facets=8)
        self.assertEqual(len(facets), 8)
        self.assertIn("rms_radial_velocity_mps", stats)
        self.assertTrue(np.isfinite(float(facets[0]["doppler_hz"])))
        normal_norm = np.sqrt(float(facets[0]["normal_x"]) ** 2 + float(facets[0]["normal_y"]) ** 2 + float(facets[0]["normal_z"]) ** 2)
        self.assertAlmostEqual(normal_norm, 1.0, places=6)


if __name__ == "__main__":
    unittest.main()
