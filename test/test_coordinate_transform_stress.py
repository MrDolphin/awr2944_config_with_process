import unittest

import numpy as np

from simulation.run_v04_89_coordinate_transform_stress import known_angle_metrics, transform_candidates
from simulation.v03 import FmcwConfig


class CoordinateTransformStressTest(unittest.TestCase):
    def test_four_centered_candidates_preserve_aperture(self):
        x = np.arange(16, dtype=float).reshape(4, 4)
        y = np.arange(16, dtype=float).reshape(4, 4) * 0.1
        candidates = transform_candidates((x, y))
        self.assertEqual(set(candidates), {"identity", "mirror_x", "mirror_y", "rotate_180"})
        for candidate_x, candidate_y in candidates.values():
            self.assertAlmostEqual(float(np.mean(candidate_x)), 0.0)
            self.assertAlmostEqual(float(np.mean(candidate_y)), 0.0)
            self.assertAlmostEqual(float(np.ptp(candidate_x)), float(np.ptp(x)))
            self.assertAlmostEqual(float(np.ptp(candidate_y)), float(np.ptp(y)))

    def test_identity_is_zero_error_for_matching_synthetic_geometry(self):
        config = FmcwConfig()
        positions = transform_candidates((np.arange(16, dtype=float).reshape(4, 4) * 0.001,
                                           np.arange(16, dtype=float).reshape(4, 4) * 0.0001))["identity"]
        metrics = known_angle_metrics(config, positions, positions)
        self.assertAlmostEqual(metrics["combined_rmse_deg"], 0.0)


if __name__ == "__main__":
    unittest.main()
