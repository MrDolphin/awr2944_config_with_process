import math
import unittest


class InstallationCoverageTest(unittest.TestCase):
    def test_five_degree_boresight_matches_geometry(self):
        from simulation.run_v04_117_installation_coverage import intersection_m
        self.assertAlmostEqual(intersection_m(1.0, 0.0, 5.0), 1.0 / math.tan(math.radians(5.0)), places=9)
        self.assertIsNone(intersection_m(1.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
