import unittest

import numpy as np

from simulation.run_v04_48_physical_target_model import target_amplitude


class PhysicalTargetModelTests(unittest.TestCase):
    def test_relative_r4_amplitude_decreases_with_range(self):
        near = target_amplitude(1.0, 10.0, 20.0)
        far = target_amplitude(1.0, 10.0, 40.0)
        self.assertAlmostEqual(far / near, 1.0 / 4.0, places=6)

    def test_rcs_increases_amplitude_by_sqrt(self):
        base = target_amplitude(1.0, 10.0, 20.0, rcs_m2=1.0)
        larger = target_amplitude(1.0, 10.0, 20.0, rcs_m2=4.0)
        self.assertAlmostEqual(larger / base, 2.0, places=6)


if __name__ == "__main__":
    unittest.main()
