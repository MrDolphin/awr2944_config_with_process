import unittest

import numpy as np

from simulation.run_v04_43_sea_range_doppler import range_doppler
from simulation.v03 import FmcwConfig, generate_single_scatterer_iq


class SeaRangeDopplerTests(unittest.TestCase):
    def test_single_scatterer_produces_range_doppler_arrays(self):
        config = FmcwConfig(samples_per_chirp=128, chirps_per_frame=64)
        iq = generate_single_scatterer_iq(config, slant_range_m=20.0, radial_velocity_mps=0.2)
        spectrum, power, ranges, velocities, positive = range_doppler(iq, config)
        self.assertEqual(spectrum.shape, (64, 64, 4, 4))
        self.assertEqual(power.shape, (64, 64))
        self.assertEqual(len(ranges), 64)
        self.assertEqual(len(velocities), 64)
        self.assertTrue(np.max(power) > 0.0)


if __name__ == "__main__":
    unittest.main()
