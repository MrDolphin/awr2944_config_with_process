import unittest

from simulation.v03 import FmcwConfig
from simulation.v04 import estimate_aoa_from_channels, generate_aoa_scatterer_iq


class VirtualArrayAoATests(unittest.TestCase):
    def test_known_azimuth_and_elevation_are_recovered(self):
        config = FmcwConfig()
        iq = generate_aoa_scatterer_iq(
            config, slant_range_m=30.0, radial_velocity_mps=0.0,
            azimuth_deg=20.0, elevation_deg=10.0,
        )
        azimuth, elevation = estimate_aoa_from_channels(iq, config)
        self.assertAlmostEqual(azimuth, 20.0, places=5)
        self.assertAlmostEqual(elevation, 10.0, places=5)

    def test_v04_preserves_iq_contract_shape(self):
        config = FmcwConfig()
        iq = generate_aoa_scatterer_iq(
            config, slant_range_m=20.0, radial_velocity_mps=0.0,
            azimuth_deg=0.0, elevation_deg=0.0,
        )
        self.assertEqual(iq.shape, (64, 256, 4, 4))
