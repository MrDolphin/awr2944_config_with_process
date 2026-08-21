import unittest

import numpy as np

from simulation.v03 import FmcwConfig, generate_single_scatterer_iq, process_range_doppler


class ComplexEchoRangeDopplerTests(unittest.TestCase):
    def test_single_scatterer_peak_matches_configured_range_and_velocity(self):
        config = FmcwConfig()
        iq = generate_single_scatterer_iq(
            config,
            slant_range_m=30.0,
            radial_velocity_mps=1.5,
            amplitude=1.0,
        )
        result = process_range_doppler(iq, config)

        self.assertEqual(iq.shape, (64, 256, 4, 4))
        self.assertLessEqual(
            abs(result.peak_range_m - 30.0), config.range_resolution_m
        )
        self.assertLessEqual(
            abs(result.peak_velocity_mps - 1.5), config.velocity_resolution_mps
        )
        self.assertGreater(result.peak_power_linear, 0.0)

    def test_all_virtual_channels_are_coherent_for_v03_minimal_case(self):
        config = FmcwConfig()
        iq = generate_single_scatterer_iq(
            config,
            slant_range_m=20.0,
            radial_velocity_mps=0.0,
            amplitude=2.0,
        )
        self.assertTrue(np.allclose(iq[:, :, 0, 0], iq[:, :, 3, 3]))

    def test_range_doppler_rejects_wrong_iq_shape(self):
        config = FmcwConfig()
        with self.assertRaisesRegex(ValueError, "shape"):
            process_range_doppler(np.zeros((4, 8), dtype=complex), config)
