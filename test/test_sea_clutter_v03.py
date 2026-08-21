import unittest

import numpy as np

from simulation.v03 import (
    FmcwConfig,
    compute_controlled_scatterer_weights,
    generate_multi_scatterer_iq,
    generate_single_scatterer_iq,
    process_range_doppler,
)


class ComplexEchoRangeDopplerTests(unittest.TestCase):

    def test_controlled_weights_apply_range_and_front_face_proxy(self):
        weights = compute_controlled_scatterer_weights(
            np.array([10.0, 20.0, 30.0]), np.array([10.0, -5.0, 20.0])
        )
        self.assertAlmostEqual(float(weights[0]), 1.0)
        self.assertEqual(float(weights[1]), 0.0)
        self.assertGreater(float(weights[0]), float(weights[2]))

    def test_controlled_weights_reject_mismatched_or_invalid_ranges(self):
        with self.assertRaisesRegex(ValueError, "same shape"):
            compute_controlled_scatterer_weights(np.ones(2), np.ones(3))
        with self.assertRaisesRegex(ValueError, "positive"):
            compute_controlled_scatterer_weights(np.array([0.0]), np.array([1.0]))
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

    def test_multi_scatterer_iq_contains_both_coherent_components(self):
        config = FmcwConfig()
        one = generate_single_scatterer_iq(
            config, slant_range_m=20.0, radial_velocity_mps=0.0, amplitude=1.0
        )
        two = generate_single_scatterer_iq(
            config, slant_range_m=35.0, radial_velocity_mps=0.0, amplitude=0.5
        )
        combined = generate_multi_scatterer_iq(
            config, [(20.0, 0.0, 1.0), (35.0, 0.0, 0.5)]
        )
        self.assertTrue(np.allclose(combined, one + two))
