import unittest

from scan_registration import (
    AngleSample,
    EncoderDirectionVerifier,
    estimate_frame_yaw,
    register_points,
)


class ScanRegistrationTests(unittest.TestCase):
    def test_encoder_direction_verifier_handles_wrap_and_ignores_jitter(self):
        increasing = EncoderDirectionVerifier(min_travel_deg=2.0)
        self.assertIsNone(increasing.add_angle(359.0))
        self.assertIsNone(increasing.add_angle(0.2))
        self.assertEqual(increasing.add_angle(1.5), 1)

        decreasing = EncoderDirectionVerifier(min_travel_deg=2.0)
        self.assertIsNone(decreasing.add_angle(1.0))
        self.assertIsNone(decreasing.add_angle(0.8))
        self.assertIsNone(decreasing.add_angle(1.0))
        self.assertEqual(decreasing.add_angle(358.5), -1)

    def test_interpolates_across_encoder_wrap_in_both_directions(self):
        ccw = estimate_frame_yaw(
            0.02,
            AngleSample(0.0, 0.0, 350.0),
            AngleSample(0.04, 0.04, 10.0),
            direction=1,
            max_span_deg=30.0,
        )
        self.assertEqual(ccw.status, "registered")
        self.assertAlmostEqual(ccw.yaw_deg, 0.0, places=5)

        cw = estimate_frame_yaw(
            0.02,
            AngleSample(0.0, 0.0, 10.0),
            AngleSample(0.04, 0.04, 350.0),
            direction=-1,
            max_span_deg=30.0,
        )
        self.assertEqual(cw.status, "registered")
        self.assertAlmostEqual(cw.yaw_deg, 0.0, places=5)

    def test_rejects_possible_hidden_revolution_even_when_angles_match(self):
        result = estimate_frame_yaw(
            1.0, AngleSample(0.0, 0.0, 30.0), AngleSample(2.0, 2.0, 30.0),
            direction=1, max_gap_s=2.1,
        )
        self.assertEqual(result.status, "ambiguous_wrap")

    def test_applies_zero_and_mounting_yaw_only_after_interpolation(self):
        result = estimate_frame_yaw(
            0.02,
            AngleSample(0.0, 0.0, 20.0),
            AngleSample(0.04, 0.04, 26.0),
            direction=1,
            zero_deg=10.0,
            mounting_yaw_deg=5.0,
            max_span_deg=30.0,
        )
        self.assertAlmostEqual(result.yaw_deg, 18.0)

    def test_rejects_unbracketed_stale_slow_or_ambiguous_samples(self):
        before = AngleSample(0.0, 0.01, 0.0)
        after = AngleSample(0.20, 0.21, 36.0)
        self.assertEqual(estimate_frame_yaw(-0.01, before, after, direction=1).status, "unbracketed")
        self.assertEqual(estimate_frame_yaw(0.1, before, after, direction=1).status, "sample_gap")
        self.assertEqual(
            estimate_frame_yaw(0.1, before, after, direction=1, max_gap_s=0.3).status,
            "angular_span",
        )
        long_query = AngleSample(0.0, 0.08, 0.0)
        self.assertEqual(
            estimate_frame_yaw(0.1, long_query, after, direction=1, max_gap_s=0.3,
                               max_span_deg=40.0).status,
            "query_duration",
        )
        ambiguous = AngleSample(0.2, 0.2, 190.0)
        self.assertEqual(
            estimate_frame_yaw(0.1, before, ambiguous, direction=1,
                               max_gap_s=0.3, max_span_deg=360.0).status,
            "ambiguous_wrap",
        )

    def test_roll_then_yaw_transform_preserves_original_fields(self):
        result = estimate_frame_yaw(
            0.06, AngleSample(0.0, 0.0, 0.0), AngleSample(0.12, 0.12, 90.0),
            direction=1, max_gap_s=0.13, max_span_deg=100.0,
        )
        original = {"x": 1.0, "y": 2.0, "z": 3.0, "snr": 12.0}
        rotated = register_points([original], result, mounting_roll_deg=90.0)[0]
        self.assertAlmostEqual(rotated["x"], 1.0 / 2 ** 0.5)
        self.assertAlmostEqual(rotated["y"], 5.0 / 2 ** 0.5)
        self.assertAlmostEqual(rotated["z"], -1.0)
        self.assertEqual(rotated["snr"], 12.0)
        self.assertEqual(original["x"], 1.0)

    def test_unregistered_frame_cannot_create_rotated_points(self):
        result = estimate_frame_yaw(
            0.5, AngleSample(0.0, 0.0, 0.0), AngleSample(1.0, 1.0, 90.0),
            direction=1,
        )
        self.assertIsNone(register_points([{"x": 1, "y": 2, "z": 0}], result))

    def test_rejects_nonfinite_quality_limit(self):
        with self.assertRaises(ValueError):
            estimate_frame_yaw(
                0.5, AngleSample(0.0, 0.0, 0.0), AngleSample(1.0, 1.0, 1.0),
                direction=1, max_gap_s=float("nan"),
            )


if __name__ == "__main__":
    unittest.main()
