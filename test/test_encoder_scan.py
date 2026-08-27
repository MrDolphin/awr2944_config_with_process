import unittest

from encoder_scan import EncoderSweepPlan


class EncoderSweepPlanTests(unittest.TestCase):
    def test_stops_at_positive_endpoint_using_measured_count(self):
        plan = EncoderSweepPlan(counts_per_rev=720, min_angle_deg=0, max_angle_deg=180)

        command = plan.command_for_count(360)

        self.assertEqual("hold", command.mode)
        self.assertEqual(0.0, command.duty)
        self.assertEqual("max_endpoint", command.reason)

    def test_slows_down_before_endpoint(self):
        plan = EncoderSweepPlan(
            counts_per_rev=720,
            min_angle_deg=0,
            max_angle_deg=180,
            cruise_duty=0.45,
            approach_duty=0.18,
            approach_window_deg=15,
        )

        command = plan.command_for_count(330)

        self.assertEqual("forward", command.mode)
        self.assertEqual(0.18, command.duty)
        self.assertEqual("approaching_max", command.reason)

    def test_reverses_only_after_settle_interval_at_endpoint(self):
        plan = EncoderSweepPlan(
            counts_per_rev=360,
            min_angle_deg=0,
            max_angle_deg=180,
            settle_s=0.25,
        )

        self.assertEqual("hold", plan.command_for_count(180, now_s=10.0).mode)
        self.assertEqual("hold", plan.command_for_count(180, now_s=10.24).mode)

        command = plan.command_for_count(180, now_s=10.25)

        self.assertEqual("reverse", command.mode)
        self.assertEqual("settled_at_max", command.reason)

    def test_rejects_an_uncalibrated_encoder_scale(self):
        with self.assertRaises(ValueError):
            EncoderSweepPlan(counts_per_rev=0, min_angle_deg=0, max_angle_deg=180)

    def test_reports_a_clamped_angle_from_the_measured_count(self):
        plan = EncoderSweepPlan(counts_per_rev=720, min_angle_deg=0, max_angle_deg=180)

        self.assertEqual(90.0, plan.angle_for_count(180))
        self.assertEqual(180.0, plan.angle_for_count(500))

    def test_capture_gated_plan_holds_at_endpoint_until_capture_is_released(self):
        plan = EncoderSweepPlan(
            counts_per_rev=360,
            min_angle_deg=0,
            max_angle_deg=180,
            settle_s=0.25,
            require_capture_release=True,
        )

        plan.command_for_count(180, now_s=10.0)
        ready = plan.command_for_count(180, now_s=10.25)

        self.assertEqual("hold", ready.mode)
        self.assertEqual("capture_ready_at_max", ready.reason)
        self.assertTrue(plan.release_capture())
        self.assertEqual("reverse", plan.command_for_count(180, now_s=10.26).mode)


if __name__ == "__main__":
    unittest.main()
