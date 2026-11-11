import math
import unittest

from live_scan import LiveScanAssembler


def sample(when, angle):
    return {
        "query_started_monotonic_s": when,
        "query_completed_monotonic_s": when,
        "angle_deg": angle,
        "speed_rpm": 30.0,
        "enabled": True,
    }


def frame(number, when, point=None):
    return {
        "frame_num": number,
        "host_monotonic_s": when,
        "points": [point or {"x": 0.0, "y": 1.0, "z": 0.0}],
    }


class LiveScanAssemblerTests(unittest.TestCase):
    def test_latest_complete_turn_is_separate_from_rolling_window_and_cumulative_rejections(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        scan.add_motor_sample(sample(1, 0))
        for n in range(1, 61):
            if n != 10:
                source = frame(n, 1 + n * .02 - .01)
                if n != 11:
                    source['range_profile'] = [100, 200]
                scan.add_frame(source)
            scan.add_motor_sample(sample(1 + n * .02, n * 6 % 360))
        result = scan.snapshot(now_monotonic_s=2.2)
        turn = result['latest_complete_turn']
        self.assertEqual(turn['valid_tlv2_spokes'], 58)
        self.assertAlmostEqual(turn['max_gap_deg'], 18)
        self.assertAlmostEqual(turn['max_gap_start_deg'], 51)
        self.assertAlmostEqual(turn['max_gap_end_deg'], 69)
        self.assertEqual(turn['raw_frame_missing'], 1)
        self.assertEqual(turn['tlv2_missing'], 1)
        self.assertEqual(turn['registration_rejected'], {})
        self.assertEqual(result['coverage']['window'], 'rolling_360_deg')
        self.assertNotEqual(result['coverage']['valid_spokes'], None)

    def test_latest_complete_turn_counts_registration_rejection_without_calling_it_raw_loss(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0, max_span_deg=12)
        scan.add_motor_sample(sample(1, 0))
        for n in range(1, 61):
            source = frame(n, 1 + n * .02 - .01)
            source['range_profile'] = [100, 200]
            if n == 10:
                source['points'][0]['x'] = float('nan')
            scan.add_frame(source)
            scan.add_motor_sample(sample(1 + n * .02, n * 6 % 360))
        turn = scan.snapshot(now_monotonic_s=2.2)['latest_complete_turn']
        self.assertEqual(turn['raw_frame_missing'], 0)
        self.assertEqual(turn['registration_rejected'], {'invalid_point': 1})
        self.assertEqual(turn['tlv2_missing'], 0)

    def test_coverage_counts_unique_angles_and_wrap_gap_not_painted_area(self):
        for direction in (1, -1):
            scan = LiveScanAssembler(direction=direction, zero_deg=0, mounting_yaw_deg=0,
                                     mounting_roll_deg=0)
            scan.add_motor_sample(sample(1, 0))
            for n in range(1, 61):
                if n not in (10, 11):
                    source = frame(n, 1 + n * .02 - .01)
                    source['range_profile'] = [100, 100, 100]
                    scan.add_frame(source)
                scan.add_motor_sample(sample(1 + n * .02, direction * n * 6 % 360))
            coverage = scan.snapshot(now_monotonic_s=2.2)['coverage']
            self.assertEqual(coverage['window'], 'rolling_360_deg')
            self.assertEqual(coverage['valid_spokes'], 58)
            self.assertEqual(coverage['unique_azimuths'], 58)
            self.assertAlmostEqual(coverage['max_gap_deg'], 18)
            self.assertFalse(coverage['capacity_limited'])
            self.assertEqual(coverage['phase'], 'rolling')

    def test_coverage_empty_stationary_and_stale_are_not_complete_scans(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        self.assertIsNone(scan.snapshot(now_monotonic_s=1)['coverage']['max_gap_deg'])
        scan.add_motor_sample(sample(1, 359))
        for n in (1, 2):
            source = frame(n, 1 + n * .02 - .01)
            source['range_profile'] = [100, 100]
            scan.add_frame(source)
            scan.add_motor_sample(sample(1 + n * .02, 359))
        coverage = scan.snapshot(now_monotonic_s=1.04)['coverage']
        self.assertEqual(coverage['valid_spokes'], 2)
        self.assertEqual(coverage['unique_azimuths'], 1)
        self.assertEqual(coverage['max_gap_deg'], 360)
        self.assertEqual(coverage['phase'], 'collecting')
        stale = scan.snapshot(now_monotonic_s=3)['coverage']
        self.assertEqual(stale['valid_spokes'], 0)
        self.assertIsNone(stale['max_gap_deg'])
        self.assertEqual(stale['phase'], 'stale')

    def test_coverage_reports_capacity_eviction_until_it_leaves_window(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0, max_profile_spokes=2)
        scan.add_motor_sample(sample(1, 0))
        for n in range(1, 4):
            source = frame(n, 1 + n * .02 - .01)
            source['range_profile'] = [100, 200]
            scan.add_frame(source)
            scan.add_motor_sample(sample(1 + n * .02, n * 6))
        self.assertTrue(scan.snapshot(now_monotonic_s=1.06)['coverage']['capacity_limited'])
        for n in range(4, 64):
            scan.add_motor_sample(sample(1 + n * .02, n * 6 % 360))
        coverage = scan.snapshot(now_monotonic_s=2.26)['coverage']
        self.assertFalse(coverage['capacity_limited'])
        self.assertEqual(coverage['valid_spokes'], 0)
        scan.reset()
        self.assertEqual(scan.snapshot(now_monotonic_s=3)['coverage']['phase'], 'stale')

    def test_default_profile_capacity_retains_four_hundred_measured_spokes(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        scan.add_motor_sample(sample(1.0, 0))
        for number in range(1, 401):
            source = frame(number, 1.0 + number * 0.005 - 0.0025)
            source["range_profile"] = [100, 200, 300]
            scan.add_frame(source)
            scan.add_motor_sample(sample(1.0 + number * 0.005, number * 0.8))
        batch = scan.profile_delta(None, 0, max_spokes=512)
        self.assertEqual(len(batch["spokes"]), 400)
        self.assertEqual(batch["spokes"][0]["frame_num"], 1)
        self.assertEqual(batch["spokes"][-1]["frame_num"], 400)

    def test_registered_tlv2_profiles_are_delivered_once_with_measured_yaw(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        scan.add_motor_sample(sample(1.0, 0))
        source = frame(1, 1.02)
        source["range_profile"] = [100, 200, 900]
        source["range_profile_bin_size_m"] = 0.05
        scan.add_frame(source)
        source["range_profile"][2] = 0
        self.assertEqual(scan.profile_delta(None, 0)["spokes"], [])
        scan.add_motor_sample(sample(1.04, 6))
        batch = scan.profile_delta(None, 0)
        self.assertEqual(len(batch["spokes"]), 1)
        self.assertAlmostEqual(batch["spokes"][0]["yaw_deg"], 3)
        self.assertEqual(batch["spokes"][0]["range_profile"], [100, 200, 900])
        self.assertEqual(batch["spokes"][0]["range_profile_bin_size_m"], 0.05)
        self.assertFalse(batch["spokes"][0]["rf_time_calibrated"])
        self.assertEqual(scan.profile_delta(batch["generation"], batch["latest_seq"])["spokes"], [])

    def test_profile_delta_batches_without_skipping_and_clears_on_discontinuity(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        scan.add_motor_sample(sample(1.0, 0))
        for number in range(1, 4):
            before = 1.0 + (number - 1) * 0.02
            source = frame(number, before + 0.01)
            source["range_profile"] = [number] * 4
            scan.add_frame(source)
            scan.add_motor_sample(sample(1.0 + number * 0.02, number * 2))
        first = scan.profile_delta(None, 0, max_spokes=2)
        self.assertEqual([spoke["frame_num"] for spoke in first["spokes"]], [1, 2])
        self.assertTrue(first["has_more"])
        second = scan.profile_delta(first["generation"], first["latest_seq"], max_spokes=2)
        self.assertEqual([spoke["frame_num"] for spoke in second["spokes"]], [3])
        self.assertFalse(second["has_more"])
        scan.add_motor_sample(sample(1.20, 40))
        cleared = scan.profile_delta(first["generation"], second["latest_seq"])
        self.assertNotEqual(cleared["generation"], first["generation"])
        self.assertEqual(cleared["spokes"], [])

    def test_unregistered_or_invalid_tlv2_never_becomes_a_spoke(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        scan.add_motor_sample(sample(1.0, 0))
        source = frame(1, 1.01)
        source["range_profile"] = [1, 2, 3]
        scan.add_frame(source)
        scan.add_motor_sample(sample(1.02, 2))
        self.assertEqual(scan.profile_delta(None, 0)["spokes"][0]["range_profile_bin_size_m"], None)
        invalid = frame(2, 1.03)
        invalid["range_profile"] = [1, -1, 3]
        scan.add_frame(invalid)
        scan.add_motor_sample(sample(1.04, 4))
        self.assertEqual(len(scan.profile_delta(None, 0)["spokes"]), 1)

    def test_tlv2_spoke_expires_after_one_measured_turn_without_new_radar_frame(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        scan.add_motor_sample(sample(1.0, 0))
        source = frame(1, 1.01)
        source["range_profile"] = [10, 20, 30]
        scan.add_frame(source)
        scan.add_motor_sample(sample(1.02, 6))
        self.assertEqual(len(scan.profile_delta(None, 0)["spokes"]), 1)
        for number in range(2, 62):
            scan.add_motor_sample(sample(1.0 + number * 0.02, (number * 6) % 360))
        self.assertEqual(scan.profile_delta(None, 0)["spokes"], [])

    def test_waits_for_later_encoder_sample_before_plotting_frame(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=90)
        scan.add_motor_sample(sample(1.0, 0))
        scan.add_frame(frame(1, 1.02))
        self.assertEqual(scan.snapshot(now_monotonic_s=1.02)["points"], [])
        self.assertEqual(scan.snapshot(now_monotonic_s=1.02)["pending_frames"], 1)

        scan.add_motor_sample(sample(1.04, 6))
        result = scan.snapshot(now_monotonic_s=1.05)
        self.assertEqual(result["state"], "provisional")
        self.assertEqual(result["registered_frames"], 1)
        self.assertEqual(result["pending_frames"], 0)
        self.assertAlmostEqual(result["last_yaw_deg"], 3.0)
        self.assertAlmostEqual(result["points"][0]["x"], -math.sin(math.radians(3)))
        self.assertAlmostEqual(result["points"][0]["y"], math.cos(math.radians(3)))
        self.assertFalse(result["rf_time_calibrated"])
        self.assertFalse(result["pose_compensated"])

    def test_rejects_large_sample_gap_without_filling_sector(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=90)
        scan.add_motor_sample(sample(1.0, 0))
        scan.add_frame(frame(1, 1.1))
        scan.add_motor_sample(sample(1.2, 36))
        result = scan.snapshot(now_monotonic_s=1.2)
        self.assertEqual(result["points"], [])
        self.assertEqual(result["last_frame_status"], "sample_gap")
        self.assertEqual(result["rejected_frames"]["sample_gap"], 1)
        events = scan.drain_events()
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["frame_num"], 1)
        self.assertEqual(events[0]["status"], "sample_gap")
        self.assertEqual(events[0]["before_sample_midpoint_s"], 1.0)
        self.assertEqual(events[0]["after_sample_midpoint_s"], 1.2)
        self.assertFalse(events[0]["rf_time_calibrated"])

    def test_angle_wrap_uses_measured_direction(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        scan.add_motor_sample(sample(1.0, 358))
        scan.add_frame(frame(1, 1.01))
        scan.add_motor_sample(sample(1.02, 2))
        self.assertAlmostEqual(scan.snapshot(now_monotonic_s=1.02)["last_yaw_deg"], 0)
        event = scan.drain_events()[0]
        self.assertEqual(event["status"], "registered")
        self.assertAlmostEqual(event["yaw_deg"], 0)
        self.assertAlmostEqual(event["angular_span_deg"], 4)
        self.assertEqual(event["point_count"], 1)
        self.assertEqual(event["registration_direction"], 1)
        self.assertEqual(event["registration_zero_deg"], 0)
        self.assertEqual(event["max_gap_s"], 0.1)

    def test_keeps_at_most_one_measured_revolution_and_clears_when_stale(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        scan.add_motor_sample(sample(1.0, 0))
        for number in range(1, 63):
            before = 1.0 + (number - 1) * 0.05
            scan.add_frame(frame(number, before + 0.025))
            scan.add_motor_sample(sample(1.0 + number * 0.05, (number * 6) % 360))
        current = scan.snapshot(now_monotonic_s=4.11)
        self.assertLessEqual(len(current["points"]), 60)
        self.assertGreater(len(current["points"]), 0)
        stale = scan.snapshot(now_monotonic_s=6.0)
        self.assertEqual(stale["state"], "stale")
        self.assertEqual(stale["points"], [])

    def test_pending_frames_are_bounded_and_reset_removes_old_map(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0, max_pending_frames=2)
        for number in range(3):
            scan.add_frame(frame(number, 1.01 + number * 0.01))
        self.assertEqual(scan.snapshot(now_monotonic_s=1.04)["pending_frames"], 2)
        self.assertEqual(scan.snapshot(now_monotonic_s=1.04)["rejected_frames"]["pending_overflow"], 1)
        overflow = scan.drain_events()[0]
        self.assertEqual(overflow["frame_num"], 0)
        self.assertEqual(overflow["status"], "pending_overflow")
        scan.reset()
        self.assertEqual(scan.snapshot(now_monotonic_s=1.04)["pending_frames"], 0)
        self.assertEqual(scan.snapshot(now_monotonic_s=1.04)["points"], [])
        self.assertEqual(scan.drain_events(), [])

    def test_discontinuous_encoder_interval_discards_previous_sweep(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        scan.add_motor_sample(sample(1.0, 0))
        scan.add_frame(frame(1, 1.02))
        scan.add_motor_sample(sample(1.04, 6))
        self.assertEqual(len(scan.snapshot(now_monotonic_s=1.05)["points"]), 1)
        scan.add_frame(frame(2, 1.14))
        scan.add_motor_sample(sample(1.24, 42))
        result = scan.snapshot(now_monotonic_s=1.25)
        self.assertEqual(result["points"], [])
        self.assertEqual(result["last_frame_status"], "sample_gap")
        self.assertEqual(result["discontinuity_reason"], "sample_gap")
        self.assertEqual(result["rejected_frames"]["sample_gap"], 1)

    def test_stationary_encoder_cannot_grow_map_without_bound(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0, max_map_frames=3, max_map_points=3)
        scan.add_motor_sample(sample(1.0, 0))
        for number in range(1, 8):
            before = 1.0 + (number - 1) * 0.02
            scan.add_frame(frame(number, before + 0.01))
            scan.add_motor_sample(sample(1.0 + number * 0.02, 0))
        result = scan.snapshot(now_monotonic_s=1.15)
        self.assertEqual(len(result["points"]), 3)
        self.assertEqual(result["map_frames"], 3)
        self.assertTrue(result["map_truncated"])

    def test_rejects_oversized_frame_instead_of_silently_truncating_points(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0, max_frame_points=2)
        scan.add_motor_sample(sample(1.0, 0))
        oversized = frame(1, 1.01)
        oversized["points"] = oversized["points"] * 3
        scan.add_frame(oversized)
        scan.add_motor_sample(sample(1.02, 2))
        result = scan.snapshot(now_monotonic_s=1.03)
        self.assertEqual(result["points"], [])
        self.assertEqual(result["rejected_frames"]["point_limit"], 1)
        self.assertEqual(scan.drain_events()[0]["status"], "point_limit")

    def test_invalid_point_is_rejected_without_poisoning_live_map(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        bad = frame(1, 1.01, {"x": math.nan, "y": 1.0, "z": 0.0})
        scan.add_frame(bad)
        self.assertEqual(scan.snapshot(now_monotonic_s=1.01)["rejected_frames"]["invalid_point"], 1)
        event = scan.drain_events()[0]
        self.assertEqual(event["status"], "invalid_point")
        self.assertEqual(event["point_count"], 1)
        scan.add_motor_sample(sample(1.0, 0))
        scan.add_frame(frame(2, 1.01))
        scan.add_motor_sample(sample(1.02, 2))
        self.assertEqual(len(scan.snapshot(now_monotonic_s=1.03)["points"]), 1)

    def test_event_backlog_is_bounded_and_loss_is_visible(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0, max_frame_points=1,
                                 max_event_backlog=2)
        for number in range(3):
            oversized = frame(number, 1.0 + number * 0.01)
            oversized["points"] *= 2
            scan.add_frame(oversized)
        self.assertEqual(scan.snapshot(now_monotonic_s=1.03)["dropped_event_backlog"], 1)
        self.assertEqual([event["frame_num"] for event in scan.drain_events()], [1, 2])

    def test_stale_snapshot_discards_old_map_before_samples_resume(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0, stale_after_s=0.05)
        scan.add_motor_sample(sample(1.0, 0))
        scan.add_frame(frame(1, 1.02))
        scan.add_motor_sample(sample(1.04, 2))
        self.assertEqual(len(scan.snapshot(now_monotonic_s=1.05)["points"]), 1)

        stale = scan.snapshot(now_monotonic_s=1.10)
        self.assertEqual(stale["state"], "stale")
        self.assertEqual(stale["map_frames"], 0)
        self.assertEqual(stale["discontinuity_reason"], "telemetry_stale")

        scan.add_frame(frame(2, 1.06))
        scan.add_motor_sample(sample(1.08, 4))
        resumed = scan.snapshot(now_monotonic_s=1.09)
        self.assertEqual(len(resumed["points"]), 1)
        self.assertEqual(resumed["map_frames"], 1)

    def test_queued_frame_owns_validated_point_snapshot(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        source = frame(1, 1.01)
        scan.add_motor_sample(sample(1.0, 0))
        scan.add_frame(source)

        source["points"][0]["x"] = math.nan
        source["points"].append({"x": math.nan, "y": 0.0, "z": 0.0})
        scan.add_motor_sample(sample(1.02, 2))

        points = scan.snapshot(now_monotonic_s=1.03)["points"]
        self.assertEqual(len(points), 1)
        self.assertTrue(math.isfinite(points[0]["x"]))

    def test_old_points_expire_after_one_measured_turn_even_without_new_radar_frames(self):
        scan = LiveScanAssembler(direction=1, zero_deg=0, mounting_yaw_deg=0,
                                 mounting_roll_deg=0)
        scan.add_motor_sample(sample(1.0, 0))
        scan.add_frame(frame(1, 1.01))
        scan.add_motor_sample(sample(1.02, 6))
        self.assertEqual(len(scan.snapshot(now_monotonic_s=1.02)["points"]), 1)
        for number in range(2, 62):
            scan.add_motor_sample(sample(1.0 + number * 0.02, (number * 6) % 360))
        result = scan.snapshot(now_monotonic_s=2.23)
        self.assertEqual(result["points"], [])
        self.assertEqual(result["map_frames"], 0)


if __name__ == "__main__":
    unittest.main()
