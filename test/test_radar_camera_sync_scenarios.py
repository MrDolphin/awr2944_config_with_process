import unittest

from radar_camera_sync_scenarios import run_timing_scenario


MS = 1_000_000


class RadarCameraSyncScenarioTests(unittest.TestCase):
    def test_gap_is_degraded_then_stale_and_recovery_matches_again(self):
        report = run_timing_scenario(
            camera_timestamps_ns=[0, 33 * MS, 66 * MS, 200 * MS, 233 * MS],
            radar_timestamps_ns=[20 * MS, 53 * MS, 90 * MS, 140 * MS, 180 * MS, 220 * MS],
        )

        self.assertEqual(
            [sample.status for sample in report.samples],
            ["matched", "matched", "matched", "degraded", "stale", "matched"],
        )
        self.assertEqual(report.status_counts, {"matched": 4, "degraded": 1, "stale": 1, "unavailable": 0})
        self.assertEqual(report.matched_ratio, 4 / 6)
        self.assertEqual(report.absolute_matched_offset_p95_ms, 24.0)

    def test_capacity_evicts_old_capture_times_before_later_radar_frame(self):
        report = run_timing_scenario(
            camera_timestamps_ns=[0, 10 * MS, 20 * MS, 30 * MS],
            radar_timestamps_ns=[35 * MS],
            capacity=2,
        )

        self.assertEqual(report.retained_frame_ids, (3, 4))
        self.assertEqual(report.samples[0].frame_id, 4)
        self.assertEqual(report.samples[0].time_offset_ms, -5.0)

    def test_rejects_non_monotonic_time_sequences(self):
        with self.assertRaisesRegex(ValueError, "camera_timestamps_ns must be strictly increasing"):
            run_timing_scenario(camera_timestamps_ns=[10, 10], radar_timestamps_ns=[])
        with self.assertRaisesRegex(ValueError, "radar_timestamps_ns must be strictly increasing"):
            run_timing_scenario(camera_timestamps_ns=[], radar_timestamps_ns=[20, 10])


if __name__ == "__main__":
    unittest.main()
