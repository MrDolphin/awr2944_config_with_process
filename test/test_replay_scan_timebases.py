import unittest

from tools.replay_scan_timebases import compare_registered_turns


class ReplayScanTimebasesTests(unittest.TestCase):
    def test_batched_host_arrivals_create_gap_but_counter_clock_does_not(self):
        frames = []
        for index in range(450):
            # Eight consecutive RF frames arrive as one short host burst.
            parse_time = index // 8 * .08 + index % 8 * .0004
            frames.append(dict(frame_num=1000 + index,
                               host_monotonic_s=parse_time, tlv_types=[2]))
        samples = [dict(query_started_monotonic_s=i * .02 - .0001,
                        query_completed_monotonic_s=i * .02 + .0001,
                        angle_deg=(i * .02 * 180) % 360)
                   for i in range(230)]
        result = compare_registered_turns(frames, samples, period_ms=10,
                                          direction=1)
        self.assertEqual(result['raw_missing_frames'], 0)
        self.assertEqual(result['tlv2_frames'], 450)
        self.assertGreater(result['host_parse']['max_complete_turn_gap_deg'], 10)
        self.assertLess(result['counter_estimate']['max_complete_turn_gap_deg'], 3)
        host_turn = next(turn for turn in result['host_parse']['turns']
                         if turn['complete'])
        self.assertIn('max_gap_start_deg', host_turn)
        self.assertIn('max_gap_end_deg', host_turn)
        self.assertGreater(host_turn['max_gap_end_frame_num'],
                           host_turn['max_gap_start_frame_num'])
        self.assertFalse(result['counter_estimate']['rf_time_calibrated'])
        self.assertFalse(result['counter_estimate']['encoder_offset_calibrated'])

    def test_counter_gap_is_retained_and_backward_counter_is_rejected(self):
        frames = [dict(frame_num=i, host_monotonic_s=i * .01,
                       tlv_types=[2]) for i in (0, 1, 3, 4)]
        samples = [dict(query_started_monotonic_s=i * .01 - .0001,
                        query_completed_monotonic_s=i * .01 + .0001,
                        angle_deg=i * 1.8) for i in range(6)]
        result = compare_registered_turns(frames, samples, period_ms=10,
                                          direction=1)
        self.assertEqual(result['raw_missing_frames'], 1)
        self.assertAlmostEqual(result['counter_estimate']['max_adjacent_step_deg'], 3.6)
        frames[1]['tlv_types'] = []
        missing_tlv2 = compare_registered_turns(frames, samples, period_ms=10,
                                                direction=1)
        self.assertEqual(missing_tlv2['tlv2_frames'], 3)
        self.assertEqual(missing_tlv2['counter_estimate']['registered_tlv2_lines'], 3)
        frames[2]['frame_num'] = 0
        with self.assertRaises(ValueError):
            compare_registered_turns(frames, samples, period_ms=10, direction=1)

    def test_stationary_encoder_does_not_invent_a_complete_turn(self):
        frames = [dict(frame_num=i, host_monotonic_s=i * .01,
                       tlv_types=[2]) for i in range(10)]
        samples = [dict(query_started_monotonic_s=i * .02 - .0001,
                        query_completed_monotonic_s=i * .02 + .0001,
                        angle_deg=35.0) for i in range(6)]
        result = compare_registered_turns(frames, samples, period_ms=10,
                                          direction=1)
        self.assertEqual(result['host_parse']['complete_turns'], 0)
        self.assertIsNone(result['host_parse']['max_complete_turn_gap_deg'])


if __name__ == '__main__':
    unittest.main()
