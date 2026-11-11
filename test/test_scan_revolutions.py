import importlib.util
import unittest


class RevolutionTests(unittest.TestCase):
    def analyze(self, events, **kwargs):
        self.assertIsNotNone(importlib.util.find_spec('scan_revolutions'),
                             'fixed-zero revolution analyzer must exist')
        from scan_revolutions import analyze_revolutions
        return analyze_revolutions(events, direction=kwargs.pop('direction', 1),
                                   max_interval_s=0.5, max_step_deg=100, **kwargs)

    def samples(self, angles):
        return [dict(frame_num=i, host_monotonic_s=i * 0.1,
                     status='registered', yaw_deg=angle % 360)
                for i, angle in enumerate(angles)]

    def test_full_turn_between_two_observed_crossings(self):
        result = self.analyze(self.samples([270, 350, 10, 90, 180, 270, 350, 10, 50]))
        turns = result['turns']
        self.assertEqual([t['complete'] for t in turns], [False, True, False])
        self.assertEqual([t['valid_lines'] for t in turns], [2, 5, 2])
        self.assertEqual(turns[1]['max_gap_deg'], 90)
        self.assertEqual(turns[1]['first_frame_num'], 2)
        self.assertFalse(result['rf_time_calibrated'])

    def test_reverse_and_shifted_zero_use_same_fixed_boundary(self):
        angles = [270, 350, 10, 90, 180, 270, 350, 10]
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                result = self.analyze(self.samples([30 + direction * a for a in angles]),
                                      direction=direction, zero_deg=30)
                self.assertEqual(sum(t['complete'] for t in result['turns']), 1)
                self.assertEqual(result['turns'][1]['max_gap_deg'], 90)

    def test_exact_zero_start_is_boundary_and_end_does_not_duplicate(self):
        result = self.analyze(self.samples([0, 90, 180, 270, 360]))
        self.assertEqual([t['valid_lines'] for t in result['turns']], [4, 1])
        self.assertEqual([t['complete'] for t in result['turns']], [True, False])

    def test_stationary_samples_are_not_full_turn(self):
        result = self.analyze(self.samples([20] * 20))
        self.assertEqual(len(result['turns']), 1)
        self.assertFalse(result['turns'][0]['complete'])
        self.assertEqual(result['turns'][0]['unique_azimuths'], 1)
        self.assertEqual(result['turns'][0]['max_gap_deg'], 360)

    def test_long_interval_reverse_and_counter_reset_break_segments(self):
        for kind in ('time_gap', 'direction_reversal', 'frame_counter_reset'):
            records = self.samples([270, 350, 10, 90, 180])
            if kind == 'time_gap':
                for row in records[2:]:
                    row['host_monotonic_s'] += 1
            elif kind == 'direction_reversal':
                records[2]['yaw_deg'] = 340
            else:
                records[2]['frame_num'] = 0
            result = self.analyze(records)
            self.assertIn(kind, [b['reason'] for b in result['breaks']])
            self.assertFalse(any(t['complete'] for t in result['turns']))

    def test_large_angular_gap_cannot_be_filled(self):
        result = self.analyze(self.samples([270, 350, 120]))
        self.assertEqual(result['breaks'][0]['reason'], 'angular_gap')
        self.assertFalse(any(t['complete'] for t in result['turns']))

    def test_rejections_and_missing_raw_frames_are_not_conflated(self):
        rows = self.samples([0, 90, 180, 270, 360])
        rows[2] = dict(frame_num=2, host_monotonic_s=0.2,
                       status='rejected', reason='sample_gap')
        result = self.analyze(rows)
        self.assertEqual(result['registration_rejections'], {'sample_gap': 1})
        self.assertIsNone(result['raw_missing_frames'])
        self.assertEqual(sum(t['valid_lines'] for t in result['turns']), 4)
        self.assertFalse(any(t['complete'] for t in result['turns']))

    def test_counter_wrap_is_not_restart(self):
        rows = self.samples([0, 90, 180, 270, 360])
        for i, row in enumerate(rows):
            row['frame_num'] = (2**32 - 2 + i) % 2**32
        result = self.analyze(rows)
        self.assertEqual(result['breaks'], [])
        self.assertTrue(result['turns'][0]['complete'])

    def test_bad_timestamp_and_changed_registration_break_continuity(self):
        for field, value, reason in [('host_monotonic_s', float('nan'), 'invalid_sample'),
                                     ('host_monotonic_s', -1, 'time_reset'),
                                     ('registration_mounting_roll_deg', 90, 'registration_changed')]:
            rows = self.samples([270, 350, 10, 90])
            for row in rows:
                row['registration_mounting_roll_deg'] = 0
            rows[2][field] = value
            result = self.analyze(rows)
            self.assertIn(reason, [b['reason'] for b in result['breaks']])
            self.assertFalse(any(t['complete'] for t in result['turns']))

    def test_empty_and_invalid_options(self):
        self.assertEqual(self.analyze([])['turns'], [])
        with self.assertRaises(ValueError):
            self.analyze([], direction=0)

    def test_capture_join_separates_raw_loss_tlv2_and_registration(self):
        from scan_revolutions import analyze_capture_records
        events = self.samples([0, 90, 180, 270, 0])
        events[2] = dict(frame_num=2, host_monotonic_s=0.2,
                         status='angular_span', yaw_deg=None)
        # Frame 3 exists but lacks TLV2; frame 4 exists and is registered.
        frames = [dict(frame_num=i, host_monotonic_s=i * 0.1,
                       tlv_types=([1] if i == 3 else [1, 2]))
                  for i in (0, 1, 2, 3, 4)]
        result = analyze_capture_records(frames, events, direction=1,
                                         max_interval_s=0.5, max_step_deg=100)
        self.assertEqual(result['raw_missing_frames'], 0)
        self.assertEqual(result['raw_tlv2_frames'], 4)
        self.assertEqual(result['registration_rejections'], {'angular_span': 1})
        self.assertEqual(sum(t['tlv2_lines'] for t in result['turns']), 3)
        self.assertEqual(sum(t['missing_tlv2'] for t in result['turns']), 1)

    def test_capture_join_keeps_recording_edge_and_raw_gap_distinct(self):
        from scan_revolutions import analyze_capture_records
        events = self.samples([270, 350, 10])
        events[0]['frame_num'] = 9  # event before recording began
        events[1]['frame_num'] = 10
        events[2]['frame_num'] = 12
        frames = [dict(frame_num=i, host_monotonic_s=t, tlv_types=[2])
                  for i, t in [(10, .1), (12, .2)]]
        result = analyze_capture_records(frames, events, direction=1,
                                         max_interval_s=0.5, max_step_deg=100)
        self.assertEqual(result['raw_missing_frames'], 1)
        self.assertEqual(result['unmatched_registration_events'], 1)
        self.assertEqual(sum(t['tlv2_lines'] for t in result['turns']), 2)

    def test_capture_join_handles_uint32_wrap_and_rejects_counter_restart(self):
        from scan_revolutions import analyze_capture_records
        frames = [dict(frame_num=i, host_monotonic_s=n * .1, tlv_types=[2])
                  for n, i in enumerate((2**32 - 2, 2**32 - 1, 0, 1))]
        result = analyze_capture_records(frames, [], direction=1)
        self.assertEqual(result['raw_missing_frames'], 0)
        frames[2]['frame_num'] = 3
        frames[3]['frame_num'] = 1
        with self.assertRaises(ValueError):
            analyze_capture_records(frames, [], direction=1)

    def test_capture_max_gap_reports_position_and_separate_evidence(self):
        from scan_revolutions import analyze_capture_records
        base = [dict(frame_num=n, host_monotonic_s=n * .01,
                     status='registered', yaw_deg=a)
                for n, a in [(0, 0), (2, 90), (3, 180), (4, 270), (5, 0)]]
        cases = [
            ('raw_missing', [0, 2, 3, 4, 5], base, {}, 'raw_frame_missing'),
            ('tlv2_missing', list(range(6)),
             [base[0], dict(frame_num=1, host_monotonic_s=.01,
                            status='registered', yaw_deg=45), *base[1:]],
             {1: [1]}, 'tlv2_missing'),
            ('registration_rejected', list(range(6)),
             [base[0], dict(frame_num=1, host_monotonic_s=.01,
                            status='rejected', reason='angular_span'), *base[1:]],
             {}, 'registration_rejected'),
            ('unobserved', list(range(6)), base, {}, 'unobserved'),
        ]
        for name, numbers, events, overrides, expected in cases:
            with self.subTest(name=name):
                frames = [dict(frame_num=n, host_monotonic_s=n * .01,
                               tlv_types=overrides.get(n, [2])) for n in numbers]
                result = analyze_capture_records(frames, events, direction=1,
                                                 max_interval_s=.5, max_step_deg=100)
                turn = result['turns'][0]
                self.assertTrue(turn['complete'])
                self.assertEqual(turn['max_gap_deg'], 90)
                self.assertEqual((turn['max_gap_start_deg'], turn['max_gap_end_deg']), (0, 90))
                self.assertEqual(turn['max_gap_evidence']['classification'], expected)
                self.assertEqual(turn['max_gap_evidence']['raw_frame_missing'] > 0,
                                 expected == 'raw_frame_missing')
                self.assertEqual(turn['max_gap_evidence']['tlv2_missing'] > 0,
                                 expected == 'tlv2_missing')
                self.assertEqual(turn['max_gap_evidence']['registration_rejected'] > 0,
                                 expected == 'registration_rejected')

    def test_capture_gap_uses_tlv2_spokes_not_point_only_registered_events(self):
        from scan_revolutions import analyze_capture_records
        angles = [0, 45, 90, 180, 270, 0]
        events = [dict(frame_num=i, host_monotonic_s=i * .01,
                       status='registered', yaw_deg=a) for i, a in enumerate(angles)]
        frames = [dict(frame_num=i, host_monotonic_s=i * .01,
                       tlv_types=([1] if i == 1 else [2])) for i in range(6)]
        result = analyze_capture_records(frames, events, direction=1,
                                         max_interval_s=.5, max_step_deg=100)
        self.assertEqual(result['turns'][0]['tlv2_lines'], 4)
        self.assertEqual(result['turns'][0]['max_gap_deg'], 90)

    def test_stationary_duplicate_bearings_do_not_break_capture_gap_report(self):
        from scan_revolutions import analyze_capture_records
        events = self.samples([20, 20, 20])
        frames = [dict(frame_num=i, host_monotonic_s=i * .1, tlv_types=[2])
                  for i in range(3)]
        turn = analyze_capture_records(frames, events, direction=1)['turns'][0]
        self.assertEqual(turn['max_gap_deg'], 360)
        self.assertFalse(turn['complete'])


if __name__ == '__main__':
    unittest.main()
