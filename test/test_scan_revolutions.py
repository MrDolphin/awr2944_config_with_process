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


if __name__ == '__main__':
    unittest.main()
