import hashlib
import io
import struct
import unittest

from tools.replay_scan_timebases import (
    compare_live_turns, compare_registered_turns, validate_live_replay_capture,
    verify_live_replay_profiles,
)


class ReplayScanTimebasesTests(unittest.TestCase):
    def test_relative_yaw_uses_the_original_frame_and_wraps_at_zero(self):
        frames = [dict(frame_num=i, host_monotonic_s=stamp, tlv_types=[2],
                       range_profile_valid=True)
                  for i, stamp in enumerate((.01, .045, .08))]
        samples = [dict(query_started_monotonic_s=i * .01 - .0001,
                        query_completed_monotonic_s=i * .01 + .0001,
                        angle_deg=(354 + i * 1.8) % 360) for i in range(7)]
        result = compare_live_turns(frames, samples, period_ms=10,
                                    direction=1, include_frame_offsets=True)
        offset = result['relative_yaw']
        self.assertEqual(offset['matched_tlv2_frames'], 2)
        self.assertEqual(offset['counter_only_tlv2_frames'], 1)
        self.assertEqual(offset['host_only_tlv2_frames'], 0)
        row = offset['frames'][1]
        self.assertEqual((row['frame_num'], row['host_monotonic_s']), (1, .045))
        self.assertAlmostEqual(row['host_yaw_deg'], 2.1)
        self.assertAlmostEqual(row['counter_yaw_deg'], 357.6)
        self.assertAlmostEqual(row['delta_deg'], -4.5)

    def test_empty_overlap_does_not_claim_zero_shift(self):
        frames = [dict(frame_num=i, host_monotonic_s=i * .01, tlv_types=[2],
                       range_profile_valid=True) for i in range(2)]
        samples = [dict(query_started_monotonic_s=stamp, query_completed_monotonic_s=stamp,
                        angle_deg=0) for stamp in (.1, .2)]
        offset = compare_live_turns(frames, samples, period_ms=10,
                                    direction=1)['relative_yaw']
        self.assertEqual(offset['neither_tlv2_frames'], 2)
        self.assertIsNone(offset['median_delta_deg'])
        self.assertNotIn('frames', offset)

    def test_live_replay_requires_intact_capture_logs_and_cfg_snapshot(self):
        content = 'frameCfg 0 0 16 0 656 10 1 0'
        metadata = dict(
            recording_status='completed', writer_error=None, frames=10, motor_samples=5,
            dropped_frames=0, dropped_motor_samples=0,
            active_config=dict(content=content,
                               sha256=hashlib.sha256(content.encode()).hexdigest()),
            registration=dict(enabled=True, motor_active=True, encoder_direction_verified=True),
        )
        validate_live_replay_capture(metadata, frame_count=10, sample_count=5)
        for change in (dict(recording_status='recording'), dict(writer_error='disk'),
                       dict(frames=9), dict(dropped_frames=1),
                       dict(motor_samples=4), dict(dropped_motor_samples=1),
                       dict(active_config=dict(content=content, sha256='bad')),
                       dict(registration=dict(enabled=True, motor_active=False)),
                       dict(registration=dict(enabled=True, motor_active=True))):
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_live_replay_capture(dict(metadata, **change),
                                             frame_count=10, sample_count=5)

    def test_tlv2_label_alone_cannot_create_a_replay_spoke(self):
        def packet(frame_num, payload):
            magic = bytes.fromhex('0201040306050807')
            return (struct.pack('<8sIIIIIIII', magic, 0, 48 + len(payload), 0,
                                frame_num, 0, 0, 1, 0)
                    + struct.pack('<II', 2, len(payload)) + payload)

        frames, payloads = [], []
        for number, payload in enumerate([b'\x01\x00', b'', b'\x01']):
            raw = packet(number, payload)
            frames.append(dict(frame_num=number, host_monotonic_s=.02 + number * .01,
                               raw_offset=sum(map(len, payloads)), raw_length=len(raw),
                               tlv_types=[2]))
            payloads.append(raw)
        checked = verify_live_replay_profiles(frames, io.BytesIO(b''.join(payloads)))
        samples = [dict(query_started_monotonic_s=i * .02,
                        query_completed_monotonic_s=i * .02 + .002,
                        angle_deg=i * 3.6) for i in range(4)]
        report = compare_live_turns(checked, samples, period_ms=10, direction=1)
        self.assertEqual(report['tlv2_frames'], 1)
        self.assertEqual(report['invalid_tlv2_frames'], 2)
        for mode in ('host_parse', 'counter_estimate'):
            self.assertEqual(report[mode]['registered_tlv2_lines'], 1)
        with self.assertRaisesRegex(ValueError, 'identity'):
            verify_live_replay_profiles([dict(frames[0], frame_num=99)],
                                        io.BytesIO(payloads[0]))

    def test_live_replay_reduces_gaps_without_adding_frames_or_tlv2(self):
        frames = [dict(frame_num=i, host_monotonic_s=.03 + (i // 4) * .04
                       + (i % 4) * .001, tlv_types=[] if i == 50 else [2],
                       range_profile_valid=i != 50)
                  for i in range(450) if i != 60]
        samples = [dict(query_started_monotonic_s=i * .023,
                        query_completed_monotonic_s=i * .023 + .002,
                        angle_deg=(i * .023 * 180) % 360)
                   for i in range(200)]
        result = compare_live_turns(frames, samples, period_ms=10, direction=1)
        self.assertEqual(result['raw_missing_frames'], 1)
        self.assertEqual(result['tlv2_frames'], 448)
        host, candidate = result['host_parse'], result['counter_estimate']
        self.assertGreaterEqual(len(host['complete_turns']), 2)
        self.assertEqual(len(host['complete_turns']), len(candidate['complete_turns']))
        self.assertEqual(
            [turn['completed_at_monotonic_s'] for turn in host['complete_turns']],
            [turn['completed_at_monotonic_s'] for turn in candidate['complete_turns']],
        )
        self.assertLess(candidate['complete_turns'][0]['max_gap_deg'],
                        host['complete_turns'][0]['max_gap_deg'])
        for mode in (host, candidate):
            self.assertEqual(sum(mode['frame_status_counts'].values()), 449)
            self.assertLessEqual(mode['registered_tlv2_lines'], 448)
            self.assertFalse(mode['rf_time_calibrated'])

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
