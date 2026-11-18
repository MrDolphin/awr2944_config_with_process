"""Offline comparison of encoder registration using two *provisional* frame clocks.

Reads recorder files only. Frame-counter time removes host delivery jitter but
does not establish the RF acquisition epoch or calibrate encoder latency.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics
import struct

from live_scan import LiveScanAssembler
from scan_registration import AngleSample, estimate_frame_yaw
from scan_revolutions import analyze_revolutions
from tools.analyze_scan_timing import analyze_records, frame_period_from_metadata
from tools.register_scan_capture import MAGIC_WORD, extract_range_profile


def _counter_indices(frames):
    indices = [0]
    for before, after in zip(frames, frames[1:]):
        step = (int(after['frame_num']) - int(before['frame_num'])) % 2**32
        if not 0 < step <= 2**31:
            raise ValueError('frame counter reset, duplicate, or moved backward')
        indices.append(indices[-1] + step)
    return indices


def _encoder_samples(records):
    samples = [AngleSample(float(row['query_started_monotonic_s']),
                           float(row['query_completed_monotonic_s']),
                           float(row['angle_deg'])) for row in records]
    if len(samples) < 2:
        raise ValueError('at least two encoder samples are required')
    midpoints = [row.midpoint_s for row in samples]
    if (any(not math.isfinite(value) for value in midpoints)
            or any(right <= left for left, right in zip(midpoints, midpoints[1:]))):
        raise ValueError('encoder sample midpoints must increase')
    return samples, midpoints


def _replay(frames, times, samples, midpoints, *, direction, zero_deg,
            mounting_yaw_deg, time_basis):
    events = []
    previous_angle = None
    largest_step = None
    for frame, timestamp in zip(frames, times):
        if 2 not in frame.get('tlv_types', []):
            continue
        right = bisect_right(midpoints, timestamp)
        if right == 0 or right == len(samples):
            status, yaw = 'unbracketed', None
        else:
            result = estimate_frame_yaw(
                timestamp, samples[right - 1], samples[right],
                direction=direction, zero_deg=zero_deg,
                mounting_yaw_deg=mounting_yaw_deg)
            status, yaw = result.status, result.yaw_deg
        events.append(dict(frame_num=int(frame['frame_num']),
                           host_monotonic_s=timestamp, status=status,
                           reason=status if status != 'registered' else None,
                           yaw_deg=yaw))
        if yaw is not None:
            if previous_angle is not None:
                step = (direction * (yaw - previous_angle)) % 360
                largest_step = step if largest_step is None else max(largest_step, step)
            previous_angle = yaw
    # These broad limits expose observed angular gaps instead of segmenting
    # them at the normal live display threshold. Encoder quality gates above
    # remain unchanged; no rejected bearing is filled in.
    result = analyze_revolutions(events, direction=direction,
                                 max_interval_s=1.0, max_step_deg=179.0)
    complete = [turn for turn in result['turns'] if turn['complete']]
    turn_rows = []
    for turn in result['turns']:
        bearings = sorted(((round((direction * events[index]['yaw_deg']) % 360, 6),
                            events[index]) for index in turn['event_indices']),
                          key=lambda row: row[0])
        gaps = [(right[0] - left[0], left, right)
                for left, right in zip(bearings, bearings[1:])]
        gaps.append((bearings[0][0] + 360 - bearings[-1][0],
                     bearings[-1], bearings[0]))
        _, start, end = max(gaps, key=lambda item: item[0])
        turn_rows.append(dict(
            turn_index=turn['turn_index'], complete=turn['complete'],
            tlv2_lines=turn['valid_lines'], max_gap_deg=turn['max_gap_deg'],
            max_gap_start_deg=start[0], max_gap_end_deg=end[0],
            max_gap_start_frame_num=start[1]['frame_num'],
            max_gap_end_frame_num=end[1]['frame_num'],
            first_frame_num=turn['first_frame_num'],
            last_frame_num=turn['last_frame_num'],
            end_reason=turn['end_reason']))
    return dict(time_basis=time_basis,
                rf_time_calibrated=False,
                encoder_offset_calibrated=False,
                registered_tlv2_lines=sum(event['status'] == 'registered'
                                          for event in events),
                registration_rejections=dict(Counter(
                    event['status'] for event in events
                    if event['status'] != 'registered')),
                max_adjacent_step_deg=largest_step,
                complete_turns=len(complete),
                max_complete_turn_gap_deg=(max(turn['max_gap_deg'] for turn in complete)
                                           if complete else None),
                turns=turn_rows,
                breaks=result['breaks'])


def compare_registered_turns(frames, motor_samples, *, period_ms,
                             direction, zero_deg=0.0, mounting_yaw_deg=0.0):
    """Compare the same raw TLV2 frames; never synthesize missing frames."""
    frames = list(frames)
    observed = analyze_records(frames)
    if observed['duplicate_frame_numbers']:
        raise ValueError('duplicate raw frame counter')
    if not math.isfinite(period_ms) or period_ms <= 0:
        raise ValueError('frame period must be positive and finite')
    indices = _counter_indices(frames)
    samples, midpoints = _encoder_samples(motor_samples)
    host_times = [float(frame['host_monotonic_s']) for frame in frames]
    period_s = period_ms / 1000
    # Median aligns only the clock origin for this replay. The unknown RF-to-
    # host and encoder query offsets remain uncalibrated.
    origin = statistics.median(time - index * period_s
                               for time, index in zip(host_times, indices))
    counter_times = [origin + index * period_s for index in indices]
    return dict(raw_frames=len(frames),
                raw_missing_frames=observed['missing_frame_numbers'],
                tlv2_frames=sum(2 in frame.get('tlv_types', []) for frame in frames),
                frame_period_ms_from_cfg=period_ms,
                comparison_note='Counter clock is an uncalibrated hypothesis, not RF time.',
                host_parse=_replay(frames, host_times, samples, midpoints,
                                   direction=direction, zero_deg=zero_deg,
                                   mounting_yaw_deg=mounting_yaw_deg,
                                   time_basis='host_packet_parse_monotonic'),
                counter_estimate=_replay(frames, counter_times, samples, midpoints,
                                         direction=direction, zero_deg=zero_deg,
                                         mounting_yaw_deg=mounting_yaw_deg,
                                         time_basis='frame_counter_x_cfg_period_estimate_only'))


def compare_live_turns(frames, motor_samples, *, period_ms, direction,
                       zero_deg=0.0, mounting_yaw_deg=0.0):
    """Replay the live assembler using recorded parse/query completion order."""
    frames, motor_samples = list(frames), list(motor_samples)
    observed = analyze_records(frames)
    _counter_indices(frames)
    _encoder_samples(motor_samples)
    if any(row.get('enabled', True) is not True for row in motor_samples):
        raise ValueError('live replay requires one enabled scan session')
    if any(type(row.get('range_profile_valid')) is not bool for row in frames):
        raise ValueError('live replay requires explicit TLV2 payload validity')
    stream = sorted(
        [(float(row['host_monotonic_s']), 1, row) for row in frames]
        + [(float(row['query_completed_monotonic_s']), 0, row)
           for row in motor_samples], key=lambda item: item[:2])
    tlv2_ids = {(int(row['frame_num']), float(row['host_monotonic_s']))
               for row in frames if row['range_profile_valid']}
    report = dict(raw_frames=len(frames),
                  raw_missing_frames=observed['missing_frame_numbers'],
                  tlv2_frames=len(tlv2_ids),
                  invalid_tlv2_frames=sum(2 in row.get('tlv_types', [])
                                          and not row['range_profile_valid'] for row in frames),
                  frame_period_ms_from_cfg=period_ms,
                  comparison_note='Live assembler replay; host parse/query completion '
                  'approximates input order. Valid TLV2 availability only; not pixel or RF validation.')
    for label, period in [('host_parse', None), ('counter_estimate', period_ms)]:
        scan = LiveScanAssembler(direction=direction, zero_deg=zero_deg,
                                 mounting_yaw_deg=mounting_yaw_deg,
                                 mounting_roll_deg=0,
                                 experimental_frame_period_ms=period)
        turns, statuses = [], Counter()
        registered_tlv2 = 0
        for stamp, kind, row in stream:
            if kind == 0:
                turn = scan.add_motor_sample(row)
                if turn is not None:
                    turns.append(dict(turn, completed_at_monotonic_s=stamp))
            else:
                number = int(row['frame_num'])
                scan.add_frame(dict(frame_num=number, host_monotonic_s=stamp,
                                    tlv_types=row.get('tlv_types', []),
                                    range_profile=[0] if (number, stamp) in tlv2_ids else None))
            for event in scan.drain_events():
                statuses[event['status']] += 1
                if (event['status'] == 'registered' and
                        (event['frame_num'], event['host_monotonic_s']) in tlv2_ids):
                    registered_tlv2 += 1
        pending = len(frames) - sum(statuses.values())
        if pending:
            statuses['unresolved_at_capture_end'] = pending
        report[label] = dict(time_basis=scan.time_basis, rf_time_calibrated=False,
                             complete_turns=turns, frame_status_counts=dict(statuses),
                             registered_tlv2_lines=registered_tlv2)
    return report


def verify_live_replay_profiles(frames, raw):
    """Join the binary packets to their index; invalid TLV2 cannot create a spoke."""
    checked = []
    for frame in frames:
        offset, length = frame.get('raw_offset'), frame.get('raw_length')
        if (type(offset) is not int or type(length) is not int or
                offset < 0 or not 40 <= length <= 2**20):
            raise ValueError('live replay found an invalid raw packet offset or length')
        raw.seek(offset)
        packet = raw.read(length)
        if (len(packet) != length or packet[:8] != MAGIC_WORD
                or struct.unpack_from('<I', packet, 12)[0] != length
                or struct.unpack_from('<I', packet, 20)[0] != frame['frame_num']):
            raise ValueError(f'raw packet identity disagrees with index at offset {offset}')
        try:
            profile = extract_range_profile(packet)
        except ValueError:
            if 2 not in frame.get('tlv_types', []):
                raise
            profile = None
        if profile is not None and 2 not in frame.get('tlv_types', []):
            raise ValueError(f'raw TLV2 is missing from the index at offset {offset}')
        checked.append(dict(frame, range_profile_valid=bool(profile and len(profile) <= 2048)))
    return checked


def validate_live_replay_capture(metadata, *, frame_count, sample_count):
    """Require intact replay inputs, without rejecting actual radar counter gaps."""
    if metadata.get('recording_status') != 'completed' or metadata.get('writer_error'):
        raise ValueError('live replay requires a completed recording without a writer error')
    for label, count in [('frames', frame_count), ('motor_samples', sample_count)]:
        if metadata.get(label) != count or metadata.get(f'dropped_{label}') != 0:
            raise ValueError(f'live replay requires intact {label} matching metadata')
    registration = metadata.get('registration') or {}
    if registration.get('enabled') is not True or registration.get('motor_active') is not True:
        raise ValueError('live replay requires registration and motor active at record start')
    if registration.get('encoder_direction_verified') is not True:
        raise ValueError('live replay requires verified encoder direction at record start')
    config = metadata.get('active_config') or {}
    content = config.get('content')
    if (not isinstance(content, str) or
            config.get('sha256') != hashlib.sha256(content.encode('utf-8')).hexdigest()):
        raise ValueError('live replay requires the original active CFG snapshot and matching SHA-256')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture_dir', type=Path)
    parser.add_argument('--live-assembler', action='store_true',
                        help='Replay both production assembler modes in arrival order')
    args = parser.parse_args()
    capture = args.capture_dir
    metadata = json.loads((capture / 'metadata.json').read_text(encoding='utf-8'))

    def jsonl(name):
        with (capture / name).open(encoding='utf-8') as source:
            return [json.loads(line) for line in source if line.strip()]

    try:
        frames, samples = jsonl('frames.jsonl'), jsonl('motor_samples.jsonl')
        if args.live_assembler:
            validate_live_replay_capture(metadata, frame_count=len(frames),
                                         sample_count=len(samples))
            with (capture / 'frames.tlv').open('rb') as raw:
                frames = verify_live_replay_profiles(frames, raw)
        registration = metadata['registration']
        compare = compare_live_turns if args.live_assembler else compare_registered_turns
        result = compare(
            frames, samples, period_ms=frame_period_from_metadata(metadata),
            direction=registration['direction'],
            zero_deg=registration.get('zero_deg', 0.0),
            mounting_yaw_deg=registration.get('mounting_yaw_deg', 0.0))
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
