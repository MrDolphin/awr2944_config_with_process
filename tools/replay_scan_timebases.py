"""Offline comparison of encoder registration using two *provisional* frame clocks.

Reads recorder files only. Frame-counter time removes host delivery jitter but
does not establish the RF acquisition epoch or calibrate encoder latency.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter
import json
import math
from pathlib import Path
import statistics

from scan_registration import AngleSample, estimate_frame_yaw
from scan_revolutions import analyze_revolutions
from tools.analyze_scan_timing import analyze_records, frame_period_from_metadata


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture_dir', type=Path)
    args = parser.parse_args()
    capture = args.capture_dir
    metadata = json.loads((capture / 'metadata.json').read_text(encoding='utf-8'))

    def jsonl(name):
        with (capture / name).open(encoding='utf-8') as source:
            return [json.loads(line) for line in source if line.strip()]

    registration = metadata['registration']
    result = compare_registered_turns(
        jsonl('frames.jsonl'), jsonl('motor_samples.jsonl'),
        period_ms=frame_period_from_metadata(metadata),
        direction=registration['direction'],
        zero_deg=registration.get('zero_deg', 0.0),
        mounting_yaw_deg=registration.get('mounting_yaw_deg', 0.0))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
