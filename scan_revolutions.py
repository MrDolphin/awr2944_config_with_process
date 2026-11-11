"""Offline fixed-zero turns from registered bearings; never controls hardware.

Completeness means two observed zero-crossing brackets, NOT full angular
coverage. Bearings retain host-parse timing uncertainty. Rejected events have
no trustworthy bearing and are counted separately, not assigned invented angles.
"""
from collections import Counter
import math


def analyze_revolutions(events, *, direction, zero_deg=0.0,
                        max_interval_s=0.1, max_step_deg=10.0,
                        max_possible_rpm=130.0):
    """direction is the measured trend of *yaw_deg*, not a motor CW label.

    Input order is preserved. Do not sort around restarts. Only ``registered``
    events are angular samples; raw missing frames require frames.jsonl, not
    gaps in this filtered event stream. All limits are offline analysis gates.
    """
    if (direction not in (-1, 1) or not math.isfinite(zero_deg)
            or not math.isfinite(max_interval_s) or max_interval_s <= 0
            or not math.isfinite(max_step_deg) or not 0 < max_step_deg < 180
            or not math.isfinite(max_possible_rpm) or max_possible_rpm <= 0):
        raise ValueError('invalid revolution analysis options')
    turns, breaks, rejections = [], [], Counter()
    previous = None
    current = []
    start_bracket = None
    segment = 0
    config_keys = ('registration_direction', 'registration_zero_deg',
                   'registration_mounting_yaw_deg', 'registration_mounting_roll_deg')

    def finish(end_bracket=None, reason='end_of_recording'):
        if not current:
            return
        angles = sorted({round(row['yaw_deg'] % 360, 6) for row in current})
        gaps = [b - a for a, b in zip(angles, angles[1:])]
        gaps.append(angles[0] + 360 - angles[-1])
        turns.append(dict(
            turn_index=len(turns), segment=segment,
            complete=start_bracket is not None and end_bracket is not None,
            end_reason=reason, valid_lines=len(current),
            unique_azimuths=len(angles), max_gap_deg=max(gaps),
            first_frame_num=current[0]['frame_num'],
            last_frame_num=current[-1]['frame_num'],
            first_host_monotonic_s=current[0]['time'],
            last_host_monotonic_s=current[-1]['time'],
            start_crossing_bracket_s=start_bracket,
            end_crossing_bracket_s=end_bracket,
        ))

    for index, event in enumerate(events):
        if event.get('status') != 'registered':
            rejections[str(event.get('reason', event.get('status', 'unknown')))] += 1
            continue
        try:
            time = float(event['host_monotonic_s'])
            yaw = float(event['yaw_deg'])
            frame = int(event['frame_num'])
            if not math.isfinite(time) or not math.isfinite(yaw) or not 0 <= frame < 2**32:
                raise ValueError('invalid sample')
        except (KeyError, TypeError, ValueError, OverflowError):
            finish(reason='invalid_sample')
            breaks.append(dict(event_index=index, reason='invalid_sample'))
            current, previous, start_bracket = [], None, None
            segment += 1
            continue
        phase = (direction * (yaw - zero_deg)) % 360
        row = dict(time=time, yaw_deg=yaw, frame_num=frame, phase=phase,
                   config=tuple(event.get(key) for key in config_keys))
        reason = None
        if previous is not None:
            dt = time - previous['time']
            frame_step = (frame - previous['frame_num']) % 2**32
            step = (phase - previous['phase'] + 180) % 360 - 180
            if dt <= 0:
                reason = 'time_reset'
            elif frame_step > 2**31:
                reason = 'frame_counter_reset'
            elif row['config'] != previous['config']:
                reason = 'registration_changed'
            elif dt > max_interval_s:
                reason = 'time_gap'
            elif dt * 6 * max_possible_rpm >= 180:
                reason = 'ambiguous_rotation'
            elif step < -1e-6:
                reason = 'direction_reversal'
            elif step > max_step_deg:
                reason = 'angular_gap'
            if reason:
                finish(reason=reason)
                breaks.append(dict(event_index=index, frame_num=frame, reason=reason))
                current, previous, start_bracket = [], None, None
                segment += 1
            elif step > 0 and phase < previous['phase']:
                bracket = [previous['time'], time]
                finish(bracket, 'zero_crossing')
                current, start_bracket = [], bracket
        if previous is None:
            start_bracket = [time, time] if phase == 0 else None
        current.append(row)
        previous = row
    finish()
    return dict(turns=turns, breaks=breaks,
                registration_rejections=dict(rejections), raw_missing_frames=None,
                time_basis='host_packet_parse_monotonic', rf_time_calibrated=False,
                direction=direction, zero_deg=zero_deg,
                limits=dict(max_interval_s=max_interval_s, max_step_deg=max_step_deg,
                            max_possible_rpm=max_possible_rpm),
                valid_lines_meaning='registered events; TLV2 presence not checked')
