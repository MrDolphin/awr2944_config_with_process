"""Offline fixed-zero turns from registered bearings; never controls hardware.

Completeness means two observed zero-crossing brackets, NOT full angular
coverage. Bearings retain host-parse timing uncertainty. Rejected events have
no trustworthy bearing and are counted separately, not assigned invented angles.
"""
from collections import Counter
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile


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
            event_indices=[row['event_index'] for row in current],
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
                   event_index=index,
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


def analyze_capture_records(frames, events, *, direction, **options):
    """Join recorded frame identities to registered events; no inferred spokes.

    Events outside the raw recording are common at its edges. Match both frame
    number and host timestamp so a counter reuse cannot create a false match.
    """
    result = analyze_revolutions(events, direction=direction, **options)
    raw = {}
    missing = 0
    previous = None
    for frame in frames:
        number = int(frame['frame_num'])
        stamp = float(frame['host_monotonic_s'])
        if not 0 <= number < 2**32 or not math.isfinite(stamp):
            raise ValueError('invalid raw frame identity')
        key = (number, stamp)
        if key in raw:
            raise ValueError('duplicate raw frame identity')
        raw[key] = frame
        if previous is not None:
            step = (number - previous) % 2**32
            if step == 0 or step > 2**31:
                raise ValueError('raw frame counter discontinuity')
            missing += step - 1
        previous = number
    unmatched = 0
    for turn in result['turns']:
        matched = tlv2 = absent = 0
        observed = []
        indices = turn.pop('event_indices')
        for index in indices:
            event = events[index]
            frame = raw.get((event['frame_num'], event['host_monotonic_s']))
            if frame is None:
                unmatched += 1
                continue
            matched += 1
            if 2 in frame['tlv_types']:
                tlv2 += 1
                observed.append((round(float(event['yaw_deg']) % 360, 6), event))
            else:
                absent += 1
        turn.update(matched_raw_frames=matched, tlv2_lines=tlv2,
                    missing_tlv2=absent)
        # Only a recorded TLV2 profile is a real intensity spoke. A rejected
        # registration has no trustworthy angle, so locate it by host time
        # between observed endpoints rather than assigning an invented bearing.
        observed = sorted(((round((direction * (angle - result['zero_deg'])) % 360, 6), event)
                           for angle, event in observed), key=lambda row: row[0])
        if not observed:
            turn.update(max_gap_deg=None, max_gap_start_deg=None,
                        max_gap_end_deg=None,
                        max_gap_evidence=dict(classification='unobserved',
                                              raw_frame_missing=0, tlv2_missing=0,
                                              registration_rejected=0),
                        max_gap_classification='unobserved')
            continue
        gaps = [(b[0] - a[0], a, b, False)
                for a, b in zip(observed, observed[1:])]
        gaps.append((observed[0][0] + 360 - observed[-1][0],
                     observed[-1], observed[0], True))
        gap, start, end, wraps = max(gaps, key=lambda item: item[0])
        evidence = dict(raw_frame_missing=0, tlv2_missing=0,
                        registration_rejected=0, classification='unobserved')
        start_time = float(start[1]['host_monotonic_s'])
        end_time = float(end[1]['host_monotonic_s'])
        if not wraps and start_time < end_time:
            interval_frames = [frame for frame in frames
                               if start_time <= float(frame['host_monotonic_s']) <= end_time]
            for left, right in zip(interval_frames, interval_frames[1:]):
                step = (int(right['frame_num']) - int(left['frame_num'])) % 2**32
                evidence['raw_frame_missing'] += max(0, step - 1)
            evidence['tlv2_missing'] = sum(2 not in frame['tlv_types']
                                           for frame in interval_frames[1:-1])
            evidence['registration_rejected'] = sum(
                event.get('status') != 'registered'
                for event in events
                if start_time < float(event.get('host_monotonic_s', -math.inf)) < end_time)
            causes = [key for key in ('raw_frame_missing', 'tlv2_missing',
                                      'registration_rejected') if evidence[key]]
            evidence['classification'] = causes[0] if len(causes) == 1 else (
                'undetermined' if causes else 'unobserved')
        else:
            evidence['classification'] = 'undetermined'
        turn.update(max_gap_deg=gap, max_gap_start_deg=start[0],
                    max_gap_end_deg=end[0], max_gap_evidence=evidence,
                    max_gap_classification=evidence['classification'],
                    max_gap_start_frame_num=int(start[1]['frame_num']),
                    max_gap_end_frame_num=int(end[1]['frame_num']))
    result.update(raw_missing_frames=missing,
                  raw_tlv2_frames=sum(2 in frame['tlv_types'] for frame in frames),
                  unmatched_registration_events=unmatched,
                  valid_lines_meaning='registered events; use tlv2_lines for recorded TLV2 spokes')
    return result


def write_capture_report(capture_dir, output_dir, *, max_step_deg=10.0):
    """Read one immutable capture and create one exclusive report directory."""
    capture = Path(capture_dir).resolve()
    output = Path(output_dir).resolve()
    if not capture.is_dir() or output == capture or capture in output.parents:
        raise ValueError('input must be a capture directory; output must be outside it')
    if output.exists():
        raise FileExistsError(f'output already exists: {output}')
    metadata = json.loads((capture / 'metadata.json').read_text(encoding='utf-8'))
    config = metadata['active_config']
    content = config['content']
    digest = hashlib.sha256(content.encode('utf-8')).hexdigest()
    if digest != config['sha256']:
        raise ValueError('captured CFG SHA-256 mismatch; report not written')

    def jsonl(name):
        with (capture / name).open(encoding='utf-8') as source:
            return [json.loads(line) for line in source if line.strip()]

    frames = jsonl('frames.jsonl')
    events = jsonl('live_registration.jsonl')
    registration = metadata['registration']
    result = analyze_capture_records(
        frames, events, direction=registration['direction'],
        zero_deg=registration.get('zero_deg', 0.0), max_step_deg=max_step_deg)
    result['config'] = {'name': config['name'], 'sha256': digest}
    result['capture'] = capture.name
    result['recording'] = {
        key: metadata.get(key) for key in ('recording_status', 'writer_error',
                                            'dropped_frames', 'dropped_registration_events')}
    report_json = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    columns = ('turn_index', 'segment', 'complete', 'end_reason', 'valid_lines',
               'matched_raw_frames', 'tlv2_lines', 'missing_tlv2',
               'unique_azimuths', 'max_gap_deg', 'max_gap_start_deg',
               'max_gap_end_deg', 'max_gap_classification',
               'max_gap_start_frame_num',
               'max_gap_end_frame_num', 'first_frame_num', 'last_frame_num')
    lines = [f"# 逐圈诊断：{capture.name}", '',
             f"CFG：`{config['name']}` · SHA-256 `{digest}`", '',
             f"原始缺帧：{result['raw_missing_frames']}；原始 TLV2 帧：{result['raw_tlv2_frames']}；"
             f"未匹配录制边缘事件：{result['unmatched_registration_events']}",
             f"配准拒绝：{json.dumps(result['registration_rejections'], ensure_ascii=False)}", '',
             '完整圈仅指起止零位边界被连续观测；不代表无角度缺口。',
             '方位采用 host 主机解包时间配准，非 RF 采集时间；本报告不是目标探测或远距性能验证。',
             '缺口归因仅依据区间内原始帧号、TLV2 与配准事件；无证据时写“未判定”。',
             '', '| 圈 | 完整 | TLV2线 | 最大缺口(°) | 角度区间 | 证据 | 断点原因 |',
             '| ---: | :---: | ---: | ---: | --- | --- | --- |']
    labels = {'raw_frame_missing': '原始帧缺失', 'tlv2_missing': 'TLV2 缺失',
              'registration_rejected': '配准拒绝', 'unobserved': '尚未采样（原因未判定）',
              'undetermined': '未判定'}
    for turn in result['turns']:
        gap = turn['max_gap_deg']
        interval = (f"{turn['max_gap_start_deg']:.2f}°→{turn['max_gap_end_deg']:.2f}°"
                    if gap is not None else '无有效 TLV2 方位线')
        evidence = turn['max_gap_evidence']
        detail = (f"{labels[evidence['classification']]} "
                  f"(原始缺帧:{evidence['raw_frame_missing']} / "
                  f"TLV2缺失:{evidence['tlv2_missing']} / "
                  f"配准拒绝:{evidence['registration_rejected']}; "
                  f"端点帧:{turn.get('max_gap_start_frame_num', '无')}→"
                  f"{turn.get('max_gap_end_frame_num', '无')})")
        gap_text = f'{gap:.2f}' if gap is not None else '—'
        lines.append(f"| {turn['turn_index']} | {'是' if turn['complete'] else '否'} | "
                     f"{turn['tlv2_lines']} | {gap_text} | "
                     f"{interval} | {detail} | "
                     f"{turn['end_reason']} |")
    # Render all files in a sibling staging directory first. Claim the final
    # path exclusively; COMPLETE is the last publication step for readers.
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f'.{output.name}.staging-',
                                     dir=output.parent) as temporary:
        staging = Path(temporary)
        (staging / 'report.json').write_text(report_json, encoding='utf-8')
        with (staging / 'report.csv').open('w', encoding='utf-8-sig', newline='') as sink:
            writer = csv.DictWriter(sink, fieldnames=columns, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(result['turns'])
        (staging / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
        output.mkdir(exist_ok=False)
        published = []
        try:
            for name in ('report.json', 'report.csv', 'report.md'):
                target = output / name
                os.link(staging / name, target)
                published.append(target)
            (output / 'COMPLETE').write_text('report files published\n', encoding='utf-8')
        except BaseException:
            (output / 'COMPLETE').unlink(missing_ok=True)
            for target in published:
                target.unlink(missing_ok=True)
            output.rmdir()  # only remove the new, empty directory we own
            raise
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Offline fixed-zero TLV2 revolution report')
    parser.add_argument('capture_dir')
    parser.add_argument('output_dir', help='new directory outside the raw capture')
    parser.add_argument('--max-step-deg', type=float, default=10.0)
    args = parser.parse_args()
    try:
        write_capture_report(args.capture_dir, args.output_dir,
                             max_step_deg=args.max_step_deg)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
