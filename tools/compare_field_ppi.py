"""Compare two actual PPI captures; report coverage and fixed-target candidates.

Reads recorder artifacts only. No result from this tool calibrates RF time or
establishes that a range-gated peak belongs to the surveyed target.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import median

from tools.analyze_scan_timing import analyze_records, frame_period_from_metadata
from tools.offline_range_axis import diagnose_capture_range_axis
from tools.register_scan_capture import extract_range_profile
from tools.replay_scan_timebases import validate_live_replay_capture, verify_live_replay_profiles


HOST = "host_packet_parse_monotonic"
COUNTER = "frame_counter_x_cfg_period_estimate_only"
SETUP_KEYS = ("motor_direction", "direction", "zero_deg", "mounting_yaw_deg", "mounting_roll_deg")
EVENT_KEYS = dict(direction="registration_direction", zero_deg="registration_zero_deg",
                  mounting_yaw_deg="registration_mounting_yaw_deg",
                  mounting_roll_deg="registration_mounting_roll_deg")
QUALITY_KEYS = ("max_gap_s", "max_query_duration_s", "max_span_deg", "max_possible_rpm", "stale_after_s")
MIN_TURNS = 3


def _delta(angle, reference):
    return (angle - reference + 180) % 360 - 180


def _load_capture(path, clock):
    meta = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
    registration = meta.get("registration") or {}
    if registration.get("time_basis") != clock:
        raise ValueError(f"{path.name}: expected a recorded {clock} run")
    expected_period = frame_period_from_metadata(meta) if clock == COUNTER else None
    if registration.get("experimental_frame_period_ms") != expected_period:
        raise ValueError(f"{path.name}: recorded clock period disagrees with CFG")
    setup = {key: registration[key] for key in SETUP_KEYS}
    if (setup["motor_direction"] not in ("cw", "ccw") or setup["direction"] not in (-1, 1)
            or not all(math.isfinite(setup[key]) for key in SETUP_KEYS[2:])):
        raise ValueError(f"{path.name}: invalid mechanical registration snapshot")

    def records(name):
        with (path / (name + ".jsonl")).open(encoding="utf-8") as source:
            return [json.loads(line) for line in source if line.strip()]

    frames = records("frames")
    samples = records("motor_samples")
    turns = records("turn_diagnostics")
    events = records("live_registration")
    validate_live_replay_capture(meta, frame_count=len(frames), sample_count=len(samples))
    for key, rows in (("turn_diagnostics", turns), ("registration_events", events)):
        if meta.get(key) != len(rows) or meta.get("dropped_" + key) != 0:
            raise ValueError(f"{path.name}: incomplete {key}")
        if any(row.get("time_basis") != clock for row in rows):
            raise ValueError(f"{path.name}: mixed clock in {key}")
    if any(event.get(field) != setup[key] for event in events for key, field in EVENT_KEYS.items()):
        raise ValueError(f"{path.name}: registration changed during capture")
    quality = {tuple(event.get(key) for key in QUALITY_KEYS) for event in events}
    if len(quality) != 1 or not all(
            isinstance(value, (int, float)) and math.isfinite(value) and value > 0
            for value in next(iter(quality))):
        raise ValueError(f"{path.name}: missing or mixed registration quality limits")
    speeds = [abs(float(sample["speed_rpm"])) for sample in samples]
    if not speeds or any(sample.get("enabled") is not True for sample in samples) or not all(
            math.isfinite(speed) and speed > 0 for speed in speeds):
        raise ValueError(f"{path.name}: live motor speed evidence is incomplete or includes a stop")
    return meta, setup, quality.pop(), frames, events, turns, speeds


def _summarize(path, capture, target):
    meta, _, _, frames, events, turns, speeds = capture
    raw_timing = analyze_records(frames)
    if raw_timing["duplicate_frame_numbers"]:
        raise ValueError(f"{path.name}: duplicate frame numbers prevent turn attribution")
    frame_indices = {row["frame_num"]: i for i, row in enumerate(frames)}
    if len(frame_indices) != len(frames):
        raise ValueError(f"{path.name}: reused frame numbers prevent turn attribution")
    by_frame = {}
    for event in events:
        stamp = (event["frame_num"], event["host_monotonic_s"])
        if stamp in by_frame:
            raise ValueError(f"{path.name}: duplicate registration event")
        by_frame[stamp] = event

    results, ignored = [], 0
    last_end = -1
    axis_cache = {}
    with (path / "frames.tlv").open("rb") as raw:
        verified = verify_live_replay_profiles(frames, raw)
        for turn in turns:
            start = frame_indices.get(turn["first_raw_frame_num"])
            end = frame_indices.get(turn["last_raw_frame_num"])
            if start is None or end is None:
                ignored += 1
                continue
            if start > end or start <= last_end:
                raise ValueError(f"{path.name}: overlapping or reversed turn intervals")
            last_end = end
            row = {key: turn[key] for key in ("first_raw_frame_num", "last_raw_frame_num",
                   "max_gap_deg", "valid_tlv2_spokes", "raw_frame_missing", "tlv2_missing",
                   "registration_rejected")}
            peak = None
            first_spoke = frame_indices.get(turn["first_frame_num"])
            last_spoke = frame_indices.get(turn["last_frame_num"])
            if first_spoke is not None and last_spoke is not None:
                for frame in verified[max(start, first_spoke):min(end, last_spoke) + 1]:
                    event = by_frame.get((frame["frame_num"], frame["host_monotonic_s"]))
                    if not frame["range_profile_valid"] or not event or event["status"] != "registered":
                        continue
                    yaw = float(event["yaw_deg"])
                    if not math.isfinite(yaw):
                        raise ValueError(f"{path.name}: nonfinite yaw in registered event")
                    delta = _delta(yaw, target["azimuth_deg"])
                    if abs(delta) >= target["azimuth_gate_deg"]:
                        continue
                    raw.seek(frame["raw_offset"])
                    profile = extract_range_profile(raw.read(frame["raw_length"]))
                    bins = len(profile)
                    if bins not in axis_cache:
                        axis_cache[bins] = diagnose_capture_range_axis(meta, bins)
                    axis = axis_cache[bins]
                    step = axis["range_bin_size_m"]
                    if step is None:
                        raise ValueError(f"{path.name}: no supported theoretical range axis: {axis['reason']}")
                    center, width = target["range_m"], target["range_gate_m"]
                    if center - width <= 0 or center + width > axis["last_tlv2_bin_m"]:
                        raise ValueError(f"{path.name}: target window is outside the supported distance bins")
                    inside, background = [], []
                    for index, value in enumerate(profile):
                        distance = abs(index * step - center)
                        if distance <= width:
                            inside.append((index, value))
                        elif distance <= 3 * width:
                            background.append(value)
                    if not inside or len(background) < 2:
                        continue
                    index, value = max(inside, key=lambda pair: pair[1])
                    contrast = value - median(background)
                    if contrast >= target["min_contrast"] and (peak is None or contrast > peak["contrast_raw"]):
                        peak = dict(frame_num=frame["frame_num"], yaw_deg=yaw % 360,
                                    error_deg=delta, range_m_theoretical=index * step,
                                    range_bin=index, contrast_raw=contrast)
            row["target_candidate"] = peak
            results.append(row)
    biases = [row["target_candidate"]["error_deg"] for row in results if row["target_candidate"]]
    gaps = [row["max_gap_deg"] for row in results]
    return dict(capture=path.name, time_basis=meta["registration"]["time_basis"],
                complete_turns=len(results), ignored_edge_turns=ignored, turns=results,
                raw_missing_frames=raw_timing["missing_frame_numbers"],
                measured_rpm_median=median(speeds), measured_rpm_min=min(speeds),
                measured_rpm_max=max(speeds), target_candidate_turns=len(biases),
                target_bias_deg=median(biases) if len(biases) >= MIN_TURNS else None,
                target_spread_deg=max(biases) - min(biases) if len(biases) >= MIN_TURNS else None,
                max_turn_gap_deg=max(gaps) if len(gaps) >= MIN_TURNS and None not in gaps else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("experiment", type=Path)
    parser.add_argument("--range-m", type=float, required=True, help="Surveyed slant distance")
    parser.add_argument("--azimuth-deg", type=float, required=True, help="Surveyed PPI bearing: 0 up, clockwise positive")
    parser.add_argument("--range-gate-m", type=float, default=0.20, help="Range search half-width")
    parser.add_argument("--azimuth-gate-deg", type=float, default=20, help="Azimuth search half-width")
    parser.add_argument("--min-contrast", type=float, required=True,
                        help="Minimum peak minus neighboring-bin background, raw TLV2 units")
    args = parser.parse_args()
    target = {key: getattr(args, key) for key in (
        "range_m", "azimuth_deg", "range_gate_m", "azimuth_gate_deg", "min_contrast")}
    try:
        if (not all(map(math.isfinite, target.values())) or not 0 < args.range_gate_m < args.range_m
                or not 0 < args.azimuth_gate_deg < 90 or args.min_contrast <= 0):
            raise ValueError("invalid surveyed target or search window")
        left = _load_capture(args.baseline, HOST)
        right = _load_capture(args.experiment, COUNTER)
        if left[0]["active_config"]["sha256"] != right[0]["active_config"]["sha256"]:
            raise ValueError("active CFG differs between captures")
        for key in SETUP_KEYS:
            if left[1][key] != right[1][key]:
                raise ValueError(f"{key} differs between captures")
        if left[2] != right[2]:
            raise ValueError("registration quality limits differ between captures")
        a, b = _summarize(args.baseline, left, target), _summarize(args.experiment, right, target)
        comparable_speed = abs(b["measured_rpm_median"] - a["measured_rpm_median"]) <= (
            .05 * a["measured_rpm_median"])
        gaps = a["max_turn_gap_deg"], b["max_turn_gap_deg"]
        biases = a["target_bias_deg"], b["target_bias_deg"]
        result = dict(baseline=a, experiment=b, survey=target, rf_time_calibrated=False,
                      config_sha256=left[0]["active_config"]["sha256"],
                      comparable_median_speed=comparable_speed,
                      max_gap_change_deg=(gaps[1] - gaps[0] if comparable_speed and None not in gaps else None),
                      target_bias_change_deg=(_delta(biases[1], biases[0])
                                              if comparable_speed and None not in biases else None),
                      interpretation="Range-gated candidates only; confirm the target and tolerances "
                                     "independently. Null means undetermined, not zero offset.")
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
