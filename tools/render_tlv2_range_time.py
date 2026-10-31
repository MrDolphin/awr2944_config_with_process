"""Inspect TLV2 intensity in a recorded capture without inventing azimuth data.

The output is a range-bin by frame heatmap, not a 360-degree PPI. It reads only
recorded packets and does not access serial ports or motor controls.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.register_scan_capture import extract_range_profile
from tools.render_polar_range_ppi import MAX_BINS, MAX_CELLS, MAX_SPOKES
from tools.render_polar_range_ppi import _contrast_limits, _intensity_color


def _load_profiles(capture_dir: Path) -> tuple[list[list[int]], list[dict]]:
    frame_path = capture_dir / "frames.jsonl"
    raw_path = capture_dir / "frames.tlv"
    frames = [json.loads(line) for line in frame_path.read_text(encoding="utf-8").splitlines()
              if line.strip()]
    if not frames:
        raise ValueError("capture has no frames")
    if len(frames) > MAX_SPOKES:
        raise ValueError("capture exceeds diagnostic renderer frame limit; split by time")
    profiles = []
    included = []
    bin_count = None
    raw_size = raw_path.stat().st_size
    with raw_path.open("rb") as raw:
        for expected_index, frame in enumerate(frames):
            if frame.get("record_index") != expected_index:
                raise ValueError("record_index must be contiguous from zero")
            offset, length = frame.get("raw_offset"), frame.get("raw_length")
            if (not isinstance(offset, int) or not isinstance(length, int)
                    or offset < 0 or length < 40 or offset + length > raw_size):
                raise ValueError(f"frame {expected_index}: invalid raw offset/length")
            raw.seek(offset)
            try:
                profile = extract_range_profile(raw.read(length))
            except ValueError as exc:
                raise ValueError(f"frame {expected_index}: {exc}") from exc
            if profile is None:
                continue
            if not profile or len(profile) > MAX_BINS:
                raise ValueError(f"frame {expected_index}: invalid range-bin count")
            if bin_count is None:
                bin_count = len(profile)
            elif len(profile) != bin_count:
                raise ValueError(f"frame {expected_index}: inconsistent range-bin count")
            profiles.append(profile)
            included.append(frame)
            if len(profiles) * bin_count > MAX_CELLS:
                raise ValueError("capture exceeds diagnostic renderer cell limit; split by time")
    if not profiles:
        raise ValueError("capture has no TLV2 range profiles")
    return profiles, included


def _svg(profiles: list[list[int]], lower: int, upper: int) -> str:
    left, top, plot_width, row_height = 64, 55, 1024, 12
    height = top + len(profiles) * row_height + 94
    cell_width = plot_width / len(profiles[0])
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1100 {height}" '
        'role="img" aria-label="Recorded TLV2 range-bin by frame intensity heatmap">',
        f'<rect width="1100" height="{height}" fill="#031018"/>',
        '<text x="64" y="28" fill="#c4f4e9" font-size="18">'
        'Recorded TLV2 intensity: range bin (horizontal) x frame (vertical)</text>',
    ]
    for frame_index, profile in enumerate(profiles):
        y = top + frame_index * row_height
        for bin_index, value in enumerate(profile):
            x = left + bin_index * cell_width
            parts.append(f'<rect x="{x:.2f}" y="{y}" width="{cell_width + 0.02:.2f}" '
                         f'height="{row_height}" fill="{_intensity_color(value, lower, upper)}"/>')
    bottom = top + len(profiles) * row_height
    parts.append(f'<rect x="{left}" y="{top}" width="{plot_width}" height="{bottom-top}" '
                 'fill="none" stroke="#145651"/>')
    for fraction in (0, 0.25, 0.5, 0.75, 1):
        x = left + plot_width * fraction
        bin_label = round(len(profiles[0]) * fraction)
        parts.append(f'<text x="{x:.1f}" y="{bottom+24}" fill="#9ab8b2" '
                     f'font-size="13" text-anchor="middle">{bin_label}</text>')
    parts.extend([
        f'<text x="64" y="{bottom+49}" fill="#c4f4e9" font-size="14">'
        f'Display only: linear contrast p05={lower}, p99={upper}; no intensity gate</text>',
        f'<text x="64" y="{bottom+70}" fill="#9ab8b2" font-size="14">'
        'Range is uncalibrated bin index; no motor azimuth, no PPI</text>',
        '</svg>',
    ])
    return "\n".join(parts) + "\n"


def render_capture(capture_dir: Path, output_dir: Path) -> dict:
    """Write display-only diagnostics to a new directory; preserve the capture."""
    capture_dir, output_dir = Path(capture_dir), Path(output_dir)
    if output_dir.exists():
        raise FileExistsError(f"output already exists: {output_dir}")
    profiles, included = _load_profiles(capture_dir)
    values = [value for profile in profiles for value in profile]
    lower, upper = _contrast_limits(values)
    motor_path = capture_dir / "motor_samples.jsonl"
    motor_samples = 0
    if motor_path.exists():
        with motor_path.open(encoding="utf-8") as source:
            motor_samples = sum(bool(line.strip()) for line in source)
    summary = {
        "capture_path": str(capture_dir.resolve()),
        "frames_jsonl_sha256": hashlib.sha256((capture_dir / "frames.jsonl").read_bytes()).hexdigest(),
        "frames_tlv_sha256": hashlib.sha256((capture_dir / "frames.tlv").read_bytes()).hexdigest(),
        "profile_frames": len(profiles),
        "range_bin_count": len(profiles[0]),
        "frame_num_start": included[0]["frame_num"],
        "frame_num_end": included[-1]["frame_num"],
        "motor_samples": motor_samples,
        "ppi_generated": False,
        "display_lower_raw": lower,
        "display_upper_raw": upper,
        "display_transform": "linear_clipped_p05_p99",
        "intensity_filter_applied": False,
        "range_axis": "bin_index_uncalibrated",
        "value_units": "firmware_raw_uint16",
        "interpretation": "Recorded TLV2 display only; no calibrated range, target detection, azimuth, or PPI claim.",
    }
    svg = _svg(profiles, lower, upper)
    output_dir.mkdir(parents=True, exist_ok=False)
    (output_dir / "range_time_contrast.svg").write_text(svg, encoding="utf-8")
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args(argv)
    print(json.dumps(render_capture(args.capture_dir, args.output_dir), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
