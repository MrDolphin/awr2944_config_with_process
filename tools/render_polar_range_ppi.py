"""Render an offline diagnostic PPI from registered firmware TLV2 range profiles.

Only observed spokes are drawn. The radial axis is the uncalibrated firmware
bin index; host packet-parse time is not the radar RF acquisition time.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


MAX_SPOKES = 2000
MAX_BINS = 4096
MAX_CELLS = 1_000_000


def _load_profiles(path: Path) -> list[dict]:
    rows = []
    seen_indices = set()
    bin_count = None
    with path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"line {line_number}: invalid JSON") from exc
            if not isinstance(row, dict):
                raise ValueError(f"line {line_number}: expected an object")
            index = row.get("record_index")
            yaw = row.get("yaw_deg")
            timestamp = row.get("host_monotonic_s")
            bins = row.get("range_bins_raw")
            if not isinstance(index, int) or isinstance(index, bool) or index in seen_indices:
                raise ValueError(f"line {line_number}: duplicate or invalid record_index")
            seen_indices.add(index)
            if (not isinstance(yaw, (int, float)) or isinstance(yaw, bool)
                    or not math.isfinite(yaw) or not 0 <= yaw < 360):
                raise ValueError(f"line {line_number}: yaw_deg must be in [0, 360)")
            if (not isinstance(timestamp, (int, float)) or isinstance(timestamp, bool)
                    or not math.isfinite(timestamp)):
                raise ValueError(f"line {line_number}: invalid host_monotonic_s")
            if row.get("range_axis") != "bin_index_uncalibrated":
                raise ValueError(f"line {line_number}: unsupported or unverified range axis")
            if row.get("value_units") != "firmware_raw_uint16":
                raise ValueError(f"line {line_number}: unsupported intensity units")
            if row.get("time_basis") != "host_packet_parse_monotonic":
                raise ValueError(f"line {line_number}: unsupported time basis")
            if not isinstance(bins, list) or not bins or len(bins) > MAX_BINS:
                raise ValueError(f"line {line_number}: invalid range_bins_raw length")
            if any(not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 65535
                   for value in bins):
                raise ValueError(f"line {line_number}: range bin is not uint16")
            if bin_count is None:
                bin_count = len(bins)
            elif len(bins) != bin_count:
                raise ValueError(f"line {line_number}: inconsistent range-bin count")
            rows.append(row)
            if len(rows) > MAX_SPOKES or len(rows) * bin_count > MAX_CELLS:
                raise ValueError("capture exceeds diagnostic renderer size limit; split by time")
    if not rows:
        raise ValueError("no registered range profiles found")
    return rows


def _largest_gap_deg(angles: list[float]) -> float:
    ordered = sorted(set(angles))
    return round(max((ordered[(i + 1) % len(ordered)] - angle) % 360 or 360
                     for i, angle in enumerate(ordered)), 3)


def _svg(rows: list[dict], skip_bins: int) -> tuple[str, int]:
    center = 400
    radius = 350
    bin_count = len(rows[0]["range_bins_raw"])
    positive = sorted(value for row in rows for value in row["range_bins_raw"][skip_bins:]
                      if value > 0)
    upper = positive[min(len(positive) - 1, int(len(positive) * 0.99))] if positive else 1
    top = max(1.0, math.log1p(upper))
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 800 840" '
        'role="img" aria-label="Diagnostic observed-spoke intensity PPI">',
        '<rect width="800" height="840" fill="#031018"/>',
        '<circle cx="400" cy="400" r="350" fill="none" stroke="#145651"/>',
    ]
    for fraction in (0.25, 0.5, 0.75):
        parts.append(f'<circle cx="400" cy="400" r="{radius * fraction:.2f}" '
                     'fill="none" stroke="#145651" stroke-opacity="0.55"/>')
    for spoke_index, row in enumerate(rows):
        angle = math.radians(row["yaw_deg"] - 90)
        cosine, sine = math.cos(angle), math.sin(angle)
        parts.append(f'<g data-spoke="{spoke_index}" data-yaw-deg="{row["yaw_deg"]:.3f}">')
        for bin_index in range(skip_bins, bin_count):
            value = row["range_bins_raw"][bin_index]
            if value == 0:
                continue
            brightness = min(1.0, math.log1p(value) / top)
            green = 35 + round(brightness * 220)
            blue = round(brightness * 85)
            inner = radius * bin_index / bin_count
            outer = radius * (bin_index + 1) / bin_count
            parts.append(
                f'<line x1="{center + inner * cosine:.2f}" y1="{center + inner * sine:.2f}" '
                f'x2="{center + outer * cosine:.2f}" y2="{center + outer * sine:.2f}" '
                f'stroke="#{0:02x}{green:02x}{blue:02x}" stroke-width="1.5"/>'
            )
        parts.append("</g>")
    parts.extend([
        '<circle cx="400" cy="400" r="3" fill="#4affd5"/>',
        '<text x="25" y="795" fill="#c4f4e9" font-size="17">'
        'Diagnostic PPI: raw firmware intensity; radial axis = uncalibrated bin index</text>',
        '<text x="25" y="820" fill="#9ab8b2" font-size="14">'
        'Host parse time, not RF time; unobserved sectors remain blank</text>',
        '</svg>',
    ])
    return "\n".join(parts) + "\n", upper


def render_ppi(profiles_jsonl: Path, output_dir: Path, *, skip_bins: int = 0) -> dict:
    """Write an SVG and evidence summary to a new directory; never alter capture files."""
    profiles_jsonl = Path(profiles_jsonl)
    output_dir = Path(output_dir)
    rows = _load_profiles(profiles_jsonl)
    if not isinstance(skip_bins, int) or not 0 <= skip_bins < len(rows[0]["range_bins_raw"]):
        raise ValueError("skip_bins must leave at least one observed range bin")
    svg, upper = _svg(rows, skip_bins)
    angles = [float(row["yaw_deg"]) for row in rows]
    times = [float(row["host_monotonic_s"]) for row in rows]
    summary = {
        "input_path": str(profiles_jsonl.resolve()),
        "input_sha256": hashlib.sha256(profiles_jsonl.read_bytes()).hexdigest(),
        "spoke_count": len(rows),
        "range_bin_count": len(rows[0]["range_bins_raw"]),
        "skipped_near_bins": skip_bins,
        "observed_10deg_bins": sorted({int(angle // 10) for angle in angles}),
        "largest_angular_gap_deg": _largest_gap_deg(angles),
        "host_parse_time_span_s": round(max(times) - min(times), 6),
        "normalization_p99_raw": upper,
        "range_axis": "bin_index_uncalibrated",
        "value_units": "firmware_raw_uint16",
        "time_basis": "host_packet_parse_monotonic",
        "rf_time_calibrated": False,
        "physical_coverage_verified": False,
        "unobserved_bearings_filled": False,
        "interpretation": "Diagnostic observed-spoke image only; no calibrated range, RCS, RF timing, or beamwidth claim.",
    }
    output_dir.mkdir(parents=True, exist_ok=False)
    (output_dir / "intensity_ppi.svg").write_text(svg, encoding="utf-8")
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profiles_jsonl", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--skip-bins", type=int, default=0,
                        help="Hide near-range leakage bins without modifying the raw file")
    args = parser.parse_args(argv)
    summary = render_ppi(args.profiles_jsonl, args.output_dir, skip_bins=args.skip_bins)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
