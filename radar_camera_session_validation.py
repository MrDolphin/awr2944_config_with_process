"""Validate a completed radar-camera recording package without hardware access."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


REQUIRED_METADATA_FIELDS = (
    "recording_status",
    "clock_basis",
    "git",
    "radar_config",
    "camera_config",
    "sync_thresholds_ms",
)
REQUIRED_INDEX_COLUMNS = (
    "radar_frame_num",
    "camera_frame_id",
    "time_offset_ms",
    "sync_status",
)


def validate_session(
    session_dir: Path,
    *,
    min_matched_ratio: float = 0.95,
    max_absolute_offset_p95_ms: float = 50.0,
) -> dict[str, Any]:
    """Return integrity errors and Task 11 synchronization acceptance metrics.

    This only validates persisted evidence.  It deliberately does not claim a
    camera frame rate, radar packet-loss result, calibration accuracy, or a
    gimbal result, because none of those facts are present in a session index.
    """
    session_dir = Path(session_dir)
    errors: list[str] = []
    metadata = _load_metadata(session_dir / "session_metadata.json", errors)
    _validate_metadata(metadata, errors)
    radar_rows = _load_radar_rows(session_dir / "radar_frames.jsonl", errors)
    index_rows = _load_index_rows(session_dir / "fusion_index.csv", errors)
    _validate_references(session_dir, radar_rows, index_rows, errors)

    matched_rows = [row for row in index_rows if row.get("sync_status") == "matched"]
    offsets = [_absolute_offset(row, errors) for row in matched_rows]
    valid_offsets = [offset for offset in offsets if offset is not None]
    total_rows = len(index_rows)
    matched_ratio = len(matched_rows) / total_rows if total_rows else 0.0
    p95 = _percentile_nearest_rank(valid_offsets, 0.95)

    return {
        "session_dir": str(session_dir),
        "errors": errors,
        "metrics": {
            "radar_rows": len(radar_rows),
            "index_rows": total_rows,
            "matched_rows": len(matched_rows),
            "matched_ratio": matched_ratio,
            "absolute_offset_p95_ms": p95,
        },
        "thresholds": {
            "min_matched_ratio": min_matched_ratio,
            "max_absolute_offset_p95_ms": max_absolute_offset_p95_ms,
        },
        "acceptance": {
            "matched_ratio": _gate(matched_ratio, min_matched_ratio, higher_is_better=True),
            "absolute_offset_p95_ms": _gate(p95, max_absolute_offset_p95_ms, higher_is_better=False),
        },
        "limitations": [
            "Offline validation does not establish camera frame rate, radar packet loss, calibration accuracy, or gimbal behavior.",
            "Sync offsets use the persisted Pi receive-time clock, not a hardware trigger clock.",
        ],
    }


def _load_metadata(path: Path, errors: list[str]) -> dict[str, Any]:
    if not path.is_file():
        errors.append("Missing session_metadata.json")
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        errors.append(f"Cannot read session_metadata.json: {exc}")
        return {}
    if not isinstance(value, dict):
        errors.append("session_metadata.json must contain a JSON object")
        return {}
    return value


def _validate_metadata(metadata: dict[str, Any], errors: list[str]) -> None:
    for field in REQUIRED_METADATA_FIELDS:
        if field not in metadata:
            errors.append(f"Missing required metadata field: {field}")
    if metadata.get("recording_status") != "completed":
        errors.append("Session recording_status must be completed")


def _load_radar_rows(path: Path, errors: list[str]) -> list[dict[str, Any]]:
    if not path.is_file():
        errors.append("Missing radar_frames.jsonl")
        return []
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            errors.append(f"Invalid radar_frames.jsonl line {line_number}: {exc}")
            continue
        if not isinstance(value, dict):
            errors.append(f"radar_frames.jsonl line {line_number} must contain a JSON object")
            continue
        rows.append(value)
    if not rows:
        errors.append("radar_frames.jsonl contains no radar rows")
    return rows


def _load_index_rows(path: Path, errors: list[str]) -> list[dict[str, str]]:
    if not path.is_file():
        errors.append("Missing fusion_index.csv")
        return []
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            fields = reader.fieldnames or []
            for field in REQUIRED_INDEX_COLUMNS:
                if field not in fields:
                    errors.append(f"fusion_index.csv missing required column: {field}")
            return list(reader)
    except OSError as exc:
        errors.append(f"Cannot read fusion_index.csv: {exc}")
        return []


def _validate_references(
    session_dir: Path,
    radar_rows: list[dict[str, Any]],
    index_rows: list[dict[str, str]],
    errors: list[str],
) -> None:
    radar_numbers = {str(row.get("frame_num")) for row in radar_rows if row.get("frame_num") is not None}
    if len(index_rows) != len(radar_rows):
        errors.append(f"fusion_index.csv has {len(index_rows)} rows but radar_frames.jsonl has {len(radar_rows)} rows")
    seen_index_numbers: set[str] = set()
    referenced_camera_ids: set[str] = set()
    for row_number, row in enumerate(index_rows, start=2):
        frame_number = (row.get("radar_frame_num") or "").strip()
        if not frame_number:
            errors.append(f"fusion_index.csv row {row_number} has no radar_frame_num")
        elif frame_number in seen_index_numbers:
            errors.append(f"fusion_index.csv repeats radar_frame_num {frame_number}")
        elif frame_number not in radar_numbers:
            errors.append(f"fusion_index.csv references absent radar_frame_num {frame_number}")
        seen_index_numbers.add(frame_number)

        camera_id = (row.get("camera_frame_id") or "").strip()
        if camera_id:
            referenced_camera_ids.add(camera_id)
    for camera_id in sorted(referenced_camera_ids, key=_natural_key):
        image = session_dir / "camera_frames" / f"{camera_id}.jpg"
        if not image.is_file():
            errors.append(f"Missing referenced camera frame: camera_frames/{camera_id}.jpg")


def _absolute_offset(row: dict[str, str], errors: list[str]) -> float | None:
    raw = (row.get("time_offset_ms") or "").strip()
    if not raw:
        errors.append(f"Matched radar_frame_num {row.get('radar_frame_num') or '?'} has no time_offset_ms")
        return None
    try:
        offset = abs(float(raw))
    except ValueError:
        errors.append(f"Matched radar_frame_num {row.get('radar_frame_num') or '?'} has invalid time_offset_ms {raw!r}")
        return None
    if not math.isfinite(offset):
        errors.append(f"Matched radar_frame_num {row.get('radar_frame_num') or '?'} has non-finite time_offset_ms")
        return None
    return offset


def _percentile_nearest_rank(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[math.ceil(percentile * len(ordered)) - 1]


def _gate(value: float | None, threshold: float, *, higher_is_better: bool) -> str:
    if value is None:
        return "not_evaluated"
    passed = value >= threshold if higher_is_better else value <= threshold
    return "pass" if passed else "fail"


def _natural_key(value: str) -> tuple[int, str]:
    try:
        return (0, f"{int(value):020d}")
    except ValueError:
        return (1, value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--min-matched-ratio", type=float, default=0.95)
    parser.add_argument("--max-absolute-offset-p95-ms", type=float, default=50.0)
    args = parser.parse_args()
    report = validate_session(
        args.session_dir,
        min_matched_ratio=args.min_matched_ratio,
        max_absolute_offset_p95_ms=args.max_absolute_offset_p95_ms,
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if report["errors"]:
        return 2
    return 0 if all(value == "pass" for value in report["acceptance"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
