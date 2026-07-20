"""Safe, dependency-free access to recorded radar captures for offline replay."""

from __future__ import annotations

import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any, Iterator


class CaptureAccessError(ValueError):
    """Raised when a capture request is invalid or its on-disk data is unusable."""


class CaptureCatalog:
    """Read captures under one root without exposing arbitrary filesystem paths."""

    def __init__(self, capture_root: str | Path):
        self._capture_root = Path(capture_root)

    def list_captures(self) -> list[dict[str, Any]]:
        if not self._capture_root.exists():
            return []
        captures = []
        for path in sorted(self._capture_root.glob("capture_*"), reverse=True):
            if path.is_symlink() or not path.is_dir() or path.resolve().parent != self._capture_root.resolve():
                continue
            try:
                metadata = self._read_metadata(path)
                frames = self._count_frames(path)
                active_config = metadata.get("active_config", {})
                captures.append({
                    "id": path.name,
                    "started_at_s": metadata.get("started_at_s"),
                    "active_config": {
                        "name": active_config.get("name"),
                        "sha256": active_config.get("sha256"),
                    } if isinstance(active_config, dict) else {},
                    "frames": frames,
                    "schema_version": metadata.get("schema_version"),
                    "recording_status": metadata.get("recording_status", "unknown"),
                    "dropped_frames": metadata.get("dropped_frames"),
                    "writer_error": metadata.get("writer_error"),
                })
            except (CaptureAccessError, OSError, json.JSONDecodeError):
                captures.append({"id": path.name, "invalid": True, "frames": 0})
        return captures

    def get_frame(self, capture_id: Any, record_index: Any) -> dict[str, Any]:
        capture_dir = self._resolve_capture(capture_id)
        try:
            expected_index = int(record_index)
        except (TypeError, ValueError) as exc:
            raise CaptureAccessError("record index must be an integer") from exc
        for frame in self._iter_frame_records(capture_dir):
            if frame.get("record_index") == expected_index:
                frame["points"] = self._read_points(capture_dir, expected_index)
                return frame
        raise CaptureAccessError("frame not found")

    def get_analysis(self, capture_id: Any) -> dict[str, Any]:
        """Build bounded, display-ready quality metrics for one recorded capture."""
        capture_dir = self._resolve_capture(capture_id)
        metadata = self._read_metadata(capture_dir)
        frames = list(self._iter_frame_records(capture_dir))
        point_metrics = self._point_metrics_by_frame(capture_dir)
        trend = []
        frame_times = []

        for frame in frames:
            record_index = int(frame.get("record_index", -1))
            metrics = point_metrics.get(record_index, _empty_point_metrics())
            host_time_s = _finite_number(frame.get("host_time_s"))
            if host_time_s is not None:
                frame_times.append(host_time_s)
            trend.append({
                "record_index": record_index,
                "frame_num": frame.get("frame_num"),
                "host_time_s": host_time_s,
                "point_count": int(frame.get("point_count") or 0),
                "side_info_count": int(frame.get("side_info_count") or 0),
                "range_mean_m": _mean_or_none(metrics["ranges"]),
                "speed_abs_mean_mps": _mean_or_none(metrics["speed_abs"]),
                "snr_mean_db": _mean_or_none(metrics["snr"]),
                "noise_mean_db": _mean_or_none(metrics["noise"]),
            })

        intervals_ms = [
            (right - left) * 1000.0
            for left, right in zip(frame_times, frame_times[1:])
            if right >= left
        ]
        raw_bytes = sum(int(frame.get("raw_length") or 0) for frame in frames)
        point_counts = [int(frame.get("point_count") or 0) for frame in frames]
        all_metrics = _merge_point_metrics(point_metrics.values())
        duration_s = (frame_times[-1] - frame_times[0]) if len(frame_times) > 1 else 0.0
        active_config = metadata.get("active_config") if isinstance(metadata.get("active_config"), dict) else {}

        return {
            "capture_id": capture_dir.name,
            "summary": {
                "frame_count": len(frames),
                "point_count": sum(point_counts),
                "empty_frame_count": sum(1 for count in point_counts if count == 0),
                "duration_s": duration_s,
                "frames_per_second": (len(frames) - 1) / duration_s if duration_s > 0 else None,
                "points_per_frame": _distribution(point_counts),
                "frame_interval_ms": _distribution(intervals_ms),
                "range_m": _distribution(all_metrics["ranges"]),
                "speed_abs_mps": _distribution(all_metrics["speed_abs"]),
                "snr_db": _distribution(all_metrics["snr"]),
                "noise_db": _distribution(all_metrics["noise"]),
                "raw_bytes": raw_bytes,
                "dropped_frames": metadata.get("dropped_frames", 0),
                "writer_error": metadata.get("writer_error"),
                "recording_status": metadata.get("recording_status", "unknown"),
                "active_config": {
                    "name": active_config.get("name"),
                    "sha256": active_config.get("sha256"),
                },
            },
            "trend": trend,
        }

    def iter_frame_summaries(self, capture_id: Any, *, offset: Any = 0, limit: Any = 200) -> Iterator[dict[str, Any]]:
        capture_dir = self._resolve_capture(capture_id)
        try:
            start = max(0, int(offset))
            maximum = min(200, max(1, int(limit)))
        except (TypeError, ValueError) as exc:
            raise CaptureAccessError("offset and limit must be integers") from exc
        for index, frame in enumerate(self._iter_frame_records(capture_dir)):
            if index < start:
                continue
            if index >= start + maximum:
                break
            yield frame

    def _resolve_capture(self, capture_id: Any) -> Path:
        if not isinstance(capture_id, str) or not capture_id or Path(capture_id).name != capture_id:
            raise CaptureAccessError("invalid capture id")
        root = self._capture_root.resolve()
        candidate = (root / capture_id).resolve()
        if candidate.parent != root or not candidate.is_dir() or not candidate.name.startswith("capture_"):
            raise CaptureAccessError("capture not found")
        return candidate

    @staticmethod
    def _read_metadata(capture_dir: Path) -> dict[str, Any]:
        metadata_path = capture_dir / "metadata.json"
        with metadata_path.open("r", encoding="utf-8") as handle:
            metadata = json.load(handle)
        if not isinstance(metadata, dict):
            raise CaptureAccessError("invalid capture metadata")
        return metadata

    @staticmethod
    def _iter_frame_records(capture_dir: Path) -> Iterator[dict[str, Any]]:
        frames_path = capture_dir / "frames.jsonl"
        with frames_path.open("r", encoding="utf-8") as handle:
            for line in handle:
                if not line.strip():
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    if handle.tell() == frames_path.stat().st_size:
                        break
                    raise
                if not isinstance(record, dict):
                    raise CaptureAccessError("invalid frame record")
                yield record

    def _count_frames(self, capture_dir: Path) -> int:
        return sum(1 for _ in self._iter_frame_records(capture_dir))

    @staticmethod
    def _point_metrics_by_frame(capture_dir: Path) -> dict[int, dict[str, list[float]]]:
        points_path = capture_dir / "points.csv"
        metrics: dict[int, dict[str, list[float]]] = {}
        if not points_path.is_file():
            return metrics
        with points_path.open("r", newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                try:
                    record_index = int(row["record_index"])
                except (KeyError, TypeError, ValueError):
                    continue
                frame_metrics = metrics.setdefault(record_index, _empty_point_metrics())
                x = _finite_number(row.get("x_m"))
                y = _finite_number(row.get("y_m"))
                z = _finite_number(row.get("z_m"))
                velocity = _finite_number(row.get("v_mps"))
                snr = _finite_number(row.get("snr_db"))
                noise = _finite_number(row.get("noise_db"))
                if x is not None and y is not None and z is not None:
                    frame_metrics["ranges"].append(math.sqrt(x * x + y * y + z * z))
                if velocity is not None:
                    frame_metrics["speed_abs"].append(abs(velocity))
                if snr is not None:
                    frame_metrics["snr"].append(snr)
                if noise is not None:
                    frame_metrics["noise"].append(noise)
        return metrics

    @staticmethod
    def _read_points(capture_dir: Path, record_index: int) -> list[dict[str, Any]]:
        points_path = capture_dir / "points.csv"
        if not points_path.is_file():
            return []
        with points_path.open("r", newline="", encoding="utf-8") as handle:
            rows = csv.DictReader(handle)
            return [
                {
                    "x": float(row["x_m"]),
                    "y": float(row["y_m"]),
                    "z": float(row["z_m"]),
                    "v": float(row["v_mps"]),
                    "snr": float(row["snr_db"]) if row["snr_db"] else None,
                    "noise": float(row["noise_db"]) if row["noise_db"] else None,
                }
                for row in rows
                if int(row["record_index"]) == record_index
            ]


def _finite_number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _empty_point_metrics() -> dict[str, list[float]]:
    return {"ranges": [], "speed_abs": [], "snr": [], "noise": []}


def _merge_point_metrics(values: Iterator[dict[str, list[float]]]) -> dict[str, list[float]]:
    merged = _empty_point_metrics()
    for metrics in values:
        for name, points in metrics.items():
            merged[name].extend(points)
    return merged


def _mean_or_none(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def _distribution(values: list[float | int]) -> dict[str, float | None]:
    if not values:
        return {"mean": None, "median": None, "p95": None, "min": None, "max": None}
    ordered = sorted(float(value) for value in values)
    p95_index = min(len(ordered) - 1, math.ceil(len(ordered) * 0.95) - 1)
    return {
        "mean": statistics.fmean(ordered),
        "median": statistics.median(ordered),
        "p95": ordered[p95_index],
        "min": ordered[0],
        "max": ordered[-1],
    }
