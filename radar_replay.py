"""Safe, dependency-free access to recorded radar captures for offline replay."""

from __future__ import annotations

import csv
import json
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
