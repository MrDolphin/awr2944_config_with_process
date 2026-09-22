"""Portable, bounded artifacts for one radar-camera synchronization session."""

from __future__ import annotations

import csv
import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Mapping

from tools.camera.camera_capture import CameraFrame


FUSION_INDEX_COLUMNS = (
    "radar_frame_num",
    "radar_monotonic_ns",
    "camera_frame_id",
    "camera_monotonic_ns",
    "time_offset_ms",
    "sync_status",
    "yaw_deg",
    "pitch_deg",
    "pose_age_ms",
)


class RadarCameraSessionWriter:
    """Write one self-contained synchronization session without opening hardware.

    The writer records each radar message and its compact index row.  It copies a
    JPEG at most once per camera frame ID, so repeated radar matches do not turn
    a single image into an unbounded duplicate stream.
    """

    def __init__(self, *, clock=time.time, monotonic_ns=time.monotonic_ns) -> None:
        self._clock = clock
        self._monotonic_ns = monotonic_ns
        self._lock = threading.RLock()
        self._session_dir: Path | None = None
        self._metadata_path: Path | None = None
        self._radar_file = None
        self._index_file = None
        self._index_writer = None
        self._written_camera_ids: set[int] = set()
        self._frames = 0
        self._camera_frames = 0
        self._started_monotonic_ns: int | None = None
        self._last_flush_monotonic_ns: int | None = None

    def start(self, root: Path, metadata: dict[str, object]) -> Path:
        with self._lock:
            if self._session_dir is not None:
                return self._session_dir

            wall_time = self._clock()
            timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(wall_time))
            session_dir = Path(root) / f"radar_camera_{timestamp}"
            suffix = 1
            while session_dir.exists():
                session_dir = Path(root) / f"radar_camera_{timestamp}_{suffix:02d}"
                suffix += 1

            session_dir.mkdir(parents=True)
            (session_dir / "camera_frames").mkdir()
            self._metadata_path = session_dir / "session_metadata.json"
            self._radar_file = (session_dir / "radar_frames.jsonl").open("w", encoding="utf-8")
            self._index_file = (session_dir / "fusion_index.csv").open("w", newline="", encoding="utf-8")
            self._index_writer = csv.DictWriter(self._index_file, fieldnames=FUSION_INDEX_COLUMNS)
            self._index_writer.writeheader()
            now_monotonic_ns = int(self._monotonic_ns())
            self._session_dir = session_dir
            self._written_camera_ids.clear()
            self._frames = 0
            self._camera_frames = 0
            self._started_monotonic_ns = now_monotonic_ns
            self._last_flush_monotonic_ns = now_monotonic_ns
            self._write_metadata({
                "schema_version": 1,
                "recording_status": "active",
                "clock_basis": "pi_receive_monotonic",
                "radar_timestamp_note": "Radar timestamps are receive-time unless hardware timestamps are available.",
                "started_wall_time_s": wall_time,
                "started_monotonic_ns": now_monotonic_ns,
                **dict(metadata),
            })
            self._flush()
            return session_dir

    def append(
        self,
        radar_message: dict[str, object],
        camera_frame: CameraFrame | None,
        pose: object | None = None,
    ) -> None:
        with self._lock:
            if self._session_dir is None or self._radar_file is None or self._index_writer is None:
                return

            sync = radar_message.get("camera_sync")
            sync_data = dict(sync) if isinstance(sync, Mapping) else {}
            camera_id = _optional_int(sync_data.get("frame_id"))
            if camera_frame is not None and camera_id == camera_frame.frame_id:
                self._write_camera_frame(camera_frame)

            radar_monotonic_ns = _radar_monotonic_ns(radar_message)
            self._radar_file.write(json.dumps(radar_message, ensure_ascii=False, separators=(",", ":")) + "\n")
            self._index_writer.writerow({
                "radar_frame_num": _csv_value(_optional_int(radar_message.get("frame_num"))),
                "radar_monotonic_ns": _csv_value(radar_monotonic_ns),
                "camera_frame_id": _csv_value(camera_id),
                "camera_monotonic_ns": _csv_value(_optional_int(sync_data.get("capture_monotonic_ns"))),
                "time_offset_ms": _csv_value(_optional_float(sync_data.get("time_offset_ms"))),
                "sync_status": sync_data.get("status") or "unavailable",
                "yaw_deg": _pose_value(pose, "yaw_deg"),
                "pitch_deg": _pose_value(pose, "pitch_deg"),
                "pose_age_ms": _pose_value(pose, "pose_age_ms"),
            })
            self._frames += 1
            if int(self._monotonic_ns()) - (self._last_flush_monotonic_ns or 0) >= 1_000_000_000:
                self._flush()

    def stop(self) -> None:
        with self._lock:
            if self._session_dir is None:
                return
            self._flush()
            self._write_metadata({
                "recording_status": "completed",
                "ended_wall_time_s": self._clock(),
                "ended_monotonic_ns": int(self._monotonic_ns()),
                "radar_frames": self._frames,
                "camera_frames": self._camera_frames,
            })
            if self._radar_file is not None:
                self._radar_file.close()
            if self._index_file is not None:
                self._index_file.close()
            self._session_dir = None
            self._metadata_path = None
            self._radar_file = None
            self._index_file = None
            self._index_writer = None
            self._written_camera_ids.clear()

    @property
    def active_path(self) -> Path | None:
        return self._session_dir

    def _write_camera_frame(self, frame: CameraFrame) -> None:
        if self._session_dir is None or frame.frame_id in self._written_camera_ids:
            return
        destination = self._session_dir / "camera_frames" / f"{frame.frame_id}.jpg"
        temporary = destination.with_suffix(".jpg.tmp")
        with temporary.open("wb") as handle:
            handle.write(frame.jpeg)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, destination)
        self._written_camera_ids.add(frame.frame_id)
        self._camera_frames += 1

    def _flush(self) -> None:
        for handle in (self._radar_file, self._index_file):
            if handle is not None:
                handle.flush()
        self._last_flush_monotonic_ns = int(self._monotonic_ns())

    def _write_metadata(self, updates: dict[str, object]) -> None:
        if self._metadata_path is None:
            return
        previous: dict[str, object] = {}
        if self._metadata_path.exists():
            previous = json.loads(self._metadata_path.read_text(encoding="utf-8"))
        previous.update(updates)
        temporary = self._metadata_path.with_suffix(".json.tmp")
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(previous, handle, ensure_ascii=False, indent=2)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, self._metadata_path)


def _optional_int(value: object) -> int | None:
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _optional_float(value: object) -> float | None:
    try:
        return float(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _radar_monotonic_ns(message: Mapping[str, object]) -> int | None:
    direct = _optional_int(message.get("host_monotonic_ns"))
    if direct is not None:
        return direct
    value = _optional_float(message.get("host_monotonic_s"))
    return int(value * 1_000_000_000) if value is not None else None


def _pose_value(pose: object | None, name: str) -> object:
    if pose is None:
        return ""
    if isinstance(pose, Mapping):
        return pose.get(name, "")
    return getattr(pose, name, "")


def _csv_value(value: object | None) -> object:
    return "" if value is None else value
