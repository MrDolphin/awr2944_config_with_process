"""Hardware-independent runtime helpers for radar recording and configuration."""

from __future__ import annotations

import csv
import hashlib
import json
import os
import queue
import threading
import time
from pathlib import Path
from typing import Any, Mapping


class ConfigPathError(ValueError):
    """Raised when a requested configuration escapes the configuration directory."""


def resolve_config_path(config_root: str | Path, filename: str, *, must_exist: bool = False) -> Path:
    """Return a safe ``.cfg`` path rooted under ``config_root``.

    The browser protocol accepts a filename, not an arbitrary path. Keeping this
    invariant here prevents accidental writes outside ``Config``.
    """

    if not isinstance(filename, str) or not filename:
        raise ConfigPathError("配置文件名不能为空")
    candidate = Path(filename)
    if candidate.name != filename or candidate.suffix != ".cfg":
        raise ConfigPathError("配置文件名必须是 Config 目录下的 .cfg 文件")

    root = Path(config_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    path = (root / candidate.name).resolve()
    if path.parent != root:
        raise ConfigPathError("配置文件路径无效")
    if must_exist and not path.is_file():
        raise ConfigPathError("配置文件不存在")
    return path


def config_snapshot(path: str | Path | None) -> dict[str, Any]:
    """Capture a stable, serializable identity for the applied radar profile."""

    if not path:
        return {"name": None, "sha256": None, "content": None}
    config_path = Path(path)
    if not config_path.is_file():
        return {"name": config_path.name, "sha256": None, "content": None}
    content = config_path.read_text(encoding="utf-8")
    return {
        "name": config_path.name,
        "sha256": hashlib.sha256(content.encode("utf-8")).hexdigest(),
        "content": content,
    }


class PointCloudRecorder:
    """Record replayable frames without making the serial parser wait for disk I/O.

    Frames are accepted through a bounded queue.  When storage cannot keep up,
    the recorder reports dropped frames rather than slowing or crashing the
    radar serial thread.  Each persisted frame is flushed; ``sync_every_frames``
    controls the stronger (but slower) OS-level sync cadence.
    """

    def __init__(
        self,
        capture_root: str | Path,
        *,
        clock=time.time,
        monotonic=time.monotonic,
        max_queue_size: int = 256,
        sync_every_frames: int = 1,
    ):
        self._capture_root = Path(capture_root)
        self._clock = clock
        self._monotonic = monotonic
        self._lock = threading.RLock()
        self._capture_dir: Path | None = None
        self._metadata_path: Path | None = None
        self._frames_file = None
        self._raw_file = None
        self._points_file = None
        self._points_writer = None
        self._frames = 0
        self._points = 0
        self._last_frame: int | None = None
        self._queue: queue.Queue[tuple[Mapping[str, Any], bytes | None] | None] | None = None
        self._writer_thread: threading.Thread | None = None
        self._max_queue_size = max_queue_size
        self._sync_every_frames = sync_every_frames
        self._dropped_frames = 0
        self._writer_error: str | None = None
        self._accepting = False

    def start(self, active_config: Mapping[str, Any] | None = None) -> str:
        with self._lock:
            if self._capture_dir is not None:
                return str(self._capture_dir)

            timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self._clock()))
            capture_dir = self._capture_root / f"capture_{timestamp}"
            suffix = 1
            while capture_dir.exists():
                capture_dir = self._capture_root / f"capture_{timestamp}_{suffix:02d}"
                suffix += 1
            created_paths: list[Path] = []
            try:
                capture_dir.mkdir(parents=True)
                self._frames_file = (capture_dir / "frames.jsonl").open("w", encoding="utf-8")
                created_paths.append(capture_dir / "frames.jsonl")
                self._raw_file = (capture_dir / "frames.tlv").open("wb")
                created_paths.append(capture_dir / "frames.tlv")
                self._points_file = (capture_dir / "points.csv").open("w", newline="", encoding="utf-8")
                created_paths.append(capture_dir / "points.csv")
                self._points_writer = csv.writer(self._points_file)
                self._points_writer.writerow([
                    "record_index", "host_time_s", "frame_num", "point_index", "x_m", "y_m", "z_m", "v_mps", "snr_db", "noise_db"
                ])
                metadata = {
                    "schema_version": 2,
                    "started_at_s": self._clock(),
                    "active_config": dict(active_config or {}),
                    "recording_status": "active",
                    "files": {"frames": "frames.jsonl", "raw_tlv": "frames.tlv", "points": "points.csv"},
                    "recording_policy": {
                        "max_queue_size": self._max_queue_size,
                        "sync_every_frames": self._sync_every_frames,
                    },
                }
                metadata_path = capture_dir / "metadata.json"
                metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
                created_paths.append(metadata_path)
            except Exception:
                self._close_files()
                for path in created_paths:
                    try:
                        path.unlink()
                    except OSError:
                        pass
                try:
                    capture_dir.rmdir()
                except OSError:
                    pass
                raise
            self._capture_dir = capture_dir
            self._metadata_path = metadata_path
            self._frames = 0
            self._points = 0
            self._last_frame = None
            self._dropped_frames = 0
            self._writer_error = None
            self._accepting = True
            self._queue = queue.Queue(maxsize=self._max_queue_size)
            self._writer_thread = threading.Thread(target=self._writer_loop, name="pointcloud-recorder", daemon=True)
            self._writer_thread.start()
            return str(capture_dir)

    def record_frame(self, frame: Mapping[str, Any], raw_packet: bytes | None = None) -> bool:
        """Persist one frame, including empty frames, and return whether it was new."""

        with self._lock:
            if not self._accepting or self._queue is None or self._writer_error is not None:
                return False
            frame_num = int(frame.get("frame_num", -1))
            if frame_num == self._last_frame:
                return False
            payload = (dict(frame), raw_packet)
            try:
                self._queue.put_nowait(payload)
            except queue.Full:
                self._dropped_frames += 1
                return False
            self._last_frame = frame_num
            return True

    def stop(self) -> dict[str, Any]:
        with self._lock:
            writer_queue = self._queue
            writer_thread = self._writer_thread
            self._accepting = False
        if writer_queue is not None:
            writer_queue.put(None)
        if writer_thread is not None:
            writer_thread.join()
        with self._lock:
            self._flush(sync=True)
            status = self.status()
            self._persist_completion_status(status)
            self._close_files()
            self._queue = None
            self._writer_thread = None
            self._capture_dir = None
            self._metadata_path = None
            self._last_frame = None
            return status

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "enabled": self._capture_dir is not None,
                "path": str(self._capture_dir) if self._capture_dir else "",
                "frames": self._frames,
                "rows": self._points,
                "points": self._points,
                "queued_frames": self._queue.qsize() if self._queue is not None else 0,
                "dropped_frames": self._dropped_frames,
                "writer_error": self._writer_error,
            }

    def _writer_loop(self) -> None:
        assert self._queue is not None
        while True:
            payload = self._queue.get()
            try:
                if payload is None:
                    return
                with self._lock:
                    failed = self._writer_error is not None
                if not failed:
                    self._write_frame(*payload)
            except Exception as exc:
                with self._lock:
                    self._writer_error = f"{type(exc).__name__}: {exc}"
                    self._accepting = False
            finally:
                self._queue.task_done()

    def _write_frame(self, frame: Mapping[str, Any], raw_packet: bytes | None) -> None:
        with self._lock:
            if self._frames_file is None or self._points_writer is None:
                return
            host_time_s = float(frame.get("host_time_s", self._clock()))
            raw_offset = self._raw_file.tell() if self._raw_file is not None else None
            raw_length = len(raw_packet or b"")
            if raw_packet and self._raw_file is not None:
                self._raw_file.write(raw_packet)
            points = list(frame.get("points") or [])
            frame_record = {
                "record_index": self._frames,
                "host_time_s": host_time_s,
                "host_monotonic_s": frame.get("host_monotonic_s", self._monotonic()),
                "frame_num": int(frame.get("frame_num", -1)),
                "device_time_cpu_cycles": frame.get("device_time_cpu_cycles"),
                "detected_object_count": frame.get("detected_object_count"),
                "point_count": len(points),
                "side_info_count": frame.get("side_info_count", 0),
                "tlv_types": frame.get("tlv_types", []),
                "raw_offset": raw_offset,
                "raw_length": raw_length,
            }
            self._frames_file.write(json.dumps(frame_record, ensure_ascii=False) + "\n")
            for index, point in enumerate(points):
                self._points_writer.writerow([
                    frame_record["record_index"], f"{host_time_s:.6f}", frame_record["frame_num"], index,
                    point.get("x", ""), point.get("y", ""), point.get("z", ""), point.get("v", ""),
                    point.get("snr", ""), point.get("noise", ""),
                ])
            self._frames += 1
            self._points += len(points)
            self._flush(sync=self._sync_every_frames > 0 and self._frames % self._sync_every_frames == 0)

    def _flush(self, *, sync: bool = False) -> None:
        for file_obj in (self._frames_file, self._raw_file, self._points_file):
            if file_obj is not None:
                file_obj.flush()
                if sync:
                    os.fsync(file_obj.fileno())

    def _close_files(self) -> None:
        for file_obj in (self._frames_file, self._raw_file, self._points_file):
            if file_obj is not None:
                file_obj.close()
        self._frames_file = self._raw_file = self._points_file = self._points_writer = None

    def _persist_completion_status(self, status: Mapping[str, Any]) -> None:
        """Make capture quality available to later offline replay and field triage."""

        if self._metadata_path is None:
            return
        try:
            metadata = json.loads(self._metadata_path.read_text(encoding="utf-8"))
            metadata.update({
                "recording_status": "failed" if status["writer_error"] else (
                    "completed_with_drops" if status["dropped_frames"] else "completed"
                ),
                "completed_at_s": self._clock(),
                "frames": status["frames"],
                "points": status["points"],
                "dropped_frames": status["dropped_frames"],
                "writer_error": status["writer_error"],
            })
            temporary_path = self._metadata_path.with_suffix(".json.tmp")
            with temporary_path.open("w", encoding="utf-8") as handle:
                json.dump(metadata, handle, ensure_ascii=False, indent=2)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_path, self._metadata_path)
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._writer_error = self._writer_error or f"metadata update failed: {exc}"
