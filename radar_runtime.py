"""Hardware-independent runtime helpers for radar recording and configuration."""

from __future__ import annotations

import csv
import hashlib
import json
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
    if candidate.name != filename or candidate.suffix.lower() != ".cfg":
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
    """Record replayable radar frames through a small, thread-safe interface."""

    def __init__(self, capture_root: str | Path, *, clock=time.time, monotonic=time.monotonic):
        self._capture_root = Path(capture_root)
        self._clock = clock
        self._monotonic = monotonic
        self._lock = threading.RLock()
        self._capture_dir: Path | None = None
        self._frames_file = None
        self._raw_file = None
        self._points_file = None
        self._points_writer = None
        self._frames = 0
        self._points = 0
        self._last_frame: int | None = None

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
            capture_dir.mkdir(parents=True)

            self._frames_file = (capture_dir / "frames.jsonl").open("w", encoding="utf-8")
            self._raw_file = (capture_dir / "frames.tlv").open("wb")
            self._points_file = (capture_dir / "points.csv").open("w", newline="", encoding="utf-8")
            self._points_writer = csv.writer(self._points_file)
            self._points_writer.writerow([
                "host_time_s", "frame_num", "point_index", "x_m", "y_m", "z_m", "v_mps", "snr_db", "noise_db"
            ])
            metadata = {
                "schema_version": 1,
                "started_at_s": self._clock(),
                "active_config": dict(active_config or {}),
                "files": {"frames": "frames.jsonl", "raw_tlv": "frames.tlv", "points": "points.csv"},
            }
            (capture_dir / "metadata.json").write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            self._capture_dir = capture_dir
            self._frames = 0
            self._points = 0
            self._last_frame = None
            return str(capture_dir)

    def record_frame(self, frame: Mapping[str, Any], raw_packet: bytes | None = None) -> bool:
        """Persist one frame, including empty frames, and return whether it was new."""

        with self._lock:
            if self._capture_dir is None or self._frames_file is None or self._points_writer is None:
                return False
            frame_num = int(frame.get("frame_num", -1))
            if frame_num == self._last_frame:
                return False

            host_time_s = float(frame.get("host_time_s", self._clock()))
            raw_offset = self._raw_file.tell() if self._raw_file is not None else None
            raw_length = len(raw_packet or b"")
            if raw_packet and self._raw_file is not None:
                self._raw_file.write(raw_packet)

            points = list(frame.get("points") or [])
            frame_record = {
                "host_time_s": host_time_s,
                "host_monotonic_s": frame.get("host_monotonic_s", self._monotonic()),
                "frame_num": frame_num,
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
                    f"{host_time_s:.6f}", frame_num, index,
                    point.get("x", ""), point.get("y", ""), point.get("z", ""), point.get("v", ""),
                    point.get("snr", ""), point.get("noise", ""),
                ])
            self._frames += 1
            self._points += len(points)
            self._last_frame = frame_num
            if self._frames % 20 == 0:
                self._flush()
            return True

    def stop(self) -> dict[str, Any]:
        with self._lock:
            status = self.status()
            for file_obj in (self._frames_file, self._raw_file, self._points_file):
                if file_obj is not None:
                    file_obj.flush()
                    file_obj.close()
            self._frames_file = self._raw_file = self._points_file = self._points_writer = None
            self._capture_dir = None
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
            }

    def _flush(self) -> None:
        for file_obj in (self._frames_file, self._raw_file, self._points_file):
            if file_obj is not None:
                file_obj.flush()
