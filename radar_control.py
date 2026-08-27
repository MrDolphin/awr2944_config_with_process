"""Testable control-plane operations for the radar WebSocket server."""

from __future__ import annotations

import os
import tempfile
from pathlib import Path
from typing import Any, Callable

from radar_runtime import ConfigPathError, config_snapshot, resolve_config_path


ConfigSender = Callable[[str, str], bool]
_CONFIG_ERRORS = (ConfigPathError, OSError, UnicodeError, TypeError)


class RadarConfigManager:
    """Keep browser-originated profile operations inside one safe directory."""

    def __init__(self, config_root: str | Path):
        self._config_root = Path(config_root)

    def list_configs(self) -> dict[str, Any]:
        try:
            self._config_root.mkdir(parents=True, exist_ok=True)
            files = sorted(path.name for path in self._config_root.glob("*.cfg") if path.is_file())
            return {"success": True, "files": files}
        except OSError as exc:
            return {"success": False, "files": [], "message": str(exc)}

    def read(self, filename: Any) -> dict[str, Any]:
        try:
            path = resolve_config_path(self._config_root, filename, must_exist=True)
            return {"success": True, "filename": path.name, "content": path.read_text(encoding="utf-8")}
        except _CONFIG_ERRORS as exc:
            return {"success": False, "message": str(exc)}

    def save(self, filename: Any, content: Any) -> dict[str, Any]:
        try:
            if not isinstance(content, str):
                raise ConfigPathError("configuration content must be a string")
            path = self._write_text(filename, content)
            return {"success": True, "filename": path.name}
        except _CONFIG_ERRORS as exc:
            return {"success": False, "message": str(exc)}

    def apply(
        self,
        filename: Any,
        content: Any,
        cfg_port: str,
        send_config: ConfigSender,
    ) -> dict[str, Any]:
        """Persist optional content then apply the exact runtime configuration port."""

        try:
            if content is not None:
                if not isinstance(content, str):
                    raise ConfigPathError("configuration content must be a string")
                if not content.strip():
                    raise ConfigPathError("configuration content must not be blank")
                path = self._write_text(filename, content)
            path = resolve_config_path(self._config_root, filename, must_exist=True)
            try:
                success = bool(send_config(cfg_port, str(path)))
            except Exception as exc:
                return {"success": False, "filename": path.name, "message": f"radar apply failed: {exc}"}
            result: dict[str, Any] = {"success": success, "filename": path.name}
            if success:
                result["active_config"] = config_snapshot(path)
            return result
        except _CONFIG_ERRORS as exc:
            return {"success": False, "message": str(exc)}

    def _write_text(self, filename: Any, content: str) -> Path:
        """Atomically replace one profile so a failed save keeps the last known cfg."""

        path = resolve_config_path(self._config_root, filename)
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                "w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", suffix=".tmp", delete=False
            ) as temporary_file:
                temporary_path = Path(temporary_file.name)
                temporary_file.write(content)
                temporary_file.flush()
                os.fsync(temporary_file.fileno())
            os.replace(temporary_path, path)
            return path
        except Exception:
            if temporary_path is not None:
                try:
                    temporary_path.unlink()
                except OSError:
                    pass
            raise
