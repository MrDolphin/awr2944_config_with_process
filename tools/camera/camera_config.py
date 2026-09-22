"""Validated camera capture settings shared by capture and probe commands."""

from dataclasses import dataclass
from pathlib import Path


_ALLOWED_KEYS = {"device", "width", "height", "fps", "input_format"}
_REQUIRED_KEYS = {"device", "width", "height", "fps"}


@dataclass(frozen=True)
class CameraConfig:
    device: str
    width: int
    height: int
    fps: int
    input_format: str = "mjpeg"

    def __post_init__(self) -> None:
        if not self.device.strip():
            raise ValueError("device must not be empty")
        for field_name in ("width", "height", "fps"):
            if getattr(self, field_name) <= 0:
                raise ValueError(f"{field_name} must be positive")
        if self.input_format.lower() != "mjpeg":
            raise ValueError("input_format must be mjpeg")


def load_camera_config(path: Path) -> CameraConfig:
    """Parse flat ``key=value`` camera settings and reject unknown keys."""
    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ValueError(f"invalid camera configuration line {line_number}")
        key, value = (part.strip() for part in line.split("=", 1))
        if key not in _ALLOWED_KEYS:
            raise ValueError(f"unknown camera configuration key: {key}")
        if key in values:
            raise ValueError(f"duplicate camera configuration key: {key}")
        values[key] = value

    missing = _REQUIRED_KEYS - values.keys()
    if missing:
        raise ValueError(f"missing camera configuration keys: {', '.join(sorted(missing))}")

    try:
        return CameraConfig(
            device=values["device"],
            width=int(values["width"]),
            height=int(values["height"]),
            fps=int(values["fps"]),
            input_format=values.get("input_format", "mjpeg").lower(),
        )
    except ValueError as error:
        raise ValueError(f"invalid camera configuration: {error}") from error
