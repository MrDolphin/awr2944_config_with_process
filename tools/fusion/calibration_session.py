"""Validated, immutable input contract for manual radar-camera calibration."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


class SessionError(ValueError):
    """Raised when a calibration session does not meet schema 1 requirements."""


@dataclass(frozen=True)
class CalibrationSample:
    sample_id: str
    set_name: str
    radar_frame_num: int | str
    radar_point_index: int
    x: float
    y: float
    z: float
    camera_frame_id: int | str
    u: float
    v: float
    sync_offset_ms: float
    timestamp: str
    note: str = ""


@dataclass(frozen=True)
class CalibrationSession:
    schema_version: int
    camera_image_size: tuple[int, int]
    radar_id: str
    camera_id: str
    samples: tuple[CalibrationSample, ...]
    mount_measurement: tuple[float, float, float] | None
    mount_yaw_pitch_roll_deg: tuple[float, float, float] | None = None
    mount_uncertainty_m: float | None = None
    mount_reference: str | None = None

    def fit_pairs(self) -> tuple[tuple[float, float, float, float, float], ...]:
        return tuple((s.x, s.y, s.z, s.u, s.v) for s in self.samples if s.set_name == "fit")

    def validation_pairs(self) -> tuple[tuple[float, float, float, float, float], ...]:
        return tuple((s.x, s.y, s.z, s.u, s.v) for s in self.samples if s.set_name == "validation")


def _mapping(value: Any, where: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise SessionError(f"{where} must be an object")
    return value


def _identifier(value: Any, where: str) -> int | str:
    if isinstance(value, bool) or not isinstance(value, (str, int)) or (isinstance(value, str) and not value.strip()):
        raise SessionError(f"{where} identifier is required")
    return value


def _finite(value: Any, where: str) -> float:
    if isinstance(value, bool):
        raise SessionError(f"{where} must be finite")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise SessionError(f"{where} must be finite") from exc
    if not math.isfinite(number):
        raise SessionError(f"{where} must be finite")
    return number


def _nonempty_string(value: Any, where: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SessionError(f"{where} is required")
    return value


def _optional_vector(source: Mapping[str, Any], keys: Sequence[str], where: str) -> tuple[float, ...] | None:
    present = [key in source and source[key] is not None for key in keys]
    if not any(present):
        return None
    if not all(present):
        raise SessionError(f"{where} must provide all of {', '.join(keys)}")
    return tuple(_finite(source[key], f"{where}.{key}") for key in keys)


def load_session(path: Path) -> CalibrationSession:
    """Load and validate a schema 1 calibration session JSON document."""
    try:
        with Path(path).open("r", encoding="utf-8") as stream:
            payload = json.load(stream)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise SessionError(f"could not read calibration session: {exc}") from exc

    root = _mapping(payload, "session")
    version = root.get("schema_version")
    if isinstance(version, bool) or version != 1:
        raise SessionError("schema_version must be 1")

    size = root.get("camera_image_size")
    if not isinstance(size, (list, tuple)) or len(size) != 2:
        raise SessionError("camera_image_size must contain positive width and height")
    dimensions = []
    for dimension in size:
        if isinstance(dimension, bool) or not isinstance(dimension, int) or dimension <= 0:
            raise SessionError("camera image dimensions must be positive integers")
        dimensions.append(dimension)

    radar_id = _nonempty_string(root.get("radar_id"), "radar_id")
    camera_id = _nonempty_string(root.get("camera_id"), "camera_id")
    raw_samples = root.get("samples")
    if not isinstance(raw_samples, list):
        raise SessionError("samples must be an array")

    samples: list[CalibrationSample] = []
    for index, raw in enumerate(raw_samples):
        row = _mapping(raw, f"samples[{index}]")
        set_name = row.get("set")
        if set_name not in ("fit", "validation"):
            raise SessionError(f"samples[{index}].set must be fit or validation")
        radar = _mapping(row.get("radar"), f"samples[{index}].radar")
        camera = _mapping(row.get("camera"), f"samples[{index}].camera")
        frame = _identifier(radar.get("frame_num"), f"samples[{index}].radar.frame_num")
        camera_frame = _identifier(camera.get("frame_id"), f"samples[{index}].camera.frame_id")
        point_index = radar.get("point_index")
        if isinstance(point_index, bool) or not isinstance(point_index, int) or point_index < 0:
            raise SessionError(f"samples[{index}].radar.point_index must be a non-negative integer")
        samples.append(CalibrationSample(
            sample_id=_nonempty_string(row.get("id"), f"samples[{index}].id"),
            set_name=set_name,
            radar_frame_num=frame,
            radar_point_index=point_index,
            x=_finite(radar.get("x"), f"samples[{index}].radar.x"),
            y=_finite(radar.get("y"), f"samples[{index}].radar.y"),
            z=_finite(radar.get("z"), f"samples[{index}].radar.z"),
            camera_frame_id=camera_frame,
            u=_finite(camera.get("u"), f"samples[{index}].camera.u"),
            v=_finite(camera.get("v"), f"samples[{index}].camera.v"),
            sync_offset_ms=_finite(row.get("sync_offset_ms"), f"samples[{index}].sync_offset_ms"),
            timestamp=_nonempty_string(row.get("timestamp"), f"samples[{index}].timestamp"),
            note=row.get("note", "") if isinstance(row.get("note", ""), str) else "",
        ))

    fit_count = sum(sample.set_name == "fit" for sample in samples)
    validation_count = sum(sample.set_name == "validation" for sample in samples)
    if fit_count < 6:
        raise SessionError("at least six fit samples are required")
    if validation_count < 1:
        raise SessionError("at least one validation sample is required")

    raw_mount = root.get("mount_measurement")
    mount = None
    ypr = None
    uncertainty = None
    reference = None
    if raw_mount is not None:
        mount_data = _mapping(raw_mount, "mount_measurement")
        mount = _optional_vector(mount_data, ("dx_m", "dy_m", "dz_m"), "mount_measurement")
        if mount is None:
            raise SessionError("mount_measurement must provide dx_m, dy_m, and dz_m")
        ypr = _optional_vector(mount_data, ("yaw_deg", "pitch_deg", "roll_deg"), "mount orientation")
        if "uncertainty_m" in mount_data and mount_data["uncertainty_m"] is not None:
            uncertainty = _finite(mount_data["uncertainty_m"], "mount_measurement.uncertainty_m")
            if uncertainty < 0:
                raise SessionError("mount uncertainty must be non-negative")
        if "reference" in mount_data and mount_data["reference"] is not None:
            reference = _nonempty_string(mount_data["reference"], "mount_measurement.reference")
        if uncertainty is None or reference is None:
            raise SessionError("mount_measurement requires non-negative uncertainty and a measurement reference")

    return CalibrationSession(
        schema_version=1,
        camera_image_size=(dimensions[0], dimensions[1]),
        radar_id=radar_id,
        camera_id=camera_id,
        samples=tuple(samples),
        mount_measurement=mount,
        mount_yaw_pitch_roll_deg=ypr,  # type: ignore[arg-type]
        mount_uncertainty_m=uncertainty,
        mount_reference=reference,
    )


def camera_center_in_radar(rotation: Sequence[Sequence[float]], translation: Sequence[float]) -> tuple[float, float, float]:
    """Convert camera-frame extrinsic translation to camera centre in radar axes: -R^T t."""
    try:
        if len(rotation) != 3 or any(len(row) != 3 for row in rotation) or len(translation) != 3:
            raise ValueError
        r = tuple(tuple(_finite(value, "rotation") for value in row) for row in rotation)
        t = tuple(_finite(value, "translation") for value in translation)
    except (TypeError, ValueError, IndexError) as exc:
        raise ValueError("rotation must be 3x3 and translation must have 3 finite values") from exc
    return tuple(-sum(r[row][column] * t[row] for row in range(3)) for column in range(3))  # type: ignore[return-value]


def mount_residual(measured: Sequence[float], solved: Sequence[float]) -> tuple[float, float, float]:
    """Return solved minus measured camera-centre offset, in radar axes."""
    try:
        if len(measured) != 3 or len(solved) != 3:
            raise ValueError
        measured_values = tuple(_finite(value, "measured mount") for value in measured)
        solved_values = tuple(_finite(value, "solved mount") for value in solved)
    except (TypeError, ValueError) as exc:
        raise ValueError("measured and solved mounts must each contain 3 finite values") from exc
    return tuple(solved_values[i] - measured_values[i] for i in range(3))  # type: ignore[return-value]
