"""Strict loading for a measured radar-to-camera calibration."""
from __future__ import annotations
from dataclasses import dataclass
import json
import math
from pathlib import Path

class CalibrationError(ValueError): pass

@dataclass(frozen=True)
class RadarCameraCalibration:
    image_width: int; image_height: int; fx: float; fy: float; cx: float; cy: float
    distortion: tuple[float, float, float, float, float]
    rotation_radar_to_camera: tuple[tuple[float, float, float], ...]
    translation_radar_to_camera_m: tuple[float, float, float]
    mount_mode: str; rms_reprojection_error_px: float

def load_calibration(path: Path) -> RadarCameraCalibration:
    try: data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error: raise CalibrationError(str(error)) from error
    try:
        if data["schema_version"] != 1: raise CalibrationError("unsupported schema version")
        width, height = data["image_size"]; matrix = data["camera_matrix"]; radar = data["radar_to_camera"]
        result = RadarCameraCalibration(int(width), int(height), float(matrix["fx"]), float(matrix["fy"]), float(matrix["cx"]), float(matrix["cy"]), tuple(float(v) for v in data["distortion"]), tuple(tuple(float(v) for v in row) for row in radar["rotation_3x3"]), tuple(float(v) for v in radar["translation_m"]), data["mount_mode"], float(data["rms_reprojection_error_px"]))
    except (KeyError, TypeError, ValueError) as error: raise CalibrationError("invalid calibration schema") from error
    if result.image_width <= 0 or result.image_height <= 0 or result.fx <= 0 or result.fy <= 0: raise CalibrationError("image size and focal lengths must be positive")
    if len(result.distortion) != 5 or len(result.rotation_radar_to_camera) != 3 or any(len(row) != 3 for row in result.rotation_radar_to_camera) or len(result.translation_radar_to_camera_m) != 3: raise CalibrationError("invalid calibration dimensions")
    if result.mount_mode not in {"co_rotating", "fixed_camera"}: raise CalibrationError("invalid mount mode")
    if not _orthonormal(result.rotation_radar_to_camera): raise CalibrationError("rotation must be orthonormal")
    if not math.isfinite(result.rms_reprojection_error_px) or result.rms_reprojection_error_px < 0: raise CalibrationError("invalid RMS error")
    return result

def _orthonormal(rows):
    for row in rows:
        if not math.isclose(sum(v*v for v in row), 1.0, abs_tol=1e-6): return False
    for left in range(3):
        for right in range(left + 1, 3):
            if not math.isclose(sum(rows[left][i]*rows[right][i] for i in range(3)), 0.0, abs_tol=1e-6): return False
    determinant = (rows[0][0]*(rows[1][1]*rows[2][2]-rows[1][2]*rows[2][1])-rows[0][1]*(rows[1][0]*rows[2][2]-rows[1][2]*rows[2][0])+rows[0][2]*(rows[1][0]*rows[2][1]-rows[1][1]*rows[2][0]))
    return math.isclose(determinant, 1.0, abs_tol=1e-6)
