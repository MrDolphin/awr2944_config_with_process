"""PC-only OpenCV command for fitting and independently validating a session."""

from __future__ import annotations

import argparse
import json
import math
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.fusion.calibration_session import (  # noqa: E402
    SessionError,
    camera_center_in_radar,
    load_session,
    mount_residual,
)


def error_metrics(projected, observed, np):
    errors = np.linalg.norm(projected.reshape(-1, 2) - observed, axis=1)
    return {
        "pair_count": int(len(errors)),
        "rms_px": float(np.sqrt(np.mean(errors ** 2))),
        "median_px": float(np.median(errors)),
        "p95_px": float(np.percentile(errors, 95)),
        "max_px": float(np.max(errors)),
    }


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _vector(value, length):
    return isinstance(value, list) and len(value) == length and all(_number(item) for item in value)


def _metrics(value):
    return (
        isinstance(value, dict)
        and set(value) == {"pair_count", "rms_px", "median_px", "p95_px", "max_px"}
        and type(value["pair_count"]) is int and value["pair_count"] > 0
        and all(_number(value[key]) and value[key] >= 0
                for key in ("rms_px", "median_px", "p95_px", "max_px"))
    )


def _runtime_product(value):
    if not isinstance(value, dict) or set(value) != {
        "schema_version", "image_size", "camera_matrix", "distortion", "radar_to_camera",
        "mount_mode", "calibrated_at", "rms_reprojection_error_px", "validation",
    }:
        return False
    matrix = value["camera_matrix"]
    extrinsics = value["radar_to_camera"]
    try:
        datetime.fromisoformat(value["calibrated_at"])
    except (TypeError, ValueError):
        return False
    return (
        type(value["schema_version"]) is int and value["schema_version"] == 1
        and isinstance(value["image_size"], list) and len(value["image_size"]) == 2
        and all(type(size) is int and size > 0 for size in value["image_size"])
        and isinstance(matrix, dict) and {"fx", "fy", "cx", "cy"} <= matrix.keys()
        and all(_number(matrix[key]) for key in ("fx", "fy", "cx", "cy"))
        and matrix["fx"] > 0 and matrix["fy"] > 0
        and isinstance(value["distortion"], list) and all(_number(item) for item in value["distortion"])
        and isinstance(extrinsics, dict) and set(extrinsics) == {"rotation_3x3", "translation_m"}
        and isinstance(extrinsics["rotation_3x3"], list)
        and len(extrinsics["rotation_3x3"]) == 3
        and all(_vector(row, 3) for row in extrinsics["rotation_3x3"])
        and _vector(extrinsics["translation_m"], 3)
        and value["mount_mode"] in ("co_rotating", "fixed_camera")
        and _number(value["rms_reprojection_error_px"])
        and value["rms_reprojection_error_px"] >= 0
        and _metrics(value["validation"])
    )


def _report_product(value):
    if not isinstance(value, dict) or set(value) != {
        "fit", "validation", "validation_passed", "camera_center_in_radar_m", "mount_comparison",
    }:
        return False
    mount = value["mount_comparison"]
    return (
        _metrics(value["fit"]) and _metrics(value["validation"])
        and type(value["validation_passed"]) is bool
        and _vector(value["camera_center_in_radar_m"], 3)
        and isinstance(mount, dict) and set(mount) == {
            "measured_camera_center_in_radar_m", "uncertainty_m", "reference",
            "yaw_pitch_roll_deg", "residual_m",
        }
        and (mount["measured_camera_center_in_radar_m"] is None
             or _vector(mount["measured_camera_center_in_radar_m"], 3))
        and (mount["uncertainty_m"] is None
             or (_number(mount["uncertainty_m"]) and mount["uncertainty_m"] >= 0))
        and (mount["reference"] is None
             or (isinstance(mount["reference"], str) and bool(mount["reference"].strip())))
        and (mount["yaw_pitch_roll_deg"] is None or _vector(mount["yaw_pitch_roll_deg"], 3))
        and (mount["residual_m"] is None or _vector(mount["residual_m"], 3))
    )


def _preflight_product(path, kind, recognizer):
    """Return whether an existing path is a safe, recognized solver product."""
    try:
        if not path.exists() and not path.is_symlink():
            return False
        if path.is_symlink() or not path.is_file():
            raise SystemExit(f"unsafe existing {kind}: expected a regular solver product file")
        if path.stat().st_nlink != 1:
            raise SystemExit(f"unsafe existing {kind}: hard-linked file")
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise SystemExit(f"unsafe existing {kind}: unreadable or invalid JSON") from error
    if not recognizer(payload):
        raise SystemExit(f"unsafe existing {kind}: unrecognized solver product schema")
    return True


def _clear_solver_products(products):
    """Clear official names together, restoring earlier names if quarantine fails."""
    quarantined = []
    for kind, path in products:
        quarantine = path.with_name(f".{path.name}.quarantine-{uuid.uuid4().hex}")
        while quarantine.exists() or quarantine.is_symlink():
            quarantine = path.with_name(f".{path.name}.quarantine-{uuid.uuid4().hex}")
        try:
            path.rename(quarantine)
        except OSError as error:
            rollback_errors = []
            for original, moved in reversed(quarantined):
                try:
                    moved.rename(original)
                except OSError as rollback_error:
                    rollback_errors.append(f"{original}: {rollback_error}")
            detail = f"; rollback failed for {', '.join(rollback_errors)}" if rollback_errors else ""
            raise SystemExit(f"could not quarantine {kind}: {error}{detail}") from error
        quarantined.append((path, quarantine))

    delete_errors = []
    for _, quarantine in quarantined:
        try:
            quarantine.unlink()
        except OSError as error:
            delete_errors.append(f"{quarantine}: {error}")
    if delete_errors:
        raise SystemExit(f"could not delete solver quarantine: {'; '.join(delete_errors)}")


def main():
    parser = argparse.ArgumentParser(description="Fit radar-to-camera extrinsics from a calibration session.")
    parser.add_argument("session", type=Path, help="exported calibration session JSON")
    parser.add_argument("intrinsics", type=Path, help="validated camera intrinsics JSON")
    parser.add_argument("output", type=Path, help="runtime calibration JSON, written only after validation passes")
    parser.add_argument("--mount-mode", choices=("co_rotating", "fixed_camera"), default="co_rotating")
    args = parser.parse_args()

    report_path = args.output.with_suffix(".report.json")
    input_paths = (args.session, args.intrinsics)
    resolved_inputs = {path.resolve() for path in input_paths}
    output_path = args.output.resolve()
    resolved_report_path = report_path.resolve()

    def aliases_input(candidate, resolved_candidate):
        return resolved_candidate in resolved_inputs or (
            candidate.exists() and any(
                source.exists() and candidate.samefile(source) for source in input_paths
            )
        )

    if aliases_input(args.output, output_path):
        raise SystemExit("output path aliases an input")
    if aliases_input(report_path, resolved_report_path):
        raise SystemExit("report path aliases an input")
    if output_path == resolved_report_path or (
        args.output.exists() and report_path.exists() and args.output.samefile(report_path)
    ):
        raise SystemExit("output and report paths alias each other")

    old_output = _preflight_product(args.output, "output", _runtime_product)
    old_report = _preflight_product(report_path, "report", _report_product)
    _clear_solver_products([
        (kind, path) for kind, path, present in (
            ("output", args.output, old_output), ("report", report_path, old_report)
        ) if present
    ])

    try:
        session = load_session(args.session)
    except SessionError as error:
        raise SystemExit(str(error)) from error

    try:
        import cv2
        import numpy as np
    except ImportError as error:
        raise SystemExit("OpenCV and NumPy are required on the PC; they are not Pi runtime dependencies.") from error

    source = json.loads(args.intrinsics.read_text(encoding="utf-8"))
    matrix = source["camera_matrix"]
    image_size = source["image_size"]
    if image_size != list(session.camera_image_size):
        raise SystemExit("session camera image size differs from intrinsics image size")
    distortion = source.get("distortion", [0, 0, 0, 0, 0])
    camera_matrix = np.array([
        [matrix["fx"], 0, matrix["cx"]],
        [0, matrix["fy"], matrix["cy"]],
        [0, 0, 1],
    ], dtype=np.float64)
    distortion_array = np.array(distortion, dtype=np.float64)

    fit_pairs = np.asarray(session.fit_pairs(), dtype=np.float64)
    validation_pairs = np.asarray(session.validation_pairs(), dtype=np.float64)
    fit_points = np.ascontiguousarray(fit_pairs[:, :3])
    fit_pixels = np.ascontiguousarray(fit_pairs[:, 3:])
    validation_points = np.ascontiguousarray(validation_pairs[:, :3])
    validation_pixels = np.ascontiguousarray(validation_pairs[:, 3:])
    success, rvec, tvec = cv2.solvePnP(fit_points, fit_pixels, camera_matrix, distortion_array)
    if not success:
        raise SystemExit("solvePnP failed")

    rotation, _ = cv2.Rodrigues(rvec)
    fit_projected, _ = cv2.projectPoints(fit_points, rvec, tvec, camera_matrix, distortion_array)
    validation_projected, _ = cv2.projectPoints(validation_points, rvec, tvec, camera_matrix, distortion_array)
    fit = error_metrics(fit_projected, fit_pixels, np)
    validation = error_metrics(validation_projected, validation_pixels, np)
    translation = tvec.reshape(3).tolist()
    solved_center = camera_center_in_radar(rotation.tolist(), translation)
    measured_center = session.mount_measurement
    mount_comparison = {
        "measured_camera_center_in_radar_m": list(measured_center) if measured_center is not None else None,
        "uncertainty_m": session.mount_uncertainty_m,
        "reference": session.mount_reference,
        "yaw_pitch_roll_deg": (list(session.mount_yaw_pitch_roll_deg)
                               if session.mount_yaw_pitch_roll_deg is not None else None),
        "residual_m": (list(mount_residual(measured_center, solved_center))
                       if measured_center is not None else None),
    }
    passed = validation["median_px"] <= 8.0 and validation["p95_px"] <= 20.0
    report = {
        "fit": fit,
        "validation": validation,
        "validation_passed": passed,
        "camera_center_in_radar_m": list(solved_center),
        "mount_comparison": mount_comparison,
    }
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    if not passed:
        raise SystemExit("independent validation failed: median must be <= 8 px and P95 <= 20 px")

    result = {
        "schema_version": 1,
        "image_size": image_size,
        "camera_matrix": matrix,
        "distortion": distortion,
        "radar_to_camera": {"rotation_3x3": rotation.tolist(), "translation_m": translation},
        "mount_mode": args.mount_mode,
        "calibrated_at": datetime.now(timezone.utc).isoformat(),
        "rms_reprojection_error_px": fit["rms_px"],
        "validation": validation,
    }
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
