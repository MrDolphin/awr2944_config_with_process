"""PC-only OpenCV command for fitting and independently validating a session."""

from __future__ import annotations

import argparse
import json
import sys
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
        args.output.unlink(missing_ok=True)
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
