"""PC-only OpenCV command for fitting and independently validating a session."""

from __future__ import annotations

import argparse
import json
import os
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


class _ReservedProduct:
    """Hold exclusive ownership of a new official path until publication ends."""

    def __init__(self, path: Path, kind: str):
        self.path = path
        self.kind = kind
        try:
            self.stream = path.open("x+b")
        except FileExistsError as error:
            raise SystemExit(f"{kind} path already exists; choose a new output path/run directory") from error
        except OSError as error:
            raise SystemExit(f"could not reserve {kind} path: {error}") from error
        identity = os.fstat(self.stream.fileno())
        self.identity = (identity.st_dev, identity.st_ino)
        self.removed = False

    def _owns_path(self):
        if self.path.is_symlink():
            return False
        current = self.path.stat()
        return (current.st_dev, current.st_ino) == self.identity

    def write(self, value):
        try:
            if not self._owns_path():
                raise OSError("reserved path was replaced")
            encoded = json.dumps(value, ensure_ascii=False, indent=2).encode("utf-8")
            self.stream.write(encoded)
            self.stream.flush()
            if not self._owns_path():
                raise OSError("reserved path was replaced")
        except Exception as error:
            raise SystemExit(f"could not publish {self.kind}: {error}") from error

    def close(self):
        if not self.stream.closed:
            self.stream.close()

    def discard(self):
        if self.removed:
            return
        self.close()  # Windows cannot unlink an open file.
        try:
            if self._owns_path():
                self.path.unlink()
            elif self.path.exists() or self.path.is_symlink():
                raise OSError("reserved path was replaced; refusing to remove it")
        except FileNotFoundError:
            pass
        self.removed = True


def main():
    parser = argparse.ArgumentParser(
        description="Fit radar-to-camera extrinsics from a calibration session.",
        epilog="Choose a new output path/run directory for every run.",
    )
    parser.add_argument("session", type=Path, help="exported calibration session JSON")
    parser.add_argument("intrinsics", type=Path, help="validated camera intrinsics JSON")
    parser.add_argument("output", type=Path, help="new runtime calibration JSON path; report path must also be unused")
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
        raise SystemExit("output path aliases an input; choose a new output path/run directory")
    if aliases_input(report_path, resolved_report_path):
        raise SystemExit("report path aliases an input; choose a new output path/run directory")
    if output_path == resolved_report_path or (
        args.output.exists() and report_path.exists() and args.output.samefile(report_path)
    ):
        raise SystemExit("output and report paths alias each other; choose a new output path/run directory")

    for kind, path in (("output", args.output), ("report", report_path)):
        try:
            occupied = path.exists() or path.is_symlink()
        except OSError as error:
            raise SystemExit(f"could not inspect {kind} path: {error}") from error
        if occupied:
            raise SystemExit(f"{kind} path already exists; choose a new output path/run directory")

    reserved = []
    try:
        output_product = _ReservedProduct(args.output, "output")
        reserved.append(output_product)
        report_product = _ReservedProduct(report_path, "report")
        reserved.append(report_product)
        validation_failure = _solve_and_publish(args, output_product, report_product)
        for product in reserved:
            try:
                product.close()
            except OSError as close_error:
                raise SystemExit(f"could not publish {product.kind}: {close_error}") from close_error
    except BaseException as error:
        cleanup_errors = []
        for product in reserved:
            try:
                product.discard()
            except OSError as cleanup_error:
                cleanup_errors.append(f"{product.kind}: {cleanup_error}")
        if cleanup_errors:
            raise SystemExit(f"{error}; could not clean reserved paths: {', '.join(cleanup_errors)}") from error
        raise
    if validation_failure:
        raise SystemExit(validation_failure)


def _solve_and_publish(args, output_product, report_product):
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
    if not passed:
        output_product.discard()
        report_product.write(report)
        return "independent validation failed: median must be <= 8 px and P95 <= 20 px"

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
    output_product.write(result)
    report_product.write(report)
    return None


if __name__ == "__main__":
    main()
