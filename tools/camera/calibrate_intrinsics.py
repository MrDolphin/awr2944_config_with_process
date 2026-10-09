"""Estimate camera intrinsics from checkerboard photographs on the PC.

The output schema is consumed directly by tools/fusion/calibrate_radar_camera.py.
This utility never opens a camera device; it only reads an evidence directory.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable, Sequence


def require_observation_count(count: int, minimum: int) -> None:
    if count < minimum:
        raise ValueError(f"at least {minimum} accepted checkerboard images are required; got {count}")


def build_intrinsics_document(
    *,
    image_size: tuple[int, int],
    camera_matrix: Sequence[Sequence[float]],
    distortion: Sequence[float],
    rms_px: float,
    accepted_images: int,
    board_columns: int,
    board_rows: int,
    square_size_mm: float,
    per_image_rms_px: Sequence[float] = (),
) -> dict:
    width, height = (int(image_size[0]), int(image_size[1]))
    matrix = tuple(tuple(float(value) for value in row) for row in camera_matrix)
    coefficients = tuple(float(value) for value in distortion)
    if width <= 0 or height <= 0:
        raise ValueError("image size must be positive")
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("camera matrix must be 3x3")
    if len(coefficients) != 5:
        raise ValueError("exactly five OpenCV distortion coefficients are required")
    if not math.isfinite(rms_px) or rms_px < 0:
        raise ValueError("RMS must be finite and non-negative")
    return {
        "schema_version": 1,
        "model": "opencv_pinhole_5",
        "image_size": [width, height],
        "camera_matrix": {
            "fx": matrix[0][0],
            "fy": matrix[1][1],
            "cx": matrix[0][2],
            "cy": matrix[1][2],
        },
        "distortion": list(coefficients),
        "calibration_quality": {
            "rms_px": float(rms_px),
            "accepted_images": int(accepted_images),
            "per_image_rms_px": [float(value) for value in per_image_rms_px],
            "board_inner_corners": [int(board_columns), int(board_rows)],
            "square_size_mm": float(square_size_mm),
        },
    }


def write_new_json(path: Path, document: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x", encoding="utf-8", newline="\n") as handle:
            json.dump(document, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
    except FileExistsError as error:
        raise FileExistsError(f"output already exists: {path}") from error


def evaluate_quality(document: dict, *, maximum_rms_px: float) -> dict:
    width, height = (float(value) for value in document["image_size"])
    matrix = document["camera_matrix"]
    fx, fy = float(matrix["fx"]), float(matrix["fy"])
    cx, cy = float(matrix["cx"]), float(matrix["cy"])
    rms = float(document["calibration_quality"]["rms_px"])
    horizontal_fov = math.degrees(2.0 * math.atan(width / (2.0 * fx))) if fx > 0 else math.nan
    vertical_fov = math.degrees(2.0 * math.atan(height / (2.0 * fy))) if fy > 0 else math.nan
    failures = []
    if not math.isfinite(rms) or rms > maximum_rms_px:
        failures.append("rms_above_limit")
    if not (0.0 <= cx <= width and 0.0 <= cy <= height):
        failures.append("principal_point_outside_image")
    if not math.isfinite(horizontal_fov) or not 20.0 <= horizontal_fov <= 170.0:
        failures.append("horizontal_fov_out_of_range")
    if not math.isfinite(vertical_fov) or not 15.0 <= vertical_fov <= 170.0:
        failures.append("vertical_fov_out_of_range")
    if fx <= 0 or fy <= 0 or not 0.5 <= fx / fy <= 2.0:
        failures.append("focal_length_ratio_out_of_range")
    return {
        "passed": not failures,
        "failures": failures,
        "maximum_rms_px": float(maximum_rms_px),
        "horizontal_fov_deg": horizontal_fov,
        "vertical_fov_deg": vertical_fov,
    }


def _object_points(columns: int, rows: int, square_size_m: float, np):
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points[:, :2] *= square_size_m
    return points


def collect_observations(
    image_paths: Iterable[Path], *, columns: int, rows: int, square_size_m: float, cv2, np
):
    object_template = _object_points(columns, rows, square_size_m, np)
    object_points = []
    image_points = []
    accepted = []
    rejected = []
    image_size = None
    pattern_size = (columns, rows)
    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            rejected.append({"path": str(path), "reason": "unreadable_image"})
            continue
        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            rejected.append({"path": str(path), "reason": "image_size_mismatch"})
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(
                gray, pattern_size, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
            )
        else:
            found, corners = cv2.findChessboardCorners(
                gray, pattern_size, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            if found:
                corners = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001),
                )
        if not found:
            rejected.append({"path": str(path), "reason": "checkerboard_not_found"})
            continue
        object_points.append(object_template.copy())
        image_points.append(np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2))
        accepted.append(str(path))
    return image_size, object_points, image_points, accepted, rejected


def calibrate_directory(
    image_dir: Path,
    *,
    glob_pattern: str,
    columns: int,
    rows: int,
    square_size_mm: float,
    minimum_images: int,
) -> tuple[dict, dict]:
    try:
        import cv2
        import numpy as np
    except ImportError as error:
        raise RuntimeError("OpenCV and NumPy are required on the PC") from error

    image_paths = sorted(path for path in Path(image_dir).glob(glob_pattern) if path.is_file())
    if not image_paths:
        raise ValueError(f"no images matched {glob_pattern!r} in {image_dir}")
    image_size, object_points, image_points, accepted, rejected = collect_observations(
        image_paths,
        columns=columns,
        rows=rows,
        square_size_m=square_size_mm / 1000.0,
        cv2=cv2,
        np=np,
    )
    require_observation_count(len(accepted), minimum_images)
    assert image_size is not None
    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    per_image_errors = []
    for object_set, image_set, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(object_set, rvec, tvec, camera_matrix, distortion)
        delta = projected.reshape(-1, 2) - image_set.reshape(-1, 2)
        per_image_errors.append(float(np.sqrt(np.mean(np.sum(delta * delta, axis=1)))))
    distortion5 = np.asarray(distortion, dtype=np.float64).reshape(-1)[:5]
    if distortion5.size != 5:
        raise ValueError("OpenCV did not return five distortion coefficients")
    document = build_intrinsics_document(
        image_size=image_size,
        camera_matrix=camera_matrix.tolist(),
        distortion=distortion5.tolist(),
        rms_px=float(rms),
        accepted_images=len(accepted),
        board_columns=columns,
        board_rows=rows,
        square_size_mm=square_size_mm,
        per_image_rms_px=per_image_errors,
    )
    quality = evaluate_quality(document, maximum_rms_px=1.5)
    report = {
        "input_directory": str(Path(image_dir).resolve()),
        "glob": glob_pattern,
        "accepted": accepted,
        "rejected": rejected,
        "quality": quality,
        "quality_gate_rms_px": quality["maximum_rms_px"],
        "quality_passed": quality["passed"],
    }
    return document, report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--glob", default="*.jpg", dest="glob_pattern")
    parser.add_argument("--columns", type=int, default=9, help="checkerboard inner corners across")
    parser.add_argument("--rows", type=int, default=6, help="checkerboard inner corners down")
    parser.add_argument("--square-mm", type=float, default=20.0)
    parser.add_argument("--minimum-images", type=int, default=12)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.columns < 3 or args.rows < 3 or args.square_mm <= 0 or args.minimum_images < 6:
        raise SystemExit("invalid board dimensions, square size, or minimum image count")
    document, report = calibrate_directory(
        args.image_dir,
        glob_pattern=args.glob_pattern,
        columns=args.columns,
        rows=args.rows,
        square_size_mm=args.square_mm,
        minimum_images=args.minimum_images,
    )
    write_new_json(args.output, document)
    report_path = args.output.with_name(args.output.stem + ".report.json")
    write_new_json(report_path, report)
    print(f"intrinsics: {args.output}")
    print(f"report: {report_path}")
    print(f"accepted images: {document['calibration_quality']['accepted_images']}")
    print(f"RMS: {document['calibration_quality']['rms_px']:.3f} px")
    if not report["quality_passed"]:
        print("quality gate failed: " + ", ".join(report["quality"]["failures"]))
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
