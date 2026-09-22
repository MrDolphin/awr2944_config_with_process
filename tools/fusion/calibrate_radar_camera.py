"""PC-only OpenCV command that fits a radar-to-camera transform from CSV pairs."""
from __future__ import annotations
import argparse, csv, json
from datetime import datetime, timezone
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description="Fit radar-to-camera extrinsics from known point pairs.")
    parser.add_argument("pairs", type=Path, help="CSV with radar_x,radar_y,radar_z,u,v columns")
    parser.add_argument("intrinsics", type=Path, help="validated camera intrinsics JSON")
    parser.add_argument("output", type=Path)
    parser.add_argument("--mount-mode", choices=("co_rotating", "fixed_camera"), default="co_rotating")
    args = parser.parse_args()
    try:
        import cv2, numpy as np
    except ImportError as error:
        raise SystemExit("OpenCV and NumPy are required on the PC; they are not Pi runtime dependencies.") from error
    source = json.loads(args.intrinsics.read_text(encoding="utf-8"))
    matrix = source["camera_matrix"]; image_size = source["image_size"]; distortion = source.get("distortion", [0, 0, 0, 0, 0])
    rows = list(csv.DictReader(args.pairs.open(encoding="utf-8", newline="")))
    if len(rows) < 6: raise SystemExit("at least six point pairs are required")
    object_points = np.array([[float(row[k]) for k in ("radar_x", "radar_y", "radar_z")] for row in rows], dtype=np.float64)
    image_points = np.array([[float(row[k]) for k in ("u", "v")] for row in rows], dtype=np.float64)
    camera_matrix = np.array([[matrix["fx"], 0, matrix["cx"]], [0, matrix["fy"], matrix["cy"]], [0, 0, 1]], dtype=np.float64)
    success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, np.array(distortion, dtype=np.float64))
    if not success: raise SystemExit("solvePnP failed")
    rotation, _ = cv2.Rodrigues(rvec); projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, np.array(distortion, dtype=np.float64))
    errors = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)
    result = {"schema_version": 1, "image_size": image_size, "camera_matrix": matrix, "distortion": distortion,
              "radar_to_camera": {"rotation_3x3": rotation.tolist(), "translation_m": tvec.reshape(3).tolist()},
              "mount_mode": args.mount_mode, "calibrated_at": datetime.now(timezone.utc).isoformat(),
              "rms_reprojection_error_px": float(np.sqrt(np.mean(errors ** 2))),
              "validation": {"median_px": float(np.median(errors)), "p95_px": float(np.percentile(errors, 95)), "pair_count": len(rows)}}
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

if __name__ == "__main__": main()
