"""PC-only integration tests for independent radar-camera calibration validation."""

import errno
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from tools.fusion.calibration import load_calibration


ROOT = Path(__file__).resolve().parents[1]
SOLVER = ROOT / "tools" / "fusion" / "calibrate_radar_camera.py"
IMAGE_SIZE = [1280, 720]
INTRINSICS = {"fx": 800.0, "fy": 810.0, "cx": 640.0, "cy": 360.0}
TRANSLATION = (0.1, -0.05, 0.2)
IDENTITY = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
ROTATED = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
FIT_POINTS = (
    (-1.2, -0.8, 4.0), (0.9, -0.5, 4.7), (-0.7, 0.9, 5.4),
    (1.1, 0.8, 6.0), (0.1, -1.1, 5.8), (-0.3, 0.2, 7.1),
)
VALIDATION_POINTS = ((0.4, 0.7, 4.4), (-0.8, -0.3, 7.5))


def sample(point, set_name, index, *, pixel_shift=0.0, rotation=IDENTITY):
    x, y, z = point
    camera_point = [sum(row[column] * point[column] for column in range(3)) + offset
                    for row, offset in zip(rotation, TRANSLATION)]
    u = INTRINSICS["fx"] * camera_point[0] / camera_point[2] + INTRINSICS["cx"] + pixel_shift
    v = INTRINSICS["fy"] * camera_point[1] / camera_point[2] + INTRINSICS["cy"]
    return {
        "id": f"{set_name}-{index}", "set": set_name,
        "radar": {"frame_num": index + 1, "point_index": index, "x": x, "y": y, "z": z},
        "camera": {"frame_id": f"camera-{index}", "u": u, "v": v},
        "sync_offset_ms": 0.5, "timestamp": "2026-09-23T00:00:00Z",
    }


class CalibrateRadarCameraTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)
        self.session_path = self.path / "session.json"
        self.intrinsics_path = self.path / "intrinsics.json"
        self.output_path = self.path / "calibration.json"
        self.report_path = self.output_path.with_suffix(".report.json")
        self.intrinsics_path.write_text(json.dumps({
            "camera_matrix": INTRINSICS, "image_size": IMAGE_SIZE,
            "distortion": [0, 0, 0, 0, 0],
        }), encoding="utf-8")

    def run_solver(self, *, validation_shift=0.0, validation_count=2, mount=True,
                   rotation=IDENTITY, mount_xyz=None, output_path=None, before_run=None):
        samples = [sample(point, "fit", index, rotation=rotation)
                   for index, point in enumerate(FIT_POINTS)]
        samples += [sample(point, "validation", index, pixel_shift=validation_shift, rotation=rotation)
                    for index, point in enumerate(VALIDATION_POINTS[:validation_count])]
        measured = mount_xyz if mount_xyz is not None else tuple(-value for value in TRANSLATION)
        session_json = json.dumps({
            "schema_version": 1, "camera_image_size": IMAGE_SIZE,
            "radar_id": "radar", "camera_id": "camera",
            "mount_measurement": ({"dx_m": measured[0], "dy_m": measured[1],
                                   "dz_m": measured[2], "uncertainty_m": 0.01,
                                   "reference": "phase centre to optical centre"} if mount else None),
            "samples": samples,
        })
        self.session_path.write_text(session_json, encoding="utf-8")
        self.session_bytes = self.session_path.read_bytes()
        if before_run is not None:
            before_run()
        return subprocess.run(
            [sys.executable, str(SOLVER), str(self.session_path),
             str(self.intrinsics_path), str(output_path or self.output_path),
             "--mount-mode", "co_rotating"],
            cwd=ROOT, capture_output=True, text=True, check=False,
        )

    def hard_link_or_skip(self, source, target):
        try:
            os.link(source, target)
        except OSError as error:
            if error.errno in {errno.EPERM, errno.EACCES, errno.ENOTSUP, errno.ENOSYS} or \
                    getattr(error, "winerror", None) in {50, 1314}:
                self.skipTest(f"hard links unavailable: {error}")
            raise

    def test_pass_writes_report_and_loader_compatible_runtime(self):
        result = self.run_solver()
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(self.report_path.read_text(encoding="utf-8"))
        self.assertEqual(report["fit"]["pair_count"], 6)
        self.assertEqual(report["validation"]["pair_count"], 2)
        for name in ("rms_px", "median_px", "p95_px", "max_px"):
            self.assertLess(report["validation"][name], 1e-5)
        for residual in report["mount_comparison"]["residual_m"]:
            self.assertAlmostEqual(residual, 0.0, places=5)
        calibration = load_calibration(self.output_path)
        self.assertAlmostEqual(calibration.translation_radar_to_camera_m[0], TRANSLATION[0], places=5)
        self.assertAlmostEqual(calibration.rms_reprojection_error_px, report["fit"]["rms_px"])

    def test_bad_validation_writes_report_but_no_runtime(self):
        result = self.run_solver(validation_shift=100.0)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output_path.exists())
        report = json.loads(self.report_path.read_text(encoding="utf-8"))
        self.assertLess(report["fit"]["rms_px"], 1e-5)
        self.assertGreater(report["validation"]["p95_px"], 20.0)
        self.assertGreater(report["validation"]["median_px"], 8.0)

    def test_failed_recalibration_removes_prior_runtime_at_same_path(self):
        first = self.run_solver()
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertTrue(self.output_path.exists())
        failed = self.run_solver(validation_shift=100.0)
        self.assertNotEqual(failed.returncode, 0)
        self.assertFalse(self.output_path.exists())
        report = json.loads(self.report_path.read_text(encoding="utf-8"))
        self.assertFalse(report["validation_passed"])
        self.assertGreater(report["validation"]["p95_px"], 20.0)

    def test_failed_validation_rejects_output_aliasing_session(self):
        result = self.run_solver(validation_shift=100.0, output_path=self.session_path)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("aliases an input", result.stderr)
        self.assertEqual(self.session_path.read_bytes(), self.session_bytes)
        self.assertFalse(self.session_path.with_suffix(".report.json").exists())

    def test_failed_validation_rejects_output_aliasing_intrinsics(self):
        original_intrinsics = self.intrinsics_path.read_bytes()
        result = self.run_solver(validation_shift=100.0, output_path=self.intrinsics_path)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("aliases an input", result.stderr)
        self.assertEqual(self.intrinsics_path.read_bytes(), original_intrinsics)
        self.assertFalse(self.intrinsics_path.with_suffix(".report.json").exists())

    def test_passed_validation_rejects_output_aliasing_session(self):
        result = self.run_solver(output_path=self.session_path)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("aliases an input", result.stderr)
        self.assertEqual(self.session_path.read_bytes(), self.session_bytes)
        self.assertFalse(self.session_path.with_suffix(".report.json").exists())

    def test_passed_validation_rejects_output_aliasing_intrinsics(self):
        original_intrinsics = self.intrinsics_path.read_bytes()
        result = self.run_solver(output_path=self.intrinsics_path)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("aliases an input", result.stderr)
        self.assertEqual(self.intrinsics_path.read_bytes(), original_intrinsics)
        self.assertFalse(self.intrinsics_path.with_suffix(".report.json").exists())

    def test_rejects_report_aliasing_session_without_writing_any_path(self):
        self.session_path = self.path / "session.report.json"
        output_path = self.path / "session.json"
        output_path.write_bytes(b"existing output")
        result = self.run_solver(output_path=output_path)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("aliases an input", result.stderr)
        self.assertEqual(self.session_path.read_bytes(), self.session_bytes)
        self.assertEqual(output_path.read_bytes(), b"existing output")

    def test_rejects_report_aliasing_intrinsics_without_writing_any_path(self):
        report_path = self.path / "intrinsics.report.json"
        self.intrinsics_path.rename(report_path)
        self.intrinsics_path = report_path
        original_intrinsics = self.intrinsics_path.read_bytes()
        output_path = self.path / "intrinsics.json"
        output_path.write_bytes(b"existing output")
        result = self.run_solver(output_path=output_path)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("aliases an input", result.stderr)
        self.assertEqual(self.intrinsics_path.read_bytes(), original_intrinsics)
        self.assertEqual(output_path.read_bytes(), b"existing output")

    def test_rejects_output_hard_link_to_session(self):
        result = self.run_solver(before_run=lambda: self.hard_link_or_skip(
            self.session_path, self.output_path))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("aliases an input", result.stderr)
        self.assertEqual(self.session_path.read_bytes(), self.session_bytes)
        self.assertFalse(self.report_path.exists())

    def test_rejects_output_hard_link_to_intrinsics(self):
        original_intrinsics = self.intrinsics_path.read_bytes()
        result = self.run_solver(before_run=lambda: self.hard_link_or_skip(
            self.intrinsics_path, self.output_path))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("aliases an input", result.stderr)
        self.assertEqual(self.intrinsics_path.read_bytes(), original_intrinsics)
        self.assertFalse(self.report_path.exists())

    def test_rejects_report_hard_link_to_session(self):
        result = self.run_solver(before_run=lambda: self.hard_link_or_skip(
            self.session_path, self.report_path))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("aliases an input", result.stderr)
        self.assertEqual(self.session_path.read_bytes(), self.session_bytes)
        self.assertFalse(self.output_path.exists())

    def test_rejects_report_hard_link_to_intrinsics(self):
        original_intrinsics = self.intrinsics_path.read_bytes()
        result = self.run_solver(before_run=lambda: self.hard_link_or_skip(
            self.intrinsics_path, self.report_path))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("aliases an input", result.stderr)
        self.assertEqual(self.intrinsics_path.read_bytes(), original_intrinsics)
        self.assertFalse(self.output_path.exists())

    def test_rotated_geometry_reports_camera_center_and_mount_residual(self):
        result = self.run_solver(rotation=ROTATED, mount_xyz=(0.03, 0.11, -0.2))
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(self.report_path.read_text(encoding="utf-8"))
        for actual, expected in zip(report["camera_center_in_radar_m"], (0.05, 0.1, -0.2)):
            self.assertAlmostEqual(actual, expected, places=5)
        for actual, expected in zip(report["mount_comparison"]["residual_m"], (0.02, -0.01, 0.0)):
            self.assertAlmostEqual(actual, expected, places=5)
        self.assertLess(report["validation"]["median_px"], 1e-5)

    def test_missing_validation_fails_before_solve(self):
        result = self.run_solver(validation_count=0)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("at least one validation", result.stderr)
        self.assertFalse(self.report_path.exists())
        self.assertFalse(self.output_path.exists())

    def test_no_mount_keeps_comparison_optional(self):
        result = self.run_solver(mount=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(self.report_path.read_text(encoding="utf-8"))
        self.assertIsNone(report["mount_comparison"]["measured_camera_center_in_radar_m"])
        self.assertIsNone(report["mount_comparison"]["residual_m"])


if __name__ == "__main__":
    unittest.main()
