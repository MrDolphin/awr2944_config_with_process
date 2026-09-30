"""PC-only integration tests for independent radar-camera calibration validation."""

import errno
import hashlib
import json
import os
import stat
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

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
        "radar": {"frame_num": index + (1 if set_name == "fit" else 101),
                  "point_index": index + (0 if set_name == "fit" else 100),
                  "x": x, "y": y, "z": z},
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
        self.complete_path = self.output_path.with_suffix(".complete.json")
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

    def symlink_or_skip(self, target, link):
        try:
            os.symlink(target, link)
        except OSError as error:
            if error.errno in {errno.EPERM, errno.EACCES, errno.ENOTSUP, errno.ENOSYS} or \
                    getattr(error, "winerror", None) in {50, 1314}:
                self.skipTest(f"symbolic links unavailable: {error}")
            raise

    def test_pass_writes_report_and_loader_compatible_runtime(self):
        from tools.fusion.calibrate_radar_camera import verify_completion

        result = self.run_solver()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(str(self.complete_path), result.stdout)
        report = json.loads(self.report_path.read_text(encoding="utf-8"))
        runtime = json.loads(self.output_path.read_text(encoding="utf-8"))
        complete = json.loads(self.complete_path.read_text(encoding="utf-8"))
        self.assertEqual(complete["schema"], "radar_camera_calibration_completion")
        self.assertEqual(complete["schema_version"], 1)
        self.assertTrue(complete["validation_passed"])
        self.assertEqual(runtime["artifact_id"], report["artifact_id"])
        self.assertEqual(complete["artifact_id"], runtime["artifact_id"])
        self.assertEqual(complete["runtime_filename"], self.output_path.name)
        self.assertEqual(complete["report_filename"], self.report_path.name)
        self.assertEqual(complete["runtime_sha256"], hashlib.sha256(self.output_path.read_bytes()).hexdigest())
        self.assertEqual(complete["report_sha256"], hashlib.sha256(self.report_path.read_bytes()).hexdigest())
        self.assertTrue(verify_completion(self.output_path))
        self.assertEqual(report["fit"]["pair_count"], 6)
        self.assertEqual(report["validation"]["pair_count"], 2)
        for name in ("rms_px", "median_px", "p95_px", "max_px"):
            self.assertLess(report["validation"][name], 1e-5)
        for residual in report["mount_comparison"]["residual_m"]:
            self.assertAlmostEqual(residual, 0.0, places=5)
        calibration = load_calibration(self.output_path)
        self.assertAlmostEqual(calibration.translation_radar_to_camera_m[0], TRANSLATION[0], places=5)
        self.assertAlmostEqual(calibration.rms_reprojection_error_px, report["fit"]["rms_px"])

    def test_competitor_cannot_occupy_output_during_report_publication(self):
        from tools.fusion import calibrate_radar_camera as solver

        prepared = self.run_solver(output_path=self.path / "setup.json")
        self.assertEqual(prepared.returncode, 0, prepared.stderr)
        argv = [str(SOLVER), str(self.session_path), str(self.intrinsics_path), str(self.output_path)]
        original_dumps = json.dumps
        attempts = []

        def compete_when_report_serializes(value, *args, **kwargs):
            if isinstance(value, dict) and "validation_passed" in value and not attempts:
                try:
                    with self.output_path.open("x", encoding="utf-8") as stream:
                        stream.write("foreign output")
                except FileExistsError:
                    attempts.append("blocked")
                else:
                    attempts.append("occupied")
            return original_dumps(value, *args, **kwargs)

        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            solver.json, "dumps", side_effect=compete_when_report_serializes
        ):
            solver.main()

        self.assertEqual(attempts, ["blocked"])
        self.assertTrue(json.loads(self.report_path.read_text(encoding="utf-8"))["validation_passed"])
        self.assertNotEqual(self.output_path.read_bytes(), b"foreign output")
        self.assertEqual(load_calibration(self.output_path).image_width, IMAGE_SIZE[0])

    def test_runtime_publication_failure_removes_both_owned_products(self):
        from tools.fusion import calibrate_radar_camera as solver

        prepared = self.run_solver(output_path=self.path / "setup.json")
        self.assertEqual(prepared.returncode, 0, prepared.stderr)
        argv = [str(SOLVER), str(self.session_path), str(self.intrinsics_path), str(self.output_path)]
        original_dumps = json.dumps

        def fail_runtime_json(value, *args, **kwargs):
            if isinstance(value, dict) and "radar_to_camera" in value:
                raise OSError("synthetic runtime publication failure")
            return original_dumps(value, *args, **kwargs)

        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            solver.json, "dumps", side_effect=fail_runtime_json
        ), self.assertRaisesRegex(SystemExit, "could not publish output"):
            solver.main()

        self.assertFalse(self.output_path.exists())
        self.assertFalse(self.report_path.exists())

    def test_report_publication_failure_removes_written_runtime_and_report(self):
        from tools.fusion import calibrate_radar_camera as solver

        prepared = self.run_solver(output_path=self.path / "setup.json")
        self.assertEqual(prepared.returncode, 0, prepared.stderr)
        argv = [str(SOLVER), str(self.session_path), str(self.intrinsics_path), str(self.output_path)]
        original_dumps = json.dumps

        def fail_report_json(value, *args, **kwargs):
            if isinstance(value, dict) and "validation_passed" in value:
                raise OSError("synthetic report publication failure")
            return original_dumps(value, *args, **kwargs)

        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            solver.json, "dumps", side_effect=fail_report_json
        ), self.assertRaisesRegex(SystemExit, "could not publish report"):
            solver.main()

        self.assertFalse(self.output_path.exists())
        self.assertFalse(self.report_path.exists())

    def test_report_reservation_race_preserves_foreign_report_and_cleans_output(self):
        from tools.fusion import calibrate_radar_camera as solver

        prepared = self.run_solver(output_path=self.path / "setup.json")
        self.assertEqual(prepared.returncode, 0, prepared.stderr)
        argv = [str(SOLVER), str(self.session_path), str(self.intrinsics_path), str(self.output_path)]
        original_open = Path.open

        def compete_for_report(path, mode="r", *args, **kwargs):
            if path == self.report_path and mode == "x+b":
                with original_open(path, "xb") as stream:
                    stream.write(b"foreign report")
            return original_open(path, mode, *args, **kwargs)

        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            Path, "open", autospec=True, side_effect=compete_for_report
        ), self.assertRaisesRegex(SystemExit, "report path already exists"):
            solver.main()

        self.assertFalse(self.output_path.exists())
        self.assertEqual(self.report_path.read_bytes(), b"foreign report")

    def test_reservation_identity_failure_cleans_all_owned_paths_and_allows_retry(self):
        from tools.fusion import calibrate_radar_camera as solver

        prepared = self.run_solver(output_path=self.path / "setup.json")
        self.assertEqual(prepared.returncode, 0, prepared.stderr)
        session_before = self.session_path.read_bytes()
        intrinsics_before = self.intrinsics_path.read_bytes()
        original_fstat = os.fstat

        for failed_reservation in range(1, 4):
            with self.subTest(failed_reservation=failed_reservation):
                output_path = self.path / f"fstat-{failed_reservation}.json"
                report_path = output_path.with_suffix(".report.json")
                completion_path = output_path.with_suffix(".complete.json")
                argv = [str(SOLVER), str(self.session_path), str(self.intrinsics_path), str(output_path)]
                calls = 0

                def fail_selected_fstat(file_descriptor):
                    nonlocal calls
                    calls += 1
                    if calls == failed_reservation:
                        raise OSError("synthetic reservation identity failure")
                    return original_fstat(file_descriptor)

                with mock.patch.object(sys, "argv", argv), mock.patch.object(
                    solver.os, "fstat", side_effect=fail_selected_fstat
                ), self.assertRaisesRegex(SystemExit, "could not reserve"):
                    solver.main()

                self.assertFalse(output_path.exists())
                self.assertFalse(report_path.exists())
                self.assertFalse(completion_path.exists())
                self.assertEqual(self.session_path.read_bytes(), session_before)
                self.assertEqual(self.intrinsics_path.read_bytes(), intrinsics_before)

                with mock.patch.object(sys, "argv", argv):
                    solver.main()
                self.assertTrue(solver.verify_completion(output_path))

    def test_bad_validation_writes_report_but_no_runtime(self):
        from tools.fusion.calibrate_radar_camera import verify_completion

        result = self.run_solver(validation_shift=100.0)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output_path.exists())
        self.assertFalse(self.complete_path.exists())
        self.assertFalse(verify_completion(self.output_path))
        report = json.loads(self.report_path.read_text(encoding="utf-8"))
        self.assertFalse(report["validation_passed"])
        self.assertLess(report["fit"]["rms_px"], 1e-5)
        self.assertGreater(report["validation"]["p95_px"], 20.0)
        self.assertGreater(report["validation"]["median_px"], 8.0)

    def test_existing_completion_blocks_run_without_touching_inputs_or_other_products(self):
        foreign = b'{"operator_note":"keep this manifest name"}'
        self.complete_path.write_bytes(foreign)
        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("completion path already exists", rejected.stderr)
        self.assertEqual(self.complete_path.read_bytes(), foreign)
        self.assertFalse(self.output_path.exists())
        self.assertFalse(self.report_path.exists())
        self.assertEqual(self.session_path.read_bytes(), self.session_bytes)

    def test_completion_reservation_race_preserves_foreign_manifest(self):
        from tools.fusion import calibrate_radar_camera as solver

        prepared = self.run_solver(output_path=self.path / "setup.json")
        self.assertEqual(prepared.returncode, 0, prepared.stderr)
        argv = [str(SOLVER), str(self.session_path), str(self.intrinsics_path), str(self.output_path)]
        original_open = Path.open

        def compete_for_completion(path, mode="r", *args, **kwargs):
            if path == self.complete_path and mode == "x+b":
                with original_open(path, "xb") as stream:
                    stream.write(b"foreign manifest")
            return original_open(path, mode, *args, **kwargs)

        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            Path, "open", autospec=True, side_effect=compete_for_completion
        ), self.assertRaisesRegex(SystemExit, "completion path already exists"):
            solver.main()

        self.assertFalse(self.output_path.exists())
        self.assertFalse(self.report_path.exists())
        self.assertEqual(self.complete_path.read_bytes(), b"foreign manifest")

    def test_completion_verifier_rejects_tampered_or_missing_products(self):
        from tools.fusion.calibrate_radar_camera import verify_completion

        completed = self.run_solver()
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertTrue(verify_completion(self.output_path))
        for path in (self.output_path, self.report_path):
            original = path.read_bytes()
            with self.subTest(path=path.name, change="tampered"):
                path.write_bytes(original + b" ")
                self.assertFalse(verify_completion(self.output_path))
            path.write_bytes(original)
            with self.subTest(path=path.name, change="missing"):
                path.unlink()
                self.assertFalse(verify_completion(self.output_path))
            path.write_bytes(original)
        original_manifest = self.complete_path.read_bytes()
        self.complete_path.write_bytes(b"{malformed")
        self.assertFalse(verify_completion(self.output_path))
        self.complete_path.unlink()
        self.assertFalse(verify_completion(self.output_path))
        self.complete_path.write_bytes(original_manifest)
        report = json.loads(self.report_path.read_text(encoding="utf-8"))
        report["artifact_id"] = "different-run"
        self.report_path.write_text(json.dumps(report), encoding="utf-8")
        manifest = json.loads(self.complete_path.read_text(encoding="utf-8"))
        manifest["report_sha256"] = hashlib.sha256(self.report_path.read_bytes()).hexdigest()
        self.complete_path.write_text(json.dumps(manifest), encoding="utf-8")
        self.assertFalse(verify_completion(self.output_path))

    def test_late_report_error_and_cleanup_failure_cannot_prove_completion(self):
        from tools.fusion import calibrate_radar_camera as solver

        prepared = self.run_solver(output_path=self.path / "setup.json")
        self.assertEqual(prepared.returncode, 0, prepared.stderr)
        argv = [str(SOLVER), str(self.session_path), str(self.intrinsics_path), str(self.output_path)]
        original_write = solver._ReservedProduct.write
        original_discard = solver._ReservedProduct.discard

        def fail_after_report_bytes(product, value):
            original_write(product, value)
            if product.kind == "report":
                raise OSError("synthetic late report error")

        def fail_report_cleanup(product):
            if product.kind == "report":
                product.close()
                raise OSError("synthetic report cleanup failure")
            return original_discard(product)

        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            solver._ReservedProduct, "write", fail_after_report_bytes
        ), mock.patch.object(solver._ReservedProduct, "discard", fail_report_cleanup), self.assertRaisesRegex(
            SystemExit, "could not clean reserved paths"
        ):
            solver.main()

        self.assertTrue(json.loads(self.report_path.read_text(encoding="utf-8"))["validation_passed"])
        self.assertFalse(self.output_path.exists())
        self.assertFalse(self.complete_path.exists())
        self.assertFalse(solver.verify_completion(self.output_path))

    def test_late_completion_error_can_only_prove_intact_pair(self):
        from tools.fusion import calibrate_radar_camera as solver

        prepared = self.run_solver(output_path=self.path / "setup.json")
        self.assertEqual(prepared.returncode, 0, prepared.stderr)
        argv = [str(SOLVER), str(self.session_path), str(self.intrinsics_path), str(self.output_path)]
        original_write = solver._ReservedProduct.write

        def fail_after_manifest_bytes(product, value):
            written = original_write(product, value)
            if product.kind == "completion":
                raise OSError("synthetic late manifest error")
            return written

        def fail_all_cleanup(product):
            product.close()
            raise OSError("synthetic cleanup failure")

        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            solver._ReservedProduct, "write", fail_after_manifest_bytes
        ), mock.patch.object(solver._ReservedProduct, "discard", fail_all_cleanup), self.assertRaisesRegex(
            SystemExit, "could not clean reserved paths"
        ):
            solver.main()

        self.assertTrue(solver.verify_completion(self.output_path))
        self.output_path.write_bytes(self.output_path.read_bytes() + b" ")
        self.assertFalse(solver.verify_completion(self.output_path))

    def test_success_then_reuse_same_path_rejects_and_preserves_old_pair(self):
        first = self.run_solver()
        self.assertEqual(first.returncode, 0, first.stderr)
        old_runtime = self.output_path.read_bytes()
        old_report = self.report_path.read_bytes()
        old_completion = self.complete_path.read_bytes()

        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("choose a new output path/run directory", rejected.stderr)
        self.assertEqual(self.output_path.read_bytes(), old_runtime)
        self.assertEqual(self.report_path.read_bytes(), old_report)
        self.assertEqual(self.complete_path.read_bytes(), old_completion)

    def test_report_only_from_failed_validation_blocks_reuse(self):
        first = self.run_solver(validation_shift=100.0)
        self.assertNotEqual(first.returncode, 0)
        self.assertFalse(self.output_path.exists())
        old_report = self.report_path.read_bytes()

        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("choose a new output path/run directory", rejected.stderr)
        self.assertFalse(self.output_path.exists())
        self.assertEqual(self.report_path.read_bytes(), old_report)

    def test_help_requires_a_new_output_path(self):
        result = subprocess.run(
            [sys.executable, str(SOLVER), "--help"], cwd=ROOT,
            capture_output=True, text=True, check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("new output path/run directory", result.stdout)

    def test_read_only_report_rejects_and_preserves_old_pair(self):
        first = self.run_solver()
        self.assertEqual(first.returncode, 0, first.stderr)
        old_runtime = self.output_path.read_bytes()
        old_report = self.report_path.read_bytes()

        def restore_writable():
            for path in self.path.iterdir():
                if path.is_file():
                    os.chmod(path, stat.S_IREAD | stat.S_IWRITE)

        self.addCleanup(restore_writable)
        os.chmod(self.report_path, stat.S_IREAD)
        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("choose a new output path/run directory", rejected.stderr)
        self.assertEqual(self.output_path.read_bytes(), old_runtime)
        self.assertEqual(self.report_path.read_bytes(), old_report)

    def test_unrelated_output_is_preserved_before_invalid_session(self):
        output_path = self.path / "operator_notes.txt"
        notes = b"keep these operator notes"
        output_path.write_bytes(notes)

        rejected = self.run_solver(validation_count=0, output_path=output_path)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("output path already exists", rejected.stderr)
        self.assertEqual(output_path.read_bytes(), notes)
        self.assertFalse(output_path.with_suffix(".report.json").exists())
        self.assertEqual(self.session_path.read_bytes(), self.session_bytes)

    def test_output_hard_link_to_unrelated_file_is_preserved(self):
        target = self.path / "operator_notes.json"
        notes = b'{"operator_note":"keep"}'
        target.write_bytes(notes)
        self.hard_link_or_skip(target, self.output_path)

        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("output path already exists", rejected.stderr)
        self.assertEqual(self.output_path.read_bytes(), notes)
        self.assertEqual(target.read_bytes(), notes)
        self.assertFalse(self.report_path.exists())

    def test_directory_output_preserves_old_report(self):
        first = self.run_solver()
        self.assertEqual(first.returncode, 0, first.stderr)
        old_report = self.report_path.read_bytes()
        self.output_path.unlink()
        self.output_path.mkdir()

        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("output path already exists", rejected.stderr)
        self.assertTrue(self.output_path.is_dir())
        self.assertEqual(self.report_path.read_bytes(), old_report)

    def test_directory_report_blocks_fresh_output(self):
        self.report_path.mkdir()
        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("report path already exists", rejected.stderr)
        self.assertFalse(self.output_path.exists())
        self.assertTrue(self.report_path.is_dir())

    def test_output_symlink_to_unrelated_file_is_preserved(self):
        target = self.path / "operator_notes.json"
        notes = b'{"operator_note":"keep"}'
        target.write_bytes(notes)
        self.symlink_or_skip(target, self.output_path)

        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("output path already exists", rejected.stderr)
        self.assertTrue(self.output_path.is_symlink())
        self.assertEqual(target.read_bytes(), notes)
        self.assertFalse(self.report_path.exists())

    def test_broken_report_symlink_blocks_fresh_output(self):
        self.symlink_or_skip(self.path / "missing-report.json", self.report_path)
        self.assertFalse(self.report_path.exists())

        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("report path already exists", rejected.stderr)
        self.assertTrue(self.report_path.is_symlink())
        self.assertFalse(self.output_path.exists())

    def test_broken_report_symlink_check_precedes_invalid_session(self):
        from tools.fusion.calibrate_radar_camera import main

        self.run_solver(validation_count=0, output_path=self.path / "setup.json")
        argv = [str(SOLVER), str(self.session_path), str(self.intrinsics_path), str(self.output_path)]
        original_is_symlink = Path.is_symlink

        def simulate_broken_report_link(path):
            return path == self.report_path or original_is_symlink(path)

        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            Path, "is_symlink", simulate_broken_report_link
        ), self.assertRaisesRegex(SystemExit, "report path already exists"):
            main()

        self.assertFalse(self.output_path.exists())
        self.assertFalse(self.report_path.exists())

    def test_unrelated_report_preserves_old_runtime(self):
        first = self.run_solver()
        self.assertEqual(first.returncode, 0, first.stderr)
        old_runtime = self.output_path.read_bytes()
        unrelated = b'{"note":"not a calibration report"}'
        self.report_path.write_bytes(unrelated)

        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("choose a new output path/run directory", rejected.stderr)
        self.assertEqual(self.output_path.read_bytes(), old_runtime)
        self.assertEqual(self.report_path.read_bytes(), unrelated)

    def test_malformed_output_preserves_old_report(self):
        first = self.run_solver()
        self.assertEqual(first.returncode, 0, first.stderr)
        old_report = self.report_path.read_bytes()
        malformed = b"{bad json"
        self.output_path.write_bytes(malformed)

        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("output path already exists", rejected.stderr)
        self.assertEqual(self.output_path.read_bytes(), malformed)
        self.assertEqual(self.report_path.read_bytes(), old_report)

    def test_unknown_json_output_preserves_old_report(self):
        first = self.run_solver()
        self.assertEqual(first.returncode, 0, first.stderr)
        old_report = self.report_path.read_bytes()
        unrelated = b'{"schema_version":1,"operator_note":"keep"}'
        self.output_path.write_bytes(unrelated)

        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("output path already exists", rejected.stderr)
        self.assertEqual(self.output_path.read_bytes(), unrelated)
        self.assertEqual(self.report_path.read_bytes(), old_report)

    def test_malformed_report_preserves_old_runtime(self):
        first = self.run_solver()
        self.assertEqual(first.returncode, 0, first.stderr)
        old_runtime = self.output_path.read_bytes()
        malformed = b"{bad json"
        self.report_path.write_bytes(malformed)

        rejected = self.run_solver(validation_count=0)
        self.assertNotEqual(rejected.returncode, 0)
        self.assertIn("choose a new output path/run directory", rejected.stderr)
        self.assertEqual(self.output_path.read_bytes(), old_runtime)
        self.assertEqual(self.report_path.read_bytes(), malformed)

    def test_bad_intrinsics_on_fresh_paths_leaves_no_products(self):
        bad_intrinsics = b"{bad json"
        failed = self.run_solver(before_run=lambda: self.intrinsics_path.write_bytes(bad_intrinsics))
        self.assertNotEqual(failed.returncode, 0)
        self.assertFalse(self.output_path.exists())
        self.assertFalse(self.report_path.exists())
        self.assertFalse(self.complete_path.exists())
        self.assertEqual(self.intrinsics_path.read_bytes(), bad_intrinsics)
        self.assertEqual(self.session_path.read_bytes(), self.session_bytes)

    def test_size_mismatch_on_fresh_paths_leaves_no_products(self):
        def mismatch():
            source = json.loads(self.intrinsics_path.read_text(encoding="utf-8"))
            source["image_size"] = [640, 480]
            self.intrinsics_path.write_text(json.dumps(source), encoding="utf-8")

        failed = self.run_solver(before_run=mismatch)
        self.assertNotEqual(failed.returncode, 0)
        self.assertIn("image size differs", failed.stderr)
        self.assertFalse(self.output_path.exists())
        self.assertFalse(self.report_path.exists())
        self.assertEqual(json.loads(self.intrinsics_path.read_text(encoding="utf-8"))["image_size"], [640, 480])
        self.assertEqual(self.session_path.read_bytes(), self.session_bytes)

    def test_solve_error_on_fresh_paths_leaves_no_products(self):
        import cv2
        from tools.fusion.calibrate_radar_camera import main

        prepared = self.run_solver(output_path=self.path / "setup.json")
        self.assertEqual(prepared.returncode, 0, prepared.stderr)
        argv = [str(SOLVER), str(self.session_path), str(self.intrinsics_path), str(self.output_path)]

        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            cv2, "solvePnP", side_effect=cv2.error("synthetic solve failure")
        ), self.assertRaisesRegex(cv2.error, "synthetic solve failure"):
            main()

        self.assertFalse(self.output_path.exists())
        self.assertFalse(self.report_path.exists())
        self.assertEqual(self.session_path.read_bytes(), self.session_bytes)
        self.assertTrue(self.intrinsics_path.exists())

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

    def test_rejects_completion_hard_link_to_session(self):
        result = self.run_solver(before_run=lambda: self.hard_link_or_skip(
            self.session_path, self.complete_path))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("completion path aliases an input", result.stderr)
        self.assertEqual(self.session_path.read_bytes(), self.session_bytes)
        self.assertFalse(self.output_path.exists())
        self.assertFalse(self.report_path.exists())

    def test_rejects_output_and_report_hard_link_without_changing_either(self):
        original = b"existing calibration"

        def link_outputs():
            self.output_path.write_bytes(original)
            self.hard_link_or_skip(self.output_path, self.report_path)

        result = self.run_solver(before_run=link_outputs)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("output and report paths alias each other", result.stderr)
        self.assertEqual(self.output_path.read_bytes(), original)
        self.assertEqual(self.report_path.read_bytes(), original)

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
        self.assertFalse(self.complete_path.exists())

    def test_no_mount_keeps_comparison_optional(self):
        result = self.run_solver(mount=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(self.report_path.read_text(encoding="utf-8"))
        self.assertIsNone(report["mount_comparison"]["measured_camera_center_in_radar_m"])
        self.assertIsNone(report["mount_comparison"]["residual_m"])


if __name__ == "__main__":
    unittest.main()
