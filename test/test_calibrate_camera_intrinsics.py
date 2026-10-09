import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "tools" / "camera" / "calibrate_intrinsics.py"


class CameraIntrinsicsCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not SCRIPT.exists():
            return
        spec = importlib.util.spec_from_file_location("calibrate_intrinsics", SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(cls.module)

    def setUp(self):
        if self._testMethodName != "test_calibration_tool_exists" and not SCRIPT.exists():
            self.skipTest("calibration tool has not been implemented yet")

    def test_calibration_tool_exists(self):
        self.assertTrue(SCRIPT.exists(), "camera intrinsics calibration tool is missing")

    def test_builds_solver_compatible_intrinsics_document(self):
        document = self.module.build_intrinsics_document(
            image_size=(1280, 720),
            camera_matrix=((700.0, 0.0, 640.0), (0.0, 702.0, 360.0), (0.0, 0.0, 1.0)),
            distortion=(-0.1, 0.02, 0.001, -0.002, 0.0),
            rms_px=0.42,
            accepted_images=18,
            board_columns=9,
            board_rows=6,
            square_size_mm=20.0,
        )

        self.assertEqual(document["image_size"], [1280, 720])
        self.assertEqual(document["camera_matrix"]["fx"], 700.0)
        self.assertEqual(document["camera_matrix"]["cy"], 360.0)
        self.assertEqual(len(document["distortion"]), 5)
        self.assertEqual(document["calibration_quality"]["accepted_images"], 18)
        self.assertEqual(document["calibration_quality"]["rms_px"], 0.42)

    def test_rejects_too_few_checkerboard_observations(self):
        with self.assertRaisesRegex(ValueError, "at least 12"):
            self.module.require_observation_count(11, 12)

    def test_quality_gate_rejects_implausible_focal_length_despite_low_rms(self):
        if not hasattr(self.module, "evaluate_quality"):
            self.fail("intrinsics quality evaluation is missing")
        document = self.module.build_intrinsics_document(
            image_size=(1280, 720),
            camera_matrix=((3_000_000.0, 0.0, 640.0), (0.0, 2_000_000.0, 360.0), (0.0, 0.0, 1.0)),
            distortion=(0.0, 0.0, 0.0, 0.0, 0.0),
            rms_px=0.1,
            accepted_images=20,
            board_columns=9,
            board_rows=6,
            square_size_mm=20.0,
        )

        quality = self.module.evaluate_quality(document, maximum_rms_px=1.5)

        self.assertFalse(quality["passed"])
        self.assertIn("horizontal_fov_out_of_range", quality["failures"])

    def test_writes_new_json_without_overwriting_evidence(self):
        document = {
            "image_size": [1280, 720],
            "camera_matrix": {"fx": 1, "fy": 1, "cx": 1, "cy": 1},
            "distortion": [0, 0, 0, 0, 0],
        }
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "intrinsics.json"
            self.module.write_new_json(output, document)
            self.assertEqual(json.loads(output.read_text(encoding="utf-8")), document)
            with self.assertRaisesRegex(FileExistsError, "already exists"):
                self.module.write_new_json(output, document)


if __name__ == "__main__":
    unittest.main()
