import json
import tempfile
import unittest
from pathlib import Path

from tools.fusion.calibration import CalibrationError, load_calibration
from tools.fusion.projection import project_radar_point
from sensor_pose import SensorPose


def payload():
    return {
        "schema_version": 1,
        "image_size": [1280, 720],
        "camera_matrix": {"fx": 800.0, "fy": 800.0, "cx": 640.0, "cy": 360.0},
        "distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
        "radar_to_camera": {"rotation_3x3": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "translation_m": [0, 0, 0]},
        "mount_mode": "co_rotating", "calibrated_at": "2026-01-01T00:00:00Z", "rms_reprojection_error_px": 1.0,
    }


class CalibrationProjectionTests(unittest.TestCase):
    def load(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            return load_calibration(path)

    def test_valid_calibration_and_optical_axis_projection(self):
        calibration = self.load(payload())
        self.assertEqual(project_radar_point((0, 0, 5), calibration), (640.0, 360.0, 5.0))

    def test_rejects_non_orthonormal_rotation(self):
        data = payload(); data["radar_to_camera"]["rotation_3x3"][0][0] = 2
        with self.assertRaises(CalibrationError): self.load(data)

    def test_repository_example_is_deliberately_not_deployable(self):
        example = Path(__file__).resolve().parents[1] / "Config" / "radar_camera_calibration.example.json"
        with self.assertRaises(CalibrationError):
            load_calibration(example)

    def test_omits_points_behind_camera_and_optionally_clips_image_bounds(self):
        calibration = self.load(payload())
        self.assertIsNone(project_radar_point((0, 0, -1), calibration))
        self.assertIsNone(project_radar_point((10, 0, 1), calibration, clip_to_image=True))
        self.assertEqual(project_radar_point((10, 0, 1), calibration), (8640.0, 360.0, 1.0))

    def test_applies_radial_and_tangential_distortion(self):
        data = payload(); data["distortion"] = [0.1, 0, 0.01, -0.01, 0]
        calibration = self.load(data)
        u, v, depth = project_radar_point((1, 1, 10), calibration)
        self.assertAlmostEqual(u, 720.0, places=5); self.assertAlmostEqual(v, 440.32, places=5); self.assertEqual(depth, 10.0)

    def test_fixed_camera_applies_measured_yaw_but_co_rotating_does_not(self):
        fixed = payload(); fixed["mount_mode"] = "fixed_camera"
        pose = SensorPose(1, 90.0, 0.0, 0.0, "encoder")
        self.assertEqual(project_radar_point((1, 0, 5), self.load(fixed), pose), (640.0, 520.0, 5.0))
        self.assertEqual(project_radar_point((1, 0, 5), self.load(payload()), pose), (800.0, 360.0, 5.0))


if __name__ == "__main__": unittest.main()
