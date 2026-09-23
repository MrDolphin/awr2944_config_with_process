import json
import tempfile
import unittest
from pathlib import Path

from tools.fusion.calibration_session import (
    SessionError,
    camera_center_in_radar,
    load_session,
    mount_residual,
)


def write_session(*, fit_count=6, validation_count=1, mount_dz=-0.10, **changes):
    samples = []
    for set_name, count in (("fit", fit_count), ("validation", validation_count)):
        for index in range(count):
            samples.append({
                "id": f"{set_name}-{index}",
                "set": set_name,
                "radar": {"frame_num": index + 1, "point_index": index,
                          "x": float(index), "y": 2.0, "z": 3.0},
                "camera": {"frame_id": f"camera-{index}", "u": 100.0, "v": 200.0},
                "sync_offset_ms": 0.5,
                "timestamp": "2026-09-23T00:00:00Z",
            })
    mount = None if mount_dz is None else {
        "dx_m": 0.0, "dy_m": 0.0, "dz_m": mount_dz,
        "uncertainty_m": 0.01, "reference": "measured optical centre to phase centre",
    }
    payload = {
        "schema_version": 1,
        "camera_image_size": [1280, 720],
        "radar_id": "AWR2944P-01",
        "camera_id": "CAM-01",
        "mount_measurement": mount,
        "samples": samples,
    }
    payload.update(changes)
    handle = tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", suffix=".json", delete=False)
    with handle:
        json.dump(payload, handle)
    return Path(handle.name)


class CalibrationSessionTests(unittest.TestCase):
    def tearDown(self):
        for path in getattr(self, "paths", []):
            path.unlink(missing_ok=True)

    def load(self, **kwargs):
        self.paths = getattr(self, "paths", [])
        path = write_session(**kwargs)
        self.paths.append(path)
        return load_session(path)

    def test_session_separates_fit_validation_and_keeps_mount(self):
        session = self.load(fit_count=6, validation_count=2)
        self.assertEqual(len(session.fit_pairs()), 6)
        self.assertEqual(len(session.validation_pairs()), 2)
        self.assertEqual(session.mount_measurement, (0.0, 0.0, -0.10))
        with self.assertRaises((AttributeError, TypeError)):
            session.radar_id = "changed"

    def test_session_rejects_missing_validation_and_non_finite_mount(self):
        path = write_session(validation_count=0)
        self.paths = [path]
        with self.assertRaisesRegex(SessionError, "at least one validation"):
            load_session(path)
        path = write_session(mount_dz="bad")
        self.paths.append(path)
        with self.assertRaisesRegex(SessionError, "finite"):
            load_session(path)

    def test_session_rejects_schema_dimensions_fit_count_and_identifiers(self):
        for changes, message in (
            ({"schema_version": 2}, "schema"),
            ({"camera_image_size": [0, 720]}, "positive"),
            ({"radar_id": ""}, "radar"),
        ):
            path = write_session(**changes)
            self.paths = getattr(self, "paths", []) + [path]
            with self.subTest(message=message), self.assertRaisesRegex(SessionError, message):
                load_session(path)
        path = write_session(fit_count=5)
        self.paths.append(path)
        with self.assertRaisesRegex(SessionError, "six fit"):
            load_session(path)

    def test_session_rejects_non_finite_sample_fields_and_invalid_mount_optionals(self):
        invalid_cases = []
        for field in ("x", "y", "z"):
            invalid_cases.append(({"radar": {field: float("nan")}}, "finite"))
        for field in ("u", "v"):
            invalid_cases.append(({"camera": {field: float("inf")}}, "finite"))
        invalid_cases.append(({"sync_offset_ms": "nan"}, "finite"))
        for change, message in invalid_cases:
            path = write_session()
            self.paths = getattr(self, "paths", []) + [path]
            payload = json.loads(path.read_text(encoding="utf-8"))
            row = payload["samples"][0]
            for group, fields in change.items():
                if group in ("radar", "camera"):
                    row[group].update(fields)
                else:
                    row[group] = fields
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.subTest(change=change), self.assertRaisesRegex(SessionError, message):
                load_session(path)

        for mount_change, message in (({"yaw_deg": "bad", "pitch_deg": 0, "roll_deg": 0}, "finite"),
                                      ({"uncertainty_m": -0.1}, "non-negative")):
            path = write_session()
            self.paths.append(path)
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["mount_measurement"].update(mount_change)
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.subTest(mount_change=mount_change), self.assertRaisesRegex(SessionError, message):
                load_session(path)

    def test_camera_centre_conversion_and_mount_residual(self):
        rotation = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
        self.assertEqual(camera_center_in_radar(rotation, (0.2, -0.1, 0.3)), (0.1, 0.2, -0.3))
        self.assertEqual(mount_residual((0.0, 0.0, -0.1), (0.1, 0.0, -0.2)), (0.1, 0.0, -0.1))


if __name__ == "__main__":
    unittest.main()
