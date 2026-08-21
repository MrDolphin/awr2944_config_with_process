import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_33_known_angle_fixture import run
from simulation.run_v04_58_measured_package_pipeline import run as pipeline


class MeasuredPackagePipelineTests(unittest.TestCase):
    def test_hdf5_scene_is_dispatched_but_synthetic_is_not_hardware_ready(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); capture = root / "scene.h5"; truth = root / "truth.json"; manifest = root / "manifest.json"; output = root / "out"
            run(capture, azimuth_deg=10.0, elevation_deg=2.0, slant_range_m=20.0, radial_velocity_mps=0.0, frames=8, samples=64)
            truth.write_text(json.dumps({"azimuth_deg": 10, "elevation_deg": 2, "range_m": 20}), encoding="utf-8")
            manifest.write_text(json.dumps({"coverage": {"azimuth_deg": [10], "elevation_deg": [2], "range_m": [20]}, "scenes": [{"scene_id": "s1", "capture": "scene.h5", "cfg": "missing.cfg", "truth": "truth.json"}]}), encoding="utf-8")
            result = pipeline(manifest, output)
            self.assertEqual(result["hdf5_processed_count"], 1)
            self.assertFalse(result["hardware_ready"])


if __name__ == "__main__":
    unittest.main()
