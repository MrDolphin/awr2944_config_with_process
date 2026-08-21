import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_33_known_angle_fixture import run
from simulation.run_v04_54_multiscene_aoa_validation import run as validate_many


class MultiSceneAoaValidationTests(unittest.TestCase):
    def test_multiple_known_angles_produce_aggregate_candidates(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); first = root / "a.h5"; second = root / "b.h5"; output = root / "out"
            run(first, azimuth_deg=10.0, elevation_deg=2.0, slant_range_m=20.0, radial_velocity_mps=0.0, frames=8, samples=64)
            run(second, azimuth_deg=30.0, elevation_deg=5.0, slant_range_m=40.0, radial_velocity_mps=0.0, frames=8, samples=64)
            summary = validate_many([first, second], output)
            self.assertEqual(summary["scene_count"], 2)
            self.assertEqual(summary["candidate_count"], 576)
            self.assertTrue((output / "multi_scene_candidate_metrics.csv").is_file())


if __name__ == "__main__":
    unittest.main()
