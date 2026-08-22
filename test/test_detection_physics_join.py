import tempfile
import unittest
from pathlib import Path


class DetectionPhysicsJoinTest(unittest.TestCase):
    def test_common_case_join(self):
        from simulation.run_v04_122_detection_physics_join import run
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); boundary = root / "boundary.csv"; physics = root / "physics.csv"
            boundary.write_text("case_id,detection_probability,mean_false_alarms_per_frame\nss2_normal,0.5,1\n", encoding="utf-8")
            physics.write_text("case_id,three_db_facets,six_db_facets,mean_three_db_scatter_proxy,three_db_doppler_min_hz,three_db_doppler_max_hz,rms_three_db_radial_velocity_mps\nss2_normal,3,4,0.2,-2,2,0.1\n", encoding="utf-8")
            self.assertEqual(run(boundary, physics, root / "out")["case_count"], 1)


if __name__ == "__main__":
    unittest.main()
