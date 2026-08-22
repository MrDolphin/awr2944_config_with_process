import tempfile
import unittest
from pathlib import Path


class CfarPhysicsJoinTest(unittest.TestCase):
    def test_joins_common_cases(self):
        from simulation.run_v04_121_cfar_physics_join import run
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); cfar = root / "cfar.csv"; physics = root / "physics.csv"
            cfar.write_text("case_id,scenario_id,frames,point_count\nss0_flat,pfa1e3_train4,2,4\n", encoding="utf-8")
            physics.write_text("case_id,three_db_facets,six_db_facets,mean_three_db_scatter_proxy,three_db_doppler_min_hz,three_db_doppler_max_hz,rms_three_db_radial_velocity_mps\nss0_flat,2,3,0.1,-1,1,0.2\n", encoding="utf-8")
            result = run(cfar, physics, root / "out")
            self.assertEqual(result["row_count"], 1)


if __name__ == "__main__":
    unittest.main()
