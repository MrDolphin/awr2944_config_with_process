import tempfile
import unittest
from pathlib import Path


class SeaClutterGridAoAV130Test(unittest.TestCase):
    def test_active_facets_are_joined_to_four_models(self):
        from simulation.run_v04_130_sea_clutter_grid_aoa import run

        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/hardware/awr2944pev/v04_120_masked_microfacet_clutter/masked_microfacet_clutter.csv"), Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), Path("simulation/hardware/awr2944pev/v04_127_rf_endpoint_candidates/rf_endpoint_candidates.csv"), Path("simulation/hardware/awr2944pev/antenna_pattern_azimuth.csv"), Path("simulation/hardware/awr2944pev/antenna_pattern_elevation.csv"), Path(directory) / "out")
        self.assertEqual(result["model_count"], 4)
        self.assertGreater(result["active_rows"], 0)
        self.assertEqual(len(result["summaries"]), 4)
        self.assertFalse(result["phase_center_ready"])


if __name__ == "__main__":
    unittest.main()
