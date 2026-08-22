import tempfile
import unittest
from pathlib import Path


class SeaClutterVisualizationV131Test(unittest.TestCase):
    def test_points_and_summary_are_written(self):
        from simulation.run_v04_131_sea_clutter_visualization import run

        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/hardware/awr2944pev/v04_130_sea_clutter_grid_aoa/sea_clutter_grid_aoa.csv"), Path("simulation/hardware/awr2944pev/v04_120_masked_microfacet_clutter/masked_microfacet_clutter.csv"), Path(directory) / "out")
        self.assertEqual(result["case_count"], 5)
        self.assertEqual(result["model_count"], 4)
        self.assertGreater(result["source_rows"], 0)


if __name__ == "__main__":
    unittest.main()
