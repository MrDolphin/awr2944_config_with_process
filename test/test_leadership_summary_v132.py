import tempfile
import unittest
from pathlib import Path


class LeadershipSummaryV132Test(unittest.TestCase):
    def test_five_cases_and_four_models_are_summarized(self):
        from simulation.run_v04_132_leadership_summary import run

        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/hardware/awr2944pev/v04_131_sea_clutter_visualization/visualization_summary.csv"), Path("simulation/hardware/awr2944pev/v04_130_sea_clutter_grid_aoa/model_summary.csv"), Path(directory) / "out")
        self.assertEqual(result["case_count"], 5)
        self.assertEqual(result["model_count"], 4)
        self.assertFalse(result["real_hardware_validated"])


if __name__ == "__main__":
    unittest.main()
