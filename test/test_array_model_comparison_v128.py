import tempfile
import unittest
from pathlib import Path


class ArrayModelComparisonV128Test(unittest.TestCase):
    def test_four_models_are_written(self):
        from simulation.run_v04_128_array_model_comparison import run

        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), Path("simulation/hardware/awr2944pev/v04_127_rf_endpoint_candidates/rf_endpoint_candidates.csv"), Path(directory) / "out")
        self.assertEqual(result["metrics_rows"], 16)
        self.assertEqual(len(result["models"]), 4)
        self.assertFalse(result["phase_center_ready"])


if __name__ == "__main__":
    unittest.main()
