import tempfile
import unittest
from pathlib import Path


class GridBeamAoAV129Test(unittest.TestCase):
    def test_grid_beam_self_model_is_accurate(self):
        from simulation.run_v04_129_grid_beam_aoa import run

        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), Path("simulation/hardware/awr2944pev/v04_127_rf_endpoint_candidates/rf_endpoint_candidates.csv"), Path(directory) / "out")
            with (Path(directory) / "out" / "grid_beam_metrics.csv").open(encoding="utf-8") as handle:
                metrics = list(__import__("csv").DictReader(handle))
        self.assertEqual(result["truth_case_count"], 560)
        ideal = next(row for row in metrics if row["actual_model"] == "ideal_half_lambda" and row["assumed_model"] == "ideal_half_lambda")
        self.assertLess(float(ideal["combined_rmse_deg"]), 1.1)


if __name__ == "__main__":
    unittest.main()
