import unittest
from pathlib import Path


class PhaseCenterSensitivityTest(unittest.TestCase):
    def test_small_grid_run_returns_monotonic_levels(self):
        from simulation.run_v04_116_phase_center_sensitivity import run
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/hardware/awr2944pev/v04_106_provisional_array_geometry/virtual_array_coordinates.csv"), Path(directory) / "out")
            self.assertEqual([row["phase_center_perturbation_mm"] for row in result["rows"]], [0.0, 0.25, 0.5, 1.0])
            self.assertTrue((Path(directory) / "out" / "phase_center_sensitivity.csv").exists())


if __name__ == "__main__":
    unittest.main()
