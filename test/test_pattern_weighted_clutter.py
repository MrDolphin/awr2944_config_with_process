import unittest
from pathlib import Path


class PatternWeightedClutterTest(unittest.TestCase):
    def test_pattern_tables_are_loaded_and_joined(self):
        from simulation.run_v04_124_pattern_weighted_clutter import run
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/hardware/awr2944pev/v04_120_masked_microfacet_clutter/masked_microfacet_clutter.csv"), Path("simulation/hardware/awr2944pev/antenna_pattern_azimuth.csv"), Path("simulation/hardware/awr2944pev/antenna_pattern_elevation.csv"), Path(directory) / "out")
            self.assertEqual(result["pattern_evidence"], "coarse_digitization_from_TI_EVM_user_guide_figures")
            self.assertEqual(result["row_count"], 9840)


if __name__ == "__main__":
    unittest.main()
