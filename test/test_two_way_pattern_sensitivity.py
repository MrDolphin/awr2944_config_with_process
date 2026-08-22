import unittest
from pathlib import Path


class TwoWayPatternSensitivityTest(unittest.TestCase):
    def test_two_way_weight_is_generated(self):
        from simulation.run_v04_125_two_way_pattern_sensitivity import run
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/hardware/awr2944pev/v04_120_masked_microfacet_clutter/masked_microfacet_clutter.csv"), Path("simulation/hardware/awr2944pev/antenna_pattern_azimuth.csv"), Path("simulation/hardware/awr2944pev/antenna_pattern_elevation.csv"), Path(directory) / "out")
            self.assertEqual(result["case_count"], 5)
            self.assertTrue((Path(directory) / "out" / "two_way_pattern_sensitivity.csv").exists())


if __name__ == "__main__":
    unittest.main()
