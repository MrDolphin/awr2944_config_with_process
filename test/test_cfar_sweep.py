import unittest
from pathlib import Path

from simulation.run_v04_45_cfar_sweep import SCENARIOS, run_case


class CfarSweepTests(unittest.TestCase):
    def test_sweep_has_multiple_parameter_scenarios(self):
        self.assertGreaterEqual(len(SCENARIOS), 6)
        path = Path("simulation/hardware/awr2944pev/v04_43_sea_range_doppler/ss3_upper_range_doppler.h5")
        result = run_case(path, SCENARIOS[0])
        self.assertEqual(result["frames"], 41)
        self.assertGreaterEqual(result["point_count"], 0)
        self.assertIsNotNone(result["cfar_alpha"])


if __name__ == "__main__":
    unittest.main()
