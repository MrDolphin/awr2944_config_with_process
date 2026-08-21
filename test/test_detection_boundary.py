import unittest

from simulation.run_v04_47_detection_boundary import PFA_VALUES, SEA_CASES, SWEEP_VALUES


class DetectionBoundaryContractTests(unittest.TestCase):
    def test_boundary_sweep_contract_is_finite_and_bounded(self):
        self.assertEqual(SEA_CASES, ("ss2_normal", "ss3_upper"))
        self.assertEqual(PFA_VALUES, (1e-2, 1e-3))
        self.assertGreaterEqual(len(SWEEP_VALUES["snr_db"]), 5)
        self.assertGreaterEqual(len(SWEEP_VALUES["range_m"]), 5)
        self.assertTrue(all(0.0 <= value <= 100.0 for value in SWEEP_VALUES["range_m"]))
        self.assertTrue(all(-60.0 <= value <= 60.0 for value in SWEEP_VALUES["azimuth_deg"]))
        self.assertTrue(all(-20.0 <= value <= 20.0 for value in SWEEP_VALUES["elevation_deg"]))


if __name__ == "__main__":
    unittest.main()
