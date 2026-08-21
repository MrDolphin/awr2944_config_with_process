import unittest

from simulation.run_v04_55_aoa_acceptance_report import build_report


class AoaAcceptanceReportTests(unittest.TestCase):
    def test_synthetic_pass_does_not_claim_hardware_ready(self):
        multi = {"scene_count": 3, "candidate_count": 576, "geometry_source": "simulation.v04", "best_candidates": [{"combined_rmse_deg": 0.0, "max_abs_azimuth_error_deg": 0.0, "max_abs_elevation_error_deg": 0.0, "identity_order": True}], "identity_candidate": {"combined_rmse_deg": 0.0}}
        hdf5 = {"source_is_hardware_measurement": False, "channel_order_verified": False, "input_metadata": {"truth_azimuth_deg": 10.0, "truth_elevation_deg": 2.0, "source_type": "synthetic", "calibration_status": "synthetic"}}
        result = build_report(multi, hdf5)
        self.assertTrue(result["synthetic_regression_pass"])
        self.assertFalse(result["real_hardware_aoa_ready"])


if __name__ == "__main__":
    unittest.main()
