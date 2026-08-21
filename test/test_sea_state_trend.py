import csv
import json
import tempfile
import unittest
from pathlib import Path


class SeaStateTrendTest(unittest.TestCase):
    def test_join_preserves_hs_and_wind_metadata(self):
        from simulation.run_v04_109_sea_state_trend import run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            summary = root / "summary.csv"
            summary.write_text("case_id,frames,ideal_half_lambda_azimuth_rmse_deg,provisional_pcb_endpoint_azimuth_rmse_deg,model_azimuth_difference_rmse_deg,model_difference_over_10deg_rate\nss0_flat,1,1,2,3,0.1\nss3_upper,1,4,5,6,0.2\n", encoding="utf-8")
            config = root / "run_config.json"
            config.write_text(json.dumps({"sea_surface": {"wind_direction_deg": 90}, "sea_states": [{"case_id": "ss0_flat", "sea_state": 0, "target_hs_m": 0, "label": "flat"}, {"case_id": "ss3_upper", "sea_state": 3, "target_hs_m": 1, "label": "upper"}]}), encoding="utf-8")
            output = root / "out"
            result = run(summary, config, output)
            self.assertEqual(result["case_count"], 2)
            with (output / "sea_state_geometry_trend.csv").open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[1]["target_hs_m"], "1.0")
            self.assertEqual(rows[0]["wind_direction_deg"], "90.0")
            self.assertTrue((output / "output_analysis.md").exists())


if __name__ == "__main__":
    unittest.main()
