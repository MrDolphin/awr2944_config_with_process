import csv
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_77_channel_order_sensitivity import run


class ChannelOrderSensitivityTest(unittest.TestCase):
    def test_identity_narrow_grid_is_self_consistent(self):
        root = Path(__file__).resolve().parents[1]
        mapping = root / "simulation/hardware/awr2944pev/antgeometry_mapping.csv"
        with tempfile.TemporaryDirectory() as temp:
            summary = run(mapping, Path(temp))
            self.assertLess(summary["identity_narrow_azimuth_rmse_deg"], 1e-9)
            self.assertLess(summary["identity_narrow_elevation_rmse_deg"], 1e-9)
            with (Path(temp) / "channel_order_sensitivity.csv").open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 16)


if __name__ == "__main__":
    unittest.main()
