import json
import tempfile
import unittest
from pathlib import Path

import h5py

from simulation.run_v04_33_known_angle_fixture import run


class KnownAngleFixtureTest(unittest.TestCase):
    def test_fixture_records_truth_and_explicitly_stays_synthetic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "known_angle.h5"
            geometry = root / "geometry.csv"
            geometry.write_text("rx_index,tx_index,row,column,azimuth_spacing_lambda,elevation_spacing_lambda\n0,0,0,0,0.5,0.8\n", encoding="utf-8")
            summary = run(output, azimuth_deg=12.0, elevation_deg=6.0, slant_range_m=20.0, radial_velocity_mps=0.0, frames=4, samples=32, geometry_csv=geometry)
            self.assertEqual(summary["virtual_shape"], [4, 32, 4, 4])
            self.assertFalse(summary["real_measurement"])
            with h5py.File(output, "r") as handle:
                self.assertEqual(handle.attrs["source_type"], "synthetic_known_angle_regression_only")
                self.assertTrue(bool(handle.attrs["channel_order_verified"]))
                self.assertEqual(handle.attrs["geometry_source"], str(geometry.resolve()))
                self.assertEqual(handle["/recovered/virtual_iq"].shape, (4, 32, 4, 4))
            payload = json.loads(output.with_suffix(".json").read_text(encoding="utf-8"))
            self.assertEqual(payload["truth_azimuth_deg"], 12.0)
            self.assertEqual(payload["calibration_status"], "synthetic_unity_not_measured")


if __name__ == "__main__":
    unittest.main()
